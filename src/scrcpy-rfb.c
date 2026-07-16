/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * scrcpy-rfb: bridge the scrcpy 4.x H.264 video and control sockets to one
 * RFB/VNC port. Clients that negotiate the Open H.264 encoding receive the
 * Android packets without transcoding; other VNC clients get Tight/JPEG,
 * ZRLE, Hextile or Raw from a lazily decoded framebuffer.
 */
#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <limits.h>
#include <netinet/in.h>
#ifdef __linux__
#include <netinet/tcp.h>
#endif
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
#include <rfb/keysym.h>
#include <rfb/rfb.h>

#define SCRCPY_CODEC_H264 UINT32_C(0x68323634)
#define SCRCPY_PACKET_FLAG_SESSION (UINT64_C(1) << 63)
#define SCRCPY_PACKET_FLAG_CONFIG (UINT64_C(1) << 62)
#define SCRCPY_PACKET_FLAG_KEY_FRAME (UINT64_C(1) << 61)
#define SCRCPY_POINTER_ID_MOUSE UINT64_MAX
#define SCRCPY_MSG_INJECT_KEYCODE 0
#define SCRCPY_MSG_INJECT_TEXT 1
#define SCRCPY_MSG_INJECT_TOUCH 2
#define SCRCPY_MSG_INJECT_SCROLL 3
#define SCRCPY_MSG_RESET_VIDEO 17

#define FRAME_QUEUE_CAPACITY 90
#define DECODE_QUEUE_CAPACITY 32
#define MAX_PACKET_SIZE (8U * 1024U * 1024U)
#define FALLBACK_TILE_SIZE 32
#define FALLBACK_FAST_UPDATE_US 14000
#define FALLBACK_MEDIUM_UPDATE_US 30000
#define FALLBACK_ADAPT_INTERVAL_NS UINT64_C(1000000000)
#define FALLBACK_TCP_SEND_BUFFER (256 * 1024)
#define FALLBACK_TCP_NOTSENT_LOWAT (64 * 1024)
#define RESET_REQUEST_INTERVAL_NS UINT64_C(1000000000)
#define SCROLL_ROW_SAMPLES 16
#define SCROLL_SAMPLE_ROW_STEP 8
#define SCROLL_MIN_SHIFT 4
#define SCROLL_MAX_SHIFT 160

struct frame {
    uint8_t *data;
    size_t size;
    int key_frame;
    uint64_t sequence;
};

struct client_state {
    uint64_t next_sequence;
    uint64_t ordinary_update_us_ema;
    struct timespec ordinary_update_started;
    int ordinary_requested_jpeg_quality;
    int ordinary_applied_jpeg_quality;
    int waiting_for_key_frame;
    int framebuffer_lock_held;
    int ordinary_update_timer_active;
    int previous_left_button;
    int previous_wheel_buttons;
    int last_pointer_x;
    int last_pointer_y;
};

struct damage_rect {
    int x1;
    int y1;
    int x2;
    int y2;
};

struct scroll_row {
    uint8_t luma[SCROLL_ROW_SAMPLES];
    uint8_t informative;
};

static struct frame frame_queue[FRAME_QUEUE_CAPACITY];
static size_t frame_queue_head;
static size_t frame_queue_count;
static uint64_t frame_next_sequence = 1;
static pthread_mutex_t frame_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Compressed packets waiting for the fallback decoder thread. Decoding runs
 * off the video reader thread so a slow decode can never stall the H.264
 * passthrough path's socket reads. */
static struct frame decode_queue[DECODE_QUEUE_CAPACITY];
static size_t decode_queue_head;
static size_t decode_queue_count;
static int decode_skip_until_key;
static pthread_mutex_t decode_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t decode_cond = PTHREAD_COND_INITIALIZER;

static uint8_t *codec_config;
static size_t codec_config_size;

static AVCodecContext *decoder_context;
static AVFrame *decoder_frame;
static struct SwsContext *sws_context;
static uint8_t *fallback_buffer;
static size_t framebuffer_size;
static int fallback_frame_ready;
static int fallback_decoder_ready_logged;
static pthread_mutex_t fallback_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_rwlock_t screen_buffer_lock = PTHREAD_RWLOCK_INITIALIZER;
static pthread_mutex_t screen_ready_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t screen_buffer_cond;
static pthread_mutex_t metrics_mutex = PTHREAD_MUTEX_INITIALIZER;
static int screen_frame_ready;
static volatile sig_atomic_t fallback_mode;
static volatile sig_atomic_t fallback_decoder_reset;
static pthread_mutex_t pointer_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t control_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t reset_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint64_t last_reset_request_ns;

static int video_fd = -1;
static int control_fd = -1;
static int video_width;
static int video_height;
static volatile sig_atomic_t running = 1;
static volatile sig_atomic_t rfb_ready;
static rfbScreenInfoPtr rfb_screen;
static rfbClientPtr pointer_owner;

static void client_gone(rfbClientPtr client);
static int send_touch(uint8_t action, int x, int y, int pressed);

static uint64_t monotonic_ns(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t) now.tv_sec * UINT64_C(1000000000)
         + (uint64_t) now.tv_nsec;
}

static uint64_t elapsed_us(const struct timespec *start,
                           const struct timespec *end) {
    int64_t seconds = end->tv_sec - start->tv_sec;
    int64_t nanoseconds = end->tv_nsec - start->tv_nsec;
    int64_t total = seconds * INT64_C(1000000000) + nanoseconds;
    return total > 0 ? (uint64_t) total / 1000 : 0;
}

static uint32_t read_u32be(const uint8_t *p) {
    return ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16)
         | ((uint32_t) p[2] << 8) | p[3];
}

static uint64_t read_u64be(const uint8_t *p) {
    return ((uint64_t) read_u32be(p) << 32) | read_u32be(p + 4);
}

static void write_u16be(uint8_t *p, uint16_t value) {
    p[0] = value >> 8;
    p[1] = value;
}

static void write_u32be(uint8_t *p, uint32_t value) {
    p[0] = value >> 24;
    p[1] = value >> 16;
    p[2] = value >> 8;
    p[3] = value;
}

static void write_u64be(uint8_t *p, uint64_t value) {
    write_u32be(p, value >> 32);
    write_u32be(p + 4, value);
}

static int recv_all(int fd, void *buffer, size_t size) {
    uint8_t *p = buffer;
    while (size) {
        ssize_t n = recv(fd, p, size, 0);
        if (n == 0) {
            return 0;
        }
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        p += n;
        size -= (size_t) n;
    }
    return 1;
}

static int send_all(int fd, const void *buffer, size_t size) {
    const uint8_t *p = buffer;
    while (size) {
        ssize_t n = send(fd, p, size, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        p += n;
        size -= (size_t) n;
    }
    return 0;
}

static int connect_tcp(const char *host, uint16_t port) {
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
        fprintf(stderr, "invalid IPv4 address: %s\n", host);
        return -1;
    }

    for (int attempt = 0; attempt < 100 && running; ++attempt) {
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            return -1;
        }
        if (connect(fd, (struct sockaddr *) &addr, sizeof(addr)) == 0) {
            return fd;
        }
        close(fd);
        struct timespec delay = {.tv_sec = 0, .tv_nsec = 50 * 1000 * 1000};
        nanosleep(&delay, NULL);
    }
    return -1;
}

static void free_frame(struct frame *frame) {
    free(frame->data);
    memset(frame, 0, sizeof(*frame));
}

static void clear_frame_queue_locked(void) {
    while (frame_queue_count) {
        free_frame(&frame_queue[frame_queue_head]);
        frame_queue_head = (frame_queue_head + 1) % FRAME_QUEUE_CAPACITY;
        --frame_queue_count;
    }
}

/* Sequence of the newest keyframe in the queue, 0 if there is none. */
static uint64_t latest_key_sequence_locked(void) {
    uint64_t sequence = 0;
    for (size_t i = 0; i < frame_queue_count; ++i) {
        size_t index = (frame_queue_head + i) % FRAME_QUEUE_CAPACITY;
        if (frame_queue[index].key_frame) {
            sequence = frame_queue[index].sequence;
        }
    }
    return sequence;
}

static int reset_scrcpy_video(void) {
    const uint8_t message = SCRCPY_MSG_RESET_VIDEO;
    pthread_mutex_lock(&control_mutex);
    int result = send_all(control_fd, &message, sizeof(message));
    pthread_mutex_unlock(&control_mutex);
    return result;
}

/* Ask scrcpy for a fresh config packet and keyframe. Unless forced, at most
 * one request per second goes out no matter how many clients are waiting. */
static void request_video_reset(int force) {
    uint64_t now = monotonic_ns();
    int send_reset = 0;

    pthread_mutex_lock(&reset_mutex);
    if (force || !last_reset_request_ns
            || now - last_reset_request_ns >= RESET_REQUEST_INTERVAL_NS) {
        last_reset_request_ns = now;
        send_reset = 1;
    }
    pthread_mutex_unlock(&reset_mutex);

    if (send_reset && reset_scrcpy_video() < 0) {
        fprintf(stderr, "failed to request a video reset\n");
    }
}

static void enqueue_frame(uint8_t *data, size_t size, int key_frame) {
    pthread_mutex_lock(&frame_mutex);

    if (frame_queue_count == FRAME_QUEUE_CAPACITY) {
        free_frame(&frame_queue[frame_queue_head]);
        frame_queue_head = (frame_queue_head + 1) % FRAME_QUEUE_CAPACITY;
        --frame_queue_count;
    }

    size_t tail = (frame_queue_head + frame_queue_count) % FRAME_QUEUE_CAPACITY;
    frame_queue[tail].data = data;
    frame_queue[tail].size = size;
    frame_queue[tail].key_frame = key_frame;
    frame_queue[tail].sequence = frame_next_sequence++;
    ++frame_queue_count;
    pthread_mutex_unlock(&frame_mutex);

    if (rfb_ready) {
        rfbNotifyH264FrameAvailable(rfb_screen);
    }
}

/* Non-blocking: copy the next access unit this client should send, skipping
 * to the newest keyframe when the client has fallen behind. Returns 0 when
 * the client has to wait for a later enqueue; *needs_keyframe is set when
 * the stream cannot resume without a fresh keyframe, *more_pending when
 * another eligible frame is already queued behind the returned one. */
static int copy_next_frame(struct client_state *state, struct frame *frame,
                           int *needs_keyframe, int *more_pending) {
    struct frame *source = NULL;

    *needs_keyframe = 0;
    *more_pending = 0;

    pthread_mutex_lock(&frame_mutex);
    if (frame_queue_count) {
        uint64_t oldest_sequence = frame_queue[frame_queue_head].sequence;
        if (state->next_sequence < oldest_sequence) {
            state->next_sequence = oldest_sequence;
            state->waiting_for_key_frame = 1;
        }

        uint64_t latest_key = latest_key_sequence_locked();
        if (latest_key > state->next_sequence &&
            frame_next_sequence - state->next_sequence > 6) {
            state->next_sequence = latest_key;
            state->waiting_for_key_frame = 0;
        }

        for (size_t i = 0; i < frame_queue_count; ++i) {
            size_t index = (frame_queue_head + i) % FRAME_QUEUE_CAPACITY;
            struct frame *candidate = &frame_queue[index];
            if (candidate->sequence < state->next_sequence) {
                continue;
            }
            if (state->waiting_for_key_frame && !candidate->key_frame) {
                continue;
            }
            source = candidate;
            break;
        }
    }

    if (!source) {
        *needs_keyframe = state->waiting_for_key_frame;
        pthread_mutex_unlock(&frame_mutex);
        return 0;
    }

    frame->data = malloc(source->size);
    if (!frame->data) {
        pthread_mutex_unlock(&frame_mutex);
        return 0;
    }
    memcpy(frame->data, source->data, source->size);
    frame->size = source->size;
    frame->key_frame = source->key_frame;
    frame->sequence = source->sequence;
    state->next_sequence = source->sequence + 1;
    state->waiting_for_key_frame = 0;
    *more_pending = frame_next_sequence > state->next_sequence;
    pthread_mutex_unlock(&frame_mutex);
    return 1;
}

static int init_fallback_decoder(void) {
    const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) {
        fprintf(stderr, "H.264 fallback decoder not found\n");
        return -1;
    }

    decoder_context = avcodec_alloc_context3(codec);
    decoder_frame = av_frame_alloc();
    if (!decoder_context || !decoder_frame
            || avcodec_open2(decoder_context, codec, NULL) < 0) {
        fprintf(stderr, "failed to initialize H.264 fallback decoder\n");
        return -1;
    }
    return 0;
}

static void decode_fallback_frame(const uint8_t *data, size_t size,
                                  int key_frame, int *decoder_active) {
    if (!fallback_mode) {
        *decoder_active = 0;
        return;
    }

    if (fallback_decoder_reset) {
        *decoder_active = 0;
        fallback_decoder_reset = 0;
    }

    if (!*decoder_active) {
        if (!key_frame) {
            return;
        }
        avcodec_flush_buffers(decoder_context);
        *decoder_active = 1;
    }

    AVPacket *packet = av_packet_alloc();
    if (!packet || size > INT_MAX || av_new_packet(packet, (int) size) < 0) {
        av_packet_free(&packet);
        return;
    }
    memcpy(packet->data, data, size);

    int send_result = avcodec_send_packet(decoder_context, packet);
    if (send_result < 0) {
        char error[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(send_result, error, sizeof(error));
        fprintf(stderr, "fallback decoder rejected packet: %s\n", error);
        av_packet_free(&packet);
        return;
    }
    av_packet_free(&packet);

    while (avcodec_receive_frame(decoder_context, decoder_frame) >= 0) {
        if (decoder_frame->width != video_width
                || decoder_frame->height != video_height) {
            fprintf(stderr, "fallback decoder size mismatch: %dx%d\n",
                    decoder_frame->width, decoder_frame->height);
            av_frame_unref(decoder_frame);
            continue;
        }

        sws_context = sws_getCachedContext(
                sws_context,
                decoder_frame->width, decoder_frame->height,
                (enum AVPixelFormat) decoder_frame->format,
                video_width, video_height, AV_PIX_FMT_RGBA,
                SWS_FAST_BILINEAR, NULL, NULL, NULL);
        if (!sws_context) {
            av_frame_unref(decoder_frame);
            return;
        }

        uint8_t *destinations[] = {fallback_buffer, NULL, NULL, NULL};
        int strides[] = {video_width * 4, 0, 0, 0};
        pthread_mutex_lock(&fallback_mutex);
        sws_scale(sws_context,
                  (const uint8_t *const *) decoder_frame->data,
                  decoder_frame->linesize, 0, decoder_frame->height,
                  destinations, strides);
        fallback_frame_ready = 1;
        if (!fallback_decoder_ready_logged) {
            fprintf(stderr,
                    "ordinary VNC decoder ready: %dx%d format=%d\n",
                    decoder_frame->width, decoder_frame->height,
                    decoder_frame->format);
            fallback_decoder_ready_logged = 1;
        }
        pthread_mutex_unlock(&fallback_mutex);
        av_frame_unref(decoder_frame);
    }
}

static void clear_decode_queue_locked(void) {
    while (decode_queue_count) {
        free_frame(&decode_queue[decode_queue_head]);
        decode_queue_head = (decode_queue_head + 1) % DECODE_QUEUE_CAPACITY;
        --decode_queue_count;
    }
}

/* Hand a copy of the packet to the decoder thread. When decoding cannot keep
 * up, drop the backlog and resynchronize from the next keyframe instead of
 * stalling the video socket. */
static void decode_submit(const uint8_t *data, size_t size, int key_frame) {
    uint8_t *copy;
    size_t tail;

    if (!fallback_mode) {
        return;
    }

    pthread_mutex_lock(&decode_mutex);
    if (decode_skip_until_key && !key_frame) {
        pthread_mutex_unlock(&decode_mutex);
        return;
    }
    decode_skip_until_key = 0;

    if (decode_queue_count == DECODE_QUEUE_CAPACITY) {
        clear_decode_queue_locked();
        fallback_decoder_reset = 1;
        if (!key_frame) {
            /* Cannot resume from a delta frame: drop until the reset-
             * triggered keyframe arrives. */
            decode_skip_until_key = 1;
            pthread_mutex_unlock(&decode_mutex);
            request_video_reset(0);
            return;
        }
    }

    copy = malloc(size);
    if (copy) {
        memcpy(copy, data, size);
        tail = (decode_queue_head + decode_queue_count)
             % DECODE_QUEUE_CAPACITY;
        decode_queue[tail].data = copy;
        decode_queue[tail].size = size;
        decode_queue[tail].key_frame = key_frame;
        decode_queue[tail].sequence = 0;
        ++decode_queue_count;
        pthread_cond_signal(&decode_cond);
    }
    pthread_mutex_unlock(&decode_mutex);
}

static void *decoder_main(void *unused) {
    (void) unused;
    int decoder_active = 0;

    while (running) {
        struct frame packet;

        pthread_mutex_lock(&decode_mutex);
        while (running && !decode_queue_count) {
            pthread_cond_wait(&decode_cond, &decode_mutex);
        }
        if (!running) {
            pthread_mutex_unlock(&decode_mutex);
            break;
        }
        packet = decode_queue[decode_queue_head];
        memset(&decode_queue[decode_queue_head], 0, sizeof(struct frame));
        decode_queue_head = (decode_queue_head + 1) % DECODE_QUEUE_CAPACITY;
        --decode_queue_count;
        pthread_mutex_unlock(&decode_mutex);

        decode_fallback_frame(packet.data, packet.size, packet.key_frame,
                              &decoder_active);
        free(packet.data);
    }
    return NULL;
}

static void *video_reader_main(void *unused) {
    (void) unused;
    uint8_t header[12];

    while (running) {
        int status = recv_all(video_fd, header, sizeof(header));
        if (status <= 0) {
            break;
        }

        uint64_t pts_flags = read_u64be(header);
        if (pts_flags & SCRCPY_PACKET_FLAG_SESSION) {
            int width = (int) read_u32be(header + 4);
            int height = (int) read_u32be(header + 8);
            fprintf(stderr, "scrcpy session: %dx%d%s\n", width, height,
                    (width == video_width && height == video_height)
                        ? "" : " (resize requires restart)");
            continue;
        }

        uint32_t packet_size = read_u32be(header + 8);
        if (!packet_size || packet_size > MAX_PACKET_SIZE) {
            fprintf(stderr, "invalid scrcpy packet size: %u\n", packet_size);
            break;
        }

        uint8_t *packet = malloc(packet_size);
        if (!packet || recv_all(video_fd, packet, packet_size) <= 0) {
            free(packet);
            break;
        }

        if (pts_flags & SCRCPY_PACKET_FLAG_CONFIG) {
            free(codec_config);
            codec_config = packet;
            codec_config_size = packet_size;
            continue;
        }

        int key_frame = !!(pts_flags & SCRCPY_PACKET_FLAG_KEY_FRAME);
        if (key_frame && codec_config_size) {
            uint8_t *merged = malloc(codec_config_size + packet_size);
            if (!merged) {
                free(packet);
                break;
            }
            memcpy(merged, codec_config, codec_config_size);
            memcpy(merged + codec_config_size, packet, packet_size);
            free(packet);
            packet = merged;
            packet_size += (uint32_t) codec_config_size;
        }

        decode_submit(packet, packet_size, key_frame);
        enqueue_frame(packet, packet_size, key_frame);
    }

    fprintf(stderr, "scrcpy video stream ended\n");
    running = 0;
    return NULL;
}

static void *control_drain_main(void *unused) {
    (void) unused;
    uint8_t buffer[4096];
    while (running) {
        ssize_t n = recv(control_fd, buffer, sizeof(buffer), 0);
        if (n <= 0) {
            break;
        }
    }
    return NULL;
}

/* Non-blocking Open H.264 frame hook. A client stuck waiting for a keyframe
 * (e.g. one that connected while the screen was static and the queue held
 * only delta frames) triggers a rate-limited encoder reset so its stream can
 * start without waiting for on-screen motion. */
static rfbBool h264_frame_hook(rfbClientPtr client, char **buffer,
                               size_t *size) {
    struct frame frame = {0};
    int needs_keyframe = 0;
    int more_pending = 0;
    struct client_state *state = client->clientData;

    if (!state) {
        return FALSE;
    }
    if (!copy_next_frame(state, &frame, &needs_keyframe, &more_pending)) {
        if (needs_keyframe) {
            request_video_reset(0);
        }
        return FALSE;
    }
    if (more_pending) {
        rfbNotifyH264FrameAvailable(rfb_screen);
    }

    *buffer = (char *) frame.data;
    *size = frame.size;
    return TRUE;
}

static enum rfbNewClientAction new_client(rfbClientPtr client) {
    struct client_state *state = calloc(1, sizeof(*state));
    if (!state) {
        return RFB_CLIENT_REFUSE;
    }

    state->ordinary_requested_jpeg_quality = -1;
    state->ordinary_applied_jpeg_quality = -1;

    int send_buffer = FALLBACK_TCP_SEND_BUFFER;
    if (setsockopt(client->sock, SOL_SOCKET, SO_SNDBUF, &send_buffer,
                   sizeof(send_buffer)) < 0) {
        fprintf(stderr, "failed to cap VNC send buffer for %s: %s\n",
                client->host, strerror(errno));
    }
#ifdef TCP_NOTSENT_LOWAT
    int notsent_lowat = FALLBACK_TCP_NOTSENT_LOWAT;
    if (setsockopt(client->sock, IPPROTO_TCP, TCP_NOTSENT_LOWAT,
                   &notsent_lowat, sizeof(notsent_lowat)) < 0) {
        fprintf(stderr, "failed to set TCP_NOTSENT_LOWAT for %s: %s\n",
                client->host, strerror(errno));
    }
#endif

    pthread_mutex_lock(&frame_mutex);
    uint64_t latest_key = latest_key_sequence_locked();
    state->next_sequence = latest_key ? latest_key : frame_next_sequence;
    state->waiting_for_key_frame = latest_key == 0;
    pthread_mutex_unlock(&frame_mutex);

    client->clientData = state;
    client->clientGoneHook = client_gone;
    fprintf(stderr, "VNC client connected: %s\n", client->host);
    return RFB_CLIENT_ACCEPT;
}

static void client_gone(rfbClientPtr client) {
    struct client_state *state = client->clientData;
    pthread_mutex_lock(&pointer_mutex);
    if (pointer_owner == client) {
        if (state && state->previous_left_button) {
            send_touch(1, state->last_pointer_x, state->last_pointer_y, 0);
        }
        pointer_owner = NULL;
    }
    pthread_mutex_unlock(&pointer_mutex);
    free(client->clientData);
    client->clientData = NULL;
    fprintf(stderr, "VNC client disconnected\n");
}

static int send_touch(uint8_t action, int x, int y, int pressed) {
    uint8_t message[32] = {0};
    message[0] = SCRCPY_MSG_INJECT_TOUCH;
    message[1] = action;
    write_u64be(message + 2, SCRCPY_POINTER_ID_MOUSE);
    write_u32be(message + 10, (uint32_t) x);
    write_u32be(message + 14, (uint32_t) y);
    write_u16be(message + 18, (uint16_t) video_width);
    write_u16be(message + 20, (uint16_t) video_height);
    write_u16be(message + 22, pressed ? UINT16_MAX : 0);
    write_u32be(message + 24, pressed ? 1U : 0U);
    write_u32be(message + 28, pressed ? 1U : 0U);
    pthread_mutex_lock(&control_mutex);
    int result = send_all(control_fd, message, sizeof(message));
    pthread_mutex_unlock(&control_mutex);
    return result;
}

/* hscroll/vscroll are one wheel click each, encoded as scrcpy i16
 * fixed-point where 32767 is 1.0. Positive vscroll scrolls up. */
static int send_scroll(int x, int y, int hscroll, int vscroll) {
    uint8_t message[21] = {0};
    message[0] = SCRCPY_MSG_INJECT_SCROLL;
    write_u32be(message + 1, (uint32_t) x);
    write_u32be(message + 5, (uint32_t) y);
    write_u16be(message + 9, (uint16_t) video_width);
    write_u16be(message + 11, (uint16_t) video_height);
    write_u16be(message + 13, (uint16_t) (int16_t) (hscroll * 32767));
    write_u16be(message + 15, (uint16_t) (int16_t) (vscroll * 32767));
    write_u32be(message + 17, 0);
    pthread_mutex_lock(&control_mutex);
    int result = send_all(control_fd, message, sizeof(message));
    pthread_mutex_unlock(&control_mutex);
    return result;
}

static void pointer_event(int button_mask, int x, int y, rfbClientPtr client) {
    struct client_state *state = client->clientData;
    if (!state) {
        return;
    }

    /* RFB buttons 4-7 (mask bits 3-6) are wheel up/down/left/right. Each
     * click arrives as a press/release pair; inject on the press edge. Wheel
     * events do not participate in pointer-drag ownership. */
    int wheel_buttons = button_mask & 0x78;
    int wheel_pressed = wheel_buttons & ~state->previous_wheel_buttons;
    state->previous_wheel_buttons = wheel_buttons;
    if (wheel_pressed & (1 << 3)) {
        send_scroll(x, y, 0, 1);
    }
    if (wheel_pressed & (1 << 4)) {
        send_scroll(x, y, 0, -1);
    }
    if (wheel_pressed & (1 << 5)) {
        send_scroll(x, y, -1, 0);
    }
    if (wheel_pressed & (1 << 6)) {
        send_scroll(x, y, 1, 0);
    }

    pthread_mutex_lock(&pointer_mutex);
    int left_button = !!(button_mask & 1);
    uint8_t action;
    if (left_button && !state->previous_left_button) {
        if (pointer_owner && pointer_owner != client) {
            goto done;
        }
        pointer_owner = client;
        action = 0;
    } else if (!left_button && state->previous_left_button) {
        if (pointer_owner != client) {
            goto done;
        }
        action = 1;
    } else if (left_button) {
        if (pointer_owner != client) {
            goto done;
        }
        action = 2;
    } else {
        goto done;
    }

    if (send_touch(action, x, y, left_button) < 0) {
        fprintf(stderr, "failed to send touch event\n");
    }
    state->last_pointer_x = x;
    state->last_pointer_y = y;
    state->previous_left_button = left_button;
    if (!left_button) {
        pointer_owner = NULL;
    }

done:
    pthread_mutex_unlock(&pointer_mutex);
}

static int keysym_to_android_keycode(rfbKeySym key) {
    switch (key) {
        case XK_Home: return 3;
        case XK_Escape: return 4;
        case XK_Up: return 19;
        case XK_Down: return 20;
        case XK_Left: return 21;
        case XK_Right: return 22;
        case XK_Tab: return 61;
        case XK_space: return 62;
        case XK_Return: return 66;
        case XK_KP_Enter: return 66;
        case XK_BackSpace: return 67;
        case XK_Page_Up: return 92;
        case XK_Page_Down: return 93;
        case XK_Delete: return 112;
        default: return -1;
    }
}

static size_t utf8_encode(uint32_t code_point, uint8_t *out) {
    if (code_point < 0x80) {
        out[0] = (uint8_t) code_point;
        return 1;
    }
    if (code_point < 0x800) {
        out[0] = (uint8_t) (0xc0 | (code_point >> 6));
        out[1] = (uint8_t) (0x80 | (code_point & 0x3f));
        return 2;
    }
    if (code_point < 0x10000) {
        if (code_point >= 0xd800 && code_point <= 0xdfff) {
            return 0;
        }
        out[0] = (uint8_t) (0xe0 | (code_point >> 12));
        out[1] = (uint8_t) (0x80 | ((code_point >> 6) & 0x3f));
        out[2] = (uint8_t) (0x80 | (code_point & 0x3f));
        return 3;
    }
    if (code_point <= 0x10ffff) {
        out[0] = (uint8_t) (0xf0 | (code_point >> 18));
        out[1] = (uint8_t) (0x80 | ((code_point >> 12) & 0x3f));
        out[2] = (uint8_t) (0x80 | ((code_point >> 6) & 0x3f));
        out[3] = (uint8_t) (0x80 | (code_point & 0x3f));
        return 4;
    }
    return 0;
}

/* Printable keysyms become scrcpy UTF-8 text injection: ASCII and Latin-1
 * map directly, X11 Unicode keysyms carry the code point in the low bits. */
static void keyboard_event(rfbBool down, rfbKeySym key, rfbClientPtr client) {
    (void) client;
    int keycode = keysym_to_android_keycode(key);
    if (keycode >= 0) {
        uint8_t message[14] = {0};
        message[0] = SCRCPY_MSG_INJECT_KEYCODE;
        message[1] = down ? 0 : 1;
        write_u32be(message + 2, (uint32_t) keycode);
        pthread_mutex_lock(&control_mutex);
        send_all(control_fd, message, sizeof(message));
        pthread_mutex_unlock(&control_mutex);
        return;
    }

    if (!down) {
        return;
    }

    uint32_t code_point = 0;
    if (key >= 0x20 && key <= 0x7e) {
        code_point = key;
    } else if (key >= 0xa0 && key <= 0xff) {
        code_point = key;
    } else if ((key & 0xff000000U) == 0x01000000U) {
        code_point = key & 0x00ffffffU;
    } else {
        return;
    }

    uint8_t message[9];
    size_t length = utf8_encode(code_point, message + 5);
    if (!length) {
        return;
    }
    message[0] = SCRCPY_MSG_INJECT_TEXT;
    write_u32be(message + 1, (uint32_t) length);
    pthread_mutex_lock(&control_mutex);
    send_all(control_fd, message, 5 + length);
    pthread_mutex_unlock(&control_mutex);
}

static int is_h264_encoding(int encoding) {
    return encoding == rfbEncodingOpenH264;
}

static void scroll_content_bounds(int height, int *top, int *bottom) {
    int margin = height / 20;
    if (margin < 32) {
        margin = 32;
    }
    *top = margin;
    *bottom = height - margin;
}

static void describe_scroll_rows(const uint8_t *frame, int width, int height,
                                 struct scroll_row *rows) {
    int top;
    int bottom;
    scroll_content_bounds(height, &top, &bottom);
    int x1 = width / 10;
    int span = width - 2 * x1;

    memset(rows, 0, (size_t) height * sizeof(*rows));
    for (int y = top; y < bottom; ++y) {
        unsigned minimum = 255;
        unsigned maximum = 0;
        unsigned transitions = 0;
        unsigned previous = 0;
        for (int sample = 0; sample < SCROLL_ROW_SAMPLES; ++sample) {
            int x = x1 + (2 * sample + 1) * span
                       / (2 * SCROLL_ROW_SAMPLES);
            const uint8_t *pixel = frame
                    + ((size_t) y * (size_t) width + (size_t) x) * 4;
            unsigned luma = (77U * pixel[0] + 150U * pixel[1]
                             + 29U * pixel[2]) >> 8;
            rows[y].luma[sample] = (uint8_t) luma;
            if (luma < minimum) {
                minimum = luma;
            }
            if (luma > maximum) {
                maximum = luma;
            }
            if (sample) {
                transitions += luma > previous ? luma - previous
                                               : previous - luma;
            }
            previous = luma;
        }
        rows[y].informative = maximum - minimum >= 18 && transitions >= 48;
    }
}

static int scroll_rows_match(const struct scroll_row *old_row,
                             const struct scroll_row *new_row) {
    unsigned difference = 0;
    unsigned close_samples = 0;
    for (int sample = 0; sample < SCROLL_ROW_SAMPLES; ++sample) {
        int delta = (int) old_row->luma[sample]
                  - (int) new_row->luma[sample];
        unsigned absolute = (unsigned) (delta < 0 ? -delta : delta);
        difference += absolute;
        if (absolute <= 12) {
            ++close_samples;
        }
    }
    return difference <= SCROLL_ROW_SAMPLES * 7
        && close_samples * 4 >= SCROLL_ROW_SAMPLES * 3;
}

/* Detect a high-confidence vertical translation. Exact pixel equality is too
 * brittle after H.264 reconstruction, so compare sparse luma descriptors and
 * ignore flat rows. A later tile pass always repairs fixed headers, exposed
 * rows, and any false-positive area before it is published as pixel damage. */
static int detect_vertical_scroll(const uint8_t *old_frame,
                                  const uint8_t *new_frame,
                                  int width, int height) {
    int top;
    int bottom;
    scroll_content_bounds(height, &top, &bottom);
    int maximum_shift = (bottom - top) / 3;
    if (maximum_shift > SCROLL_MAX_SHIFT) {
        maximum_shift = SCROLL_MAX_SHIFT;
    }
    if (maximum_shift < SCROLL_MIN_SHIFT) {
        return 0;
    }

    struct scroll_row *old_rows = calloc((size_t) height, sizeof(*old_rows));
    struct scroll_row *new_rows = calloc((size_t) height, sizeof(*new_rows));
    int *scores = calloc((size_t) (2 * maximum_shift + 1), sizeof(*scores));
    int *matches = calloc((size_t) (2 * maximum_shift + 1), sizeof(*matches));
    if (!old_rows || !new_rows || !scores || !matches) {
        free(old_rows);
        free(new_rows);
        free(scores);
        free(matches);
        return 0;
    }

    describe_scroll_rows(old_frame, width, height, old_rows);
    describe_scroll_rows(new_frame, width, height, new_rows);
    for (int dy = -maximum_shift; dy <= maximum_shift; ++dy) {
        if (dy > -SCROLL_MIN_SHIFT && dy < SCROLL_MIN_SHIFT) {
            continue;
        }
        int valid_rows = 0;
        int matching_rows = 0;
        for (int y = top; y < bottom; y += SCROLL_SAMPLE_ROW_STEP) {
            int source_y = y - dy;
            if (source_y < top || source_y >= bottom
                    || !new_rows[y].informative) {
                continue;
            }
            ++valid_rows;
            if (old_rows[source_y].informative
                    && scroll_rows_match(&old_rows[source_y], &new_rows[y])) {
                ++matching_rows;
            }
        }
        int index = dy + maximum_shift;
        matches[index] = matching_rows;
        scores[index] = valid_rows ? matching_rows * 1000 / valid_rows : 0;
    }

    int best_dy = 0;
    int best_score = 0;
    int best_matches = 0;
    for (int dy = -maximum_shift; dy <= maximum_shift; ++dy) {
        int index = dy + maximum_shift;
        if (scores[index] > best_score
                || (scores[index] == best_score
                    && matches[index] > best_matches)) {
            best_dy = dy;
            best_score = scores[index];
            best_matches = matches[index];
        }
    }

    int second_score = 0;
    for (int dy = -maximum_shift; dy <= maximum_shift; ++dy) {
        if (dy >= best_dy - 2 && dy <= best_dy + 2) {
            continue;
        }
        int score = scores[dy + maximum_shift];
        if (score > second_score) {
            second_score = score;
        }
    }

    free(old_rows);
    free(new_rows);
    free(scores);
    free(matches);
    if (best_matches < 10 || best_score < 450
            || (best_score < 800 && best_score - second_score < 100)) {
        return 0;
    }
    return best_dy;
}

static int run_self_test(void) {
    const int width = 320;
    const int height = 480;
    const int expected_dy = -24;
    size_t size = (size_t) width * (size_t) height * 4;
    uint8_t *old_frame = malloc(size);
    uint8_t *new_frame = malloc(size);
    if (!old_frame || !new_frame) {
        free(old_frame);
        free(new_frame);
        return 1;
    }

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            uint8_t *pixel = old_frame
                    + ((size_t) y * (size_t) width + (size_t) x) * 4;
            pixel[0] = (uint8_t) ((x * 11 + y * 3 + (x * y) / 17) & 0xff);
            pixel[1] = (uint8_t) ((x * 5 + y * 13 + (x * y) / 29) & 0xff);
            pixel[2] = (uint8_t) ((x * 7 + y * 17 + (x * y) / 37) & 0xff);
            pixel[3] = 255;
        }
    }
    memcpy(new_frame, old_frame, size);
    if (detect_vertical_scroll(old_frame, new_frame, width, height) != 0) {
        fprintf(stderr, "scroll self-test: static frame was misdetected\n");
        free(old_frame);
        free(new_frame);
        return 1;
    }

    int top;
    int bottom;
    scroll_content_bounds(height, &top, &bottom);
    for (int y = top; y < bottom + expected_dy; ++y) {
        memcpy(new_frame + (size_t) y * (size_t) width * 4,
               old_frame + (size_t) (y - expected_dy) * (size_t) width * 4,
               (size_t) width * 4);
    }
    int detected_dy = detect_vertical_scroll(old_frame, new_frame,
                                              width, height);
    free(old_frame);
    free(new_frame);
    if (detected_dy != expected_dy) {
        fprintf(stderr, "scroll self-test: expected dy=%d, got %d\n",
                expected_dy, detected_dy);
        return 1;
    }

    uint8_t utf8[4];
    if (utf8_encode(0x41, utf8) != 1 || utf8[0] != 0x41
            || utf8_encode(0xe9, utf8) != 2
            || utf8_encode(0x4e2d, utf8) != 3
            || utf8_encode(0x1f600, utf8) != 4
            || utf8_encode(0xd800, utf8) != 0) {
        fprintf(stderr, "utf8 self-test failed\n");
        return 1;
    }

    fprintf(stderr, "scrcpy-rfb self-test passed\n");
    return 0;
}

static void adapt_client_jpeg(rfbClientPtr client,
                              struct client_state *state) {
    if (client->preferredEncoding != rfbEncodingTight
            || client->supportsH264Encoding) {
        return;
    }

    int previous_quality = client->turboQualityLevel;
    int target_quality = -1;
    uint64_t update_us;
    pthread_mutex_lock(&metrics_mutex);
    if (client->turboQualityLevel
            != state->ordinary_applied_jpeg_quality) {
        state->ordinary_requested_jpeg_quality = client->turboQualityLevel;
        state->ordinary_applied_jpeg_quality = client->turboQualityLevel;
    }
    update_us = state->ordinary_update_us_ema;
    if (state->ordinary_requested_jpeg_quality >= 0) {
        target_quality = update_us > FALLBACK_MEDIUM_UPDATE_US ? 80
                       : update_us > FALLBACK_FAST_UPDATE_US ? 86 : 92;
        if (target_quality > state->ordinary_requested_jpeg_quality) {
            target_quality = state->ordinary_requested_jpeg_quality;
        }
        state->ordinary_applied_jpeg_quality = target_quality;
    }
    pthread_mutex_unlock(&metrics_mutex);

    if (target_quality >= 0) {
        client->turboQualityLevel = target_quality;
        client->tightCompressLevel = 1;
        if (target_quality != previous_quality) {
            fprintf(stderr,
                    "ordinary Tight JPEG for %s: Q%d (update %.1f ms)\n",
                    client->host, target_quality, (double) update_us / 1000.0);
        }
    }
}

/* Publish only pixels that differ from the last stable framebuffer. The
 * decoder thread intentionally owns a single pending fallback buffer: when
 * VNC output is slower than Android, swscale replaces that pending image and
 * this function publishes the newest image instead of replaying stale decoded
 * frames. */
static size_t publish_latest_fallback_frame(struct damage_rect *rects,
                                            size_t rect_capacity,
                                            int enable_copyrect,
                                            int *copyrect_dy,
                                            int *full_screen,
                                            int *consumed) {
    size_t rect_count = 0;
    *copyrect_dy = 0;
    *full_screen = 0;
    *consumed = 0;

    pthread_mutex_lock(&fallback_mutex);
    int frame_pending = fallback_frame_ready;
    pthread_mutex_unlock(&fallback_mutex);
    if (!frame_pending) {
        return 0;
    }

    /* Do not hold fallback_mutex while waiting for slow VNC readers. The
     * decoder thread can keep replacing the pending image, and H.264
     * passthrough stays independent of ordinary-client backpressure. */
    pthread_rwlock_wrlock(&screen_buffer_lock);
    pthread_mutex_lock(&fallback_mutex);
    if (!fallback_frame_ready) {
        pthread_mutex_unlock(&fallback_mutex);
        pthread_rwlock_unlock(&screen_buffer_lock);
        return 0;
    }
    *consumed = 1;
    pthread_mutex_lock(&screen_ready_mutex);
    int first_frame = !screen_frame_ready;
    pthread_mutex_unlock(&screen_ready_mutex);

    if (first_frame) {
        memcpy(rfb_screen->frameBuffer, fallback_buffer, framebuffer_size);
        *full_screen = 1;
    } else {
        if (enable_copyrect) {
            int dy = detect_vertical_scroll(
                    (const uint8_t *) rfb_screen->frameBuffer,
                    fallback_buffer, video_width, video_height);
            if (dy) {
                int top;
                int bottom;
                scroll_content_bounds(video_height, &top, &bottom);
                int destination_top = dy > 0 ? top + dy : top;
                int destination_bottom = dy < 0 ? bottom + dy : bottom;
                if (destination_bottom > destination_top) {
                    rfbDoCopyRect(rfb_screen, 0, destination_top,
                                  video_width, destination_bottom, 0, dy);
                    *copyrect_dy = dy;
                }
            }
        }
        const int row_stride = video_width * 4;
        for (int y1 = 0; y1 < video_height; y1 += FALLBACK_TILE_SIZE) {
            int y2 = y1 + FALLBACK_TILE_SIZE;
            if (y2 > video_height) {
                y2 = video_height;
            }
            int run_x1 = -1;

            for (int x1 = 0; x1 < video_width; x1 += FALLBACK_TILE_SIZE) {
                int x2 = x1 + FALLBACK_TILE_SIZE;
                if (x2 > video_width) {
                    x2 = video_width;
                }
                size_t row_bytes = (size_t) (x2 - x1) * 4;
                int changed = 0;

                for (int y = y1; y < y2; ++y) {
                    size_t offset = (size_t) y * (size_t) row_stride
                                  + (size_t) x1 * 4;
                    if (memcmp(rfb_screen->frameBuffer + offset,
                               fallback_buffer + offset, row_bytes) != 0) {
                        changed = 1;
                        break;
                    }
                }

                if (changed) {
                    for (int y = y1; y < y2; ++y) {
                        size_t offset = (size_t) y * (size_t) row_stride
                                      + (size_t) x1 * 4;
                        memcpy(rfb_screen->frameBuffer + offset,
                               fallback_buffer + offset, row_bytes);
                    }
                    if (run_x1 < 0) {
                        run_x1 = x1;
                    }
                }

                if ((!changed || x2 == video_width) && run_x1 >= 0) {
                    int run_x2 = changed ? x2 : x1;
                    int merged = 0;
                    for (size_t i = rect_count; i > 0; --i) {
                        struct damage_rect *previous = &rects[i - 1];
                        if (previous->x1 == run_x1
                                && previous->x2 == run_x2
                                && previous->y2 == y1) {
                            previous->y2 = y2;
                            merged = 1;
                            break;
                        }
                    }
                    /* Reuse the previous tile row's identical horizontal run,
                     * turning full-motion frames into a few tall rectangles. */
                    if (!merged) {
                        if (rect_count < rect_capacity) {
                            rects[rect_count++] = (struct damage_rect) {
                                .x1 = run_x1,
                                .y1 = y1,
                                .x2 = run_x2,
                                .y2 = y2,
                            };
                        } else {
                            *full_screen = 1;
                        }
                    }
                    run_x1 = -1;
                }
            }
        }
    }

    fallback_frame_ready = 0;
    pthread_mutex_lock(&screen_ready_mutex);
    screen_frame_ready = 1;
    pthread_cond_broadcast(&screen_buffer_cond);
    pthread_mutex_unlock(&screen_ready_mutex);
    pthread_rwlock_unlock(&screen_buffer_lock);
    pthread_mutex_unlock(&fallback_mutex);
    return rect_count;
}

/* Ordinary encoders read screen->frameBuffer while their independent output
 * threads are sending. Hold a shared publication lock for the whole update so
 * a newly decoded frame cannot be copied over a client mid-rectangle. The
 * first ordinary update also waits for the reset-triggered keyframe instead
 * of exposing the calloc()ed black framebuffer. H.264 clients bypass this
 * lock because their frame hook reads the packet queue, not the framebuffer. */
static void display_hook(rfbClientPtr client) {
    struct client_state *state = client->clientData;
    if (!state || is_h264_encoding(client->preferredEncoding)) {
        return;
    }

    /* Ordinary clients should work with their defaults. Do not advertise a
     * resizeable desktop: this bridge has a fixed-size scrcpy session, and
     * several viewers terminate when their automatic SetDesktopSize request
     * is rejected. H.264 clients retain the extension and explicitly opt out
     * of remote resize because their integration is already specialized. */
    client->useExtDesktopSize = FALSE;
    client->useNewFBSize = FALSE;
    client->newFBSizePending = FALSE;
    if (client->supportsH264Encoding) {
        client->tightQualityLevel = -1;
        client->turboQualityLevel = -1;
    } else {
        adapt_client_jpeg(client, state);
    }

    pthread_mutex_lock(&screen_ready_mutex);
    while (running && !screen_frame_ready
            && client->sock != RFB_INVALID_SOCKET) {
        struct timespec deadline;
        clock_gettime(CLOCK_MONOTONIC, &deadline);
        deadline.tv_nsec += 100 * 1000 * 1000;
        if (deadline.tv_nsec >= 1000 * 1000 * 1000) {
            ++deadline.tv_sec;
            deadline.tv_nsec -= 1000 * 1000 * 1000;
        }
        pthread_cond_timedwait(&screen_buffer_cond, &screen_ready_mutex,
                               &deadline);
    }
    if (!running || client->sock == RFB_INVALID_SOCKET) {
        pthread_mutex_unlock(&screen_ready_mutex);
        return;
    }
    pthread_mutex_unlock(&screen_ready_mutex);

    pthread_rwlock_rdlock(&screen_buffer_lock);
    pthread_mutex_lock(&metrics_mutex);
    clock_gettime(CLOCK_MONOTONIC, &state->ordinary_update_started);
    state->ordinary_update_timer_active = 1;
    pthread_mutex_unlock(&metrics_mutex);
    state->framebuffer_lock_held = 1;
}

static void display_finished_hook(rfbClientPtr client, int result) {
    (void) result;
    struct client_state *state = client->clientData;
    if (state && state->framebuffer_lock_held) {
        struct timespec finished;
        clock_gettime(CLOCK_MONOTONIC, &finished);
        pthread_mutex_lock(&metrics_mutex);
        if (state->ordinary_update_timer_active) {
            uint64_t sample = elapsed_us(&state->ordinary_update_started,
                                         &finished);
            if (!state->ordinary_update_us_ema) {
                state->ordinary_update_us_ema = sample;
            } else {
                state->ordinary_update_us_ema =
                        (state->ordinary_update_us_ema * 3 + sample) / 4;
            }
            state->ordinary_update_timer_active = 0;
        }
        pthread_mutex_unlock(&metrics_mutex);
        state->framebuffer_lock_held = 0;
        pthread_rwlock_unlock(&screen_buffer_lock);
    }
}

static void count_client_modes(int *h264_clients, int *standard_clients,
                               int *copyrect_clients,
                               uint64_t *slowest_update_us) {
    *h264_clients = 0;
    *standard_clients = 0;
    *copyrect_clients = 0;
    *slowest_update_us = 0;

    rfbClientIteratorPtr iterator = rfbGetClientIterator(rfb_screen);
    rfbClientPtr client;
    while ((client = rfbClientIteratorNext(iterator)) != NULL) {
        if (client->state != RFB_NORMAL || client->preferredEncoding == -1) {
            continue;
        }
        if (is_h264_encoding(client->preferredEncoding)) {
            ++*h264_clients;
        } else {
            ++*standard_clients;
            if (client->useCopyRect) {
                ++*copyrect_clients;
            }
            struct client_state *state = client->clientData;
            if (state) {
                pthread_mutex_lock(&metrics_mutex);
                uint64_t update_us = state->ordinary_update_us_ema;
                pthread_mutex_unlock(&metrics_mutex);
                if (update_us > *slowest_update_us) {
                    *slowest_update_us = update_us;
                }
            }
        }
    }
    rfbReleaseClientIterator(iterator);
}

static void handle_signal(int signal_number) {
    (void) signal_number;
    running = 0;
}

static void usage(const char *program) {
    fprintf(stderr,
            "usage: %s [options] [libvncserver options]\n"
            "\n"
            "  --scrcpy-host addr   scrcpy server IPv4 address (default 127.0.0.1)\n"
            "  --scrcpy-port port   scrcpy server TCP port (default 27183)\n"
            "  --self-test          run built-in self tests and exit\n"
            "  --help               show this help\n"
            "\n"
            "libvncserver options (-listen, -rfbport, -passwd, -rfbauth, ...):\n",
            program);
    rfbUsage();
}

int main(int argc, char **argv) {
    const char *scrcpy_host = "127.0.0.1";
    uint16_t scrcpy_port = 27183;

    for (int i = 1; i < argc;) {
        if (strcmp(argv[i], "--self-test") == 0) {
            return run_self_test();
        }
        if (strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        }
        if (strcmp(argv[i], "--scrcpy-host") == 0 && i + 1 < argc) {
            scrcpy_host = argv[i + 1];
            memmove(argv + i, argv + i + 2,
                    (size_t) (argc - i - 2) * sizeof(*argv));
            argc -= 2;
            continue;
        }
        if (strcmp(argv[i], "--scrcpy-port") == 0 && i + 1 < argc) {
            long port = strtol(argv[i + 1], NULL, 10);
            if (port <= 0 || port > 65535) {
                fprintf(stderr, "invalid scrcpy port: %s\n", argv[i + 1]);
                return 1;
            }
            scrcpy_port = (uint16_t) port;
            memmove(argv + i, argv + i + 2,
                    (size_t) (argc - i - 2) * sizeof(*argv));
            argc -= 2;
            continue;
        }
        ++i;
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    signal(SIGPIPE, SIG_IGN);

    pthread_condattr_t monotonic_attr;
    pthread_condattr_init(&monotonic_attr);
    pthread_condattr_setclock(&monotonic_attr, CLOCK_MONOTONIC);
    pthread_cond_init(&screen_buffer_cond, &monotonic_attr);
    pthread_condattr_destroy(&monotonic_attr);

    video_fd = connect_tcp(scrcpy_host, scrcpy_port);
    if (video_fd < 0) {
        fprintf(stderr, "failed to connect scrcpy video socket\n");
        return 1;
    }
    control_fd = connect_tcp(scrcpy_host, scrcpy_port);
    if (control_fd < 0) {
        fprintf(stderr, "failed to connect scrcpy control socket\n");
        return 1;
    }

    uint8_t codec_header[4];
    uint8_t session_header[12];
    if (recv_all(video_fd, codec_header, sizeof(codec_header)) <= 0
            || read_u32be(codec_header) != SCRCPY_CODEC_H264) {
        fprintf(stderr, "scrcpy did not negotiate H.264\n");
        return 1;
    }
    if (recv_all(video_fd, session_header, sizeof(session_header)) <= 0
            || !(read_u64be(session_header) & SCRCPY_PACKET_FLAG_SESSION)) {
        fprintf(stderr, "missing initial scrcpy session metadata\n");
        return 1;
    }
    video_width = (int) read_u32be(session_header + 4);
    video_height = (int) read_u32be(session_header + 8);
    fprintf(stderr, "scrcpy H.264 stream: %dx%d\n", video_width, video_height);

    framebuffer_size = (size_t) video_width * (size_t) video_height * 4;
    fallback_buffer = calloc(1, framebuffer_size);
    if (!fallback_buffer || init_fallback_decoder() < 0) {
        free(fallback_buffer);
        return 1;
    }

    pthread_t video_thread;
    pthread_t control_thread;
    pthread_t decoder_thread;
    pthread_create(&video_thread, NULL, video_reader_main, NULL);
    pthread_create(&control_thread, NULL, control_drain_main, NULL);
    pthread_create(&decoder_thread, NULL, decoder_main, NULL);

    rfb_screen = rfbGetScreen(&argc, argv, video_width, video_height, 8, 3, 4);
    if (!rfb_screen) {
        return 1;
    }
    for (int i = 1; i < argc; ++i) {
        fprintf(stderr, "ignoring unknown argument: %s\n", argv[i]);
    }
    rfb_screen->desktopName = "scrcpy H.264 passthrough";
    rfb_screen->frameBuffer = calloc(1, framebuffer_size);
    if (!rfb_screen->frameBuffer) {
        return 1;
    }
    size_t damage_rect_capacity =
            ((size_t) video_width + FALLBACK_TILE_SIZE - 1) / FALLBACK_TILE_SIZE
          * (((size_t) video_height + FALLBACK_TILE_SIZE - 1)
             / FALLBACK_TILE_SIZE);
    struct damage_rect *damage_rects =
            calloc(damage_rect_capacity, sizeof(*damage_rects));
    if (!damage_rects) {
        return 1;
    }
    rfb_screen->alwaysShared = TRUE;
    rfb_screen->neverShared = FALSE;
    rfb_screen->newClientHook = new_client;
    rfb_screen->ptrAddEvent = pointer_event;
    rfb_screen->kbdAddEvent = keyboard_event;
    rfb_screen->getH264FrameHook = h264_frame_hook;
    rfb_screen->displayHook = display_hook;
    rfb_screen->displayFinishedHook = display_finished_hook;
    rfb_screen->deferUpdateTime = 0;
    rfb_screen->ipv6port = -1;
    rfbInitServer(rfb_screen);
    /* Explicit listener select timeout: with deferUpdateTime 0 the default
     * would otherwise derive a zero-timeout busy poll on unpatched trees. */
    rfbRunEventLoop(rfb_screen, 100 * 1000, TRUE);
    rfb_ready = 1;

    fprintf(stderr,
            "RFB listening on port %d (H.264 passthrough + Tight/JPEG fallback)\n",
            rfb_screen->port);
    uint64_t last_fallback_publish_ns = 0;
    uint64_t last_adapt_ns = monotonic_ns();
    int ordinary_fps = 60;
    int faster_intervals = 0;
    int previous_h264_clients = -1;
    int previous_standard_clients = -1;
    int previous_copyrect_clients = -1;
    uint64_t last_copyrect_log_ns = 0;
    unsigned copyrect_frames_since_log = 0;
    int last_copyrect_dy = 0;
    while (running && rfbIsActive(rfb_screen)) {
        int h264_clients;
        int standard_clients;
        int copyrect_clients;
        uint64_t slowest_update_us;
        count_client_modes(&h264_clients, &standard_clients, &copyrect_clients,
                           &slowest_update_us);
        if (h264_clients != previous_h264_clients
                || standard_clients != previous_standard_clients
                || copyrect_clients != previous_copyrect_clients) {
            fprintf(stderr,
                    "active VNC clients: H.264=%d ordinary=%d CopyRect=%d\n",
                    h264_clients, standard_clients, copyrect_clients);
            previous_h264_clients = h264_clients;
            previous_standard_clients = standard_clients;
            previous_copyrect_clients = copyrect_clients;
        }

        int requested_fallback = standard_clients > 0;
        if (requested_fallback != fallback_mode) {
            fallback_mode = requested_fallback;
            fprintf(stderr, "ordinary VNC fallback %s\n",
                    requested_fallback ? "enabled" : "disabled");
            if (requested_fallback) {
                pthread_mutex_lock(&screen_ready_mutex);
                screen_frame_ready = 0;
                pthread_mutex_unlock(&screen_ready_mutex);
                pthread_mutex_lock(&fallback_mutex);
                fallback_frame_ready = 0;
                pthread_mutex_unlock(&fallback_mutex);
                fallback_decoder_reset = 1;
                last_fallback_publish_ns = 0;
                ordinary_fps = 60;
                faster_intervals = 0;
                request_video_reset(1);
            }
        }

        uint64_t now_ns = monotonic_ns();
        if (standard_clients > 0
                && now_ns - last_adapt_ns >= FALLBACK_ADAPT_INTERVAL_NS) {
            int desired_fps = slowest_update_us > FALLBACK_MEDIUM_UPDATE_US ? 20
                            : slowest_update_us > FALLBACK_FAST_UPDATE_US ? 30
                            : 60;
            int previous_fps = ordinary_fps;
            if (desired_fps < ordinary_fps) {
                ordinary_fps = desired_fps;
                faster_intervals = 0;
            } else if (desired_fps > ordinary_fps) {
                if (++faster_intervals >= 3) {
                    ordinary_fps = desired_fps;
                    faster_intervals = 0;
                }
            } else {
                faster_intervals = 0;
            }
            if (ordinary_fps != previous_fps) {
                fprintf(stderr,
                        "ordinary VNC publish rate: %d fps (slowest update %.1f ms)\n",
                        ordinary_fps, (double) slowest_update_us / 1000.0);
            }
            last_adapt_ns = now_ns;
        }

        uint64_t publish_interval_ns = UINT64_C(1000000000)
                                     / (uint64_t) ordinary_fps;
        pthread_mutex_lock(&screen_ready_mutex);
        int first_frame_needed = !screen_frame_ready;
        pthread_mutex_unlock(&screen_ready_mutex);
        if (standard_clients > 0
                && (first_frame_needed || !last_fallback_publish_ns
                    || now_ns - last_fallback_publish_ns
                       >= publish_interval_ns)) {
            int full_screen;
            int consumed;
            int copyrect_dy;
            size_t rect_count = publish_latest_fallback_frame(
                    damage_rects, damage_rect_capacity, copyrect_clients > 0,
                    &copyrect_dy, &full_screen, &consumed);
            if (consumed) {
                last_fallback_publish_ns = monotonic_ns();
                if (copyrect_dy) {
                    ++copyrect_frames_since_log;
                    last_copyrect_dy = copyrect_dy;
                    if (!last_copyrect_log_ns
                            || last_fallback_publish_ns - last_copyrect_log_ns
                               >= UINT64_C(2000000000)) {
                        fprintf(stderr,
                                "ordinary VNC CopyRect: %u scroll frames, last dy=%d\n",
                                copyrect_frames_since_log, last_copyrect_dy);
                        copyrect_frames_since_log = 0;
                        last_copyrect_log_ns = last_fallback_publish_ns;
                    }
                }
                if (full_screen) {
                    rfbMarkRectAsModified(rfb_screen, 0, 0,
                                          video_width, video_height);
                } else {
                    for (size_t i = 0; i < rect_count; ++i) {
                        rfbMarkRectAsModified(rfb_screen,
                                              damage_rects[i].x1,
                                              damage_rects[i].y1,
                                              damage_rects[i].x2,
                                              damage_rects[i].y2);
                    }
                }
            }
        }

        struct timespec delay = {.tv_sec = 0, .tv_nsec = 2 * 1000 * 1000};
        nanosleep(&delay, NULL);
    }

    running = 0;
    rfbShutdownServer(rfb_screen, TRUE);
    shutdown(video_fd, SHUT_RDWR);
    shutdown(control_fd, SHUT_RDWR);
    pthread_mutex_lock(&decode_mutex);
    pthread_cond_broadcast(&decode_cond);
    pthread_mutex_unlock(&decode_mutex);
    pthread_mutex_lock(&screen_ready_mutex);
    pthread_cond_broadcast(&screen_buffer_cond);
    pthread_mutex_unlock(&screen_ready_mutex);
    pthread_join(video_thread, NULL);
    pthread_join(control_thread, NULL);
    pthread_join(decoder_thread, NULL);

    close(video_fd);
    close(control_fd);
    free(codec_config);
    sws_freeContext(sws_context);
    av_frame_free(&decoder_frame);
    avcodec_free_context(&decoder_context);
    free(fallback_buffer);
    free(damage_rects);
    pthread_mutex_lock(&frame_mutex);
    clear_frame_queue_locked();
    pthread_mutex_unlock(&frame_mutex);
    pthread_mutex_lock(&decode_mutex);
    clear_decode_queue_locked();
    pthread_mutex_unlock(&decode_mutex);
    free(rfb_screen->frameBuffer);
    rfb_screen->frameBuffer = NULL;
    rfbScreenCleanup(rfb_screen);
    return 0;
}
