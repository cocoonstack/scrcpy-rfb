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
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/intreadwrite.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
#include <rfb/keysym.h>
#include <rfb/rfb.h>
#include <zlib.h>

#define SCRCPY_DEFAULT_HOST "127.0.0.1"
#define SCRCPY_DEFAULT_PORT 27183
#define SCRCPY_CODEC_H264 UINT32_C(0x68323634)
#define SCRCPY_PACKET_FLAG_SESSION (UINT64_C(1) << 63)
#define SCRCPY_PACKET_FLAG_CONFIG (UINT64_C(1) << 62)
#define SCRCPY_PACKET_FLAG_KEY_FRAME (UINT64_C(1) << 61)
#define SCRCPY_POINTER_ID_MOUSE UINT64_MAX
#define SCRCPY_MSG_INJECT_KEYCODE 0
#define SCRCPY_MSG_INJECT_TEXT 1
#define SCRCPY_MSG_INJECT_TOUCH 2
#define SCRCPY_MSG_INJECT_SCROLL 3
#define SCRCPY_MSG_SET_CLIPBOARD 9
#define SCRCPY_MSG_SET_DISPLAY_POWER 10
#define SCRCPY_MSG_RESET_VIDEO 17
#define SCRCPY_DEVICE_MSG_CLIPBOARD 0
#define SCRCPY_MESSAGE_MAX_SIZE (1U << 18)
#define SCRCPY_SET_CLIPBOARD_HEADER 14
#define SCRCPY_CLIPBOARD_TEXT_MAX \
    (SCRCPY_MESSAGE_MAX_SIZE - SCRCPY_SET_CLIPBOARD_HEADER)

#define ANDROID_KEYCODE_BACK 4
#define ANDROID_KEYCODE_0 7
#define ANDROID_KEYCODE_A 29
#define ANDROID_KEYCODE_C 31
#define ANDROID_KEYCODE_V 50
#define ANDROID_KEYCODE_X 52
#define ANDROID_KEYCODE_F1 131
#define ANDROID_META_SHIFT_ON 0x1U
#define ANDROID_META_ALT_ON 0x2U
#define ANDROID_META_ALT_LEFT_ON 0x10U
#define ANDROID_META_ALT_RIGHT_ON 0x20U
#define ANDROID_META_SHIFT_LEFT_ON 0x40U
#define ANDROID_META_SHIFT_RIGHT_ON 0x80U
#define ANDROID_META_CTRL_ON 0x1000U
#define ANDROID_META_CTRL_LEFT_ON 0x2000U
#define ANDROID_META_CTRL_RIGHT_ON 0x4000U
#define ANDROID_META_META_ON 0x10000U
#define ANDROID_META_META_LEFT_ON 0x20000U
#define ANDROID_META_META_RIGHT_ON 0x40000U
#define PASTE_DEFER_NS UINT64_C(2000000000)
#define CLIPBOARD_RETRY_MIN_MS 250U
#define CLIPBOARD_RETRY_MAX_MS 8000U
#define CLIPBOARD_SESSION_STEADY_NS UINT64_C(10000000000)
#define CLIPBOARD_ECHO_NS UINT64_C(1000000000)

#define FRAME_QUEUE_CAPACITY 90
#define H264_MAX_CATCH_UP_FRAMES 6
#define DECODE_QUEUE_CAPACITY 32
#define MAX_PACKET_SIZE (8U * 1024U * 1024U)
#define FALLBACK_TILE_SIZE 32
#define FALLBACK_FAST_UPDATE_US 14000
#define FALLBACK_MEDIUM_UPDATE_US 30000
#define FALLBACK_ADAPT_INTERVAL_NS UINT64_C(1000000000)
#define FALLBACK_TCP_SEND_BUFFER (256 * 1024)
#define FALLBACK_TCP_NOTSENT_LOWAT (64 * 1024)
#define RESET_REQUEST_INTERVAL_NS UINT64_C(1000000000)
#define DECODE_KEYFRAME_WAIT_NS UINT64_C(1500000000)
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
    uint64_t ordinary_update_started_ns;
    int ordinary_requested_jpeg_quality;
    int ordinary_applied_jpeg_quality;
    int waiting_for_key_frame;
    int framebuffer_lock_held;
    int mode_logged;
    int previous_left_button;
    int previous_right_button;
    int previous_wheel_buttons;
    uint32_t held_modifiers;
    uint64_t paste_deferred_ns;
    int pastes_deferred;
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

struct bridge_options {
    const char *scrcpy_host;
    uint16_t scrcpy_port;
    uint16_t clipboard_port;
    int turn_screen_off;
    int self_test;
    int help;
};

static struct frame frame_queue[FRAME_QUEUE_CAPACITY];
static size_t frame_queue_head;
static size_t frame_queue_count;
static uint64_t frame_next_sequence = 1;
static pthread_mutex_t frame_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Packets for the fallback decoder thread: decoding off the video reader
 * thread means a slow decode never stalls the passthrough socket reads. */
static struct frame decode_queue[DECODE_QUEUE_CAPACITY];
static size_t decode_queue_head;
static size_t decode_queue_count;
static int decode_skip_until_key;
static uint64_t decode_keyframe_wait_started_ns;
static uint64_t decode_backlog_drop_count;
static pthread_mutex_t decode_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t decode_cond = PTHREAD_COND_INITIALIZER;

static uint8_t *codec_config;
static size_t codec_config_size;

static AVCodecContext *decoder_context;
static AVFrame *decoder_frame;
/* Newest decoded frame, handed from the decoder thread to the publisher under
 * fallback_mutex as a refcounted move: the critical section stays O(1) no
 * matter how long the publisher's scale/diff work takes. */
static AVFrame *pending_frame;
static AVFrame *publish_frame;
static struct SwsContext *sws_context;
static uint8_t *fallback_buffer;
static size_t framebuffer_size;
static struct scroll_row *scroll_old_rows;
static struct scroll_row *scroll_new_rows;
static int scroll_rows_capacity;
static int scroll_scores[2 * SCROLL_MAX_SHIFT + 1];
static int scroll_matches[2 * SCROLL_MAX_SHIFT + 1];
static int fallback_frame_ready;
static int fallback_decoder_ready_logged;
static int mismatch_width;
static int mismatch_height;
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
/* The clipboard text a client set last, so its echo is not sent back. */
static char *clipboard_text;
static size_t clipboard_length;
static uint64_t clipboard_set_ns;
static pthread_mutex_t clipboard_mutex = PTHREAD_MUTEX_INITIALIZER;

static int video_fd = -1;
static int control_fd = -1;
static int clipboard_fd = -1;
static pthread_mutex_t clipboard_fd_mutex = PTHREAD_MUTEX_INITIALIZER;
static const char *clipboard_host;
static uint16_t clipboard_port;
static int video_width;
static int video_height;
static volatile sig_atomic_t running = 1;
static volatile sig_atomic_t rfb_ready;
static rfbScreenInfoPtr rfb_screen;
static rfbClientPtr pointer_owner;

static void client_gone(rfbClientPtr client);
static int send_touch(uint8_t action, int x, int y, int pressed);
static int send_keycode(uint8_t action, int keycode, uint32_t meta);
static void log_client_mode_once(rfbClientPtr client,
                                 struct client_state *state);

static uint64_t monotonic_ns(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t) now.tv_sec * UINT64_C(1000000000)
         + (uint64_t) now.tv_nsec;
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

static void remove_arguments(int *argc, char **argv, int index, int count) {
    memmove(argv + index, argv + index + count,
            (size_t) (*argc - index - count + 1) * sizeof(*argv));
    *argc -= count;
}

static int parse_bridge_options(int *argc, char **argv,
                                struct bridge_options *options) {
    *options = (struct bridge_options) {
        .scrcpy_host = SCRCPY_DEFAULT_HOST,
        .scrcpy_port = SCRCPY_DEFAULT_PORT,
    };

    for (int i = 1; i < *argc;) {
        if (strcmp(argv[i], "--turn-screen-off") == 0) {
            options->turn_screen_off = 1;
            remove_arguments(argc, argv, i, 1);
            continue;
        }
        if (strcmp(argv[i], "--self-test") == 0) {
            options->self_test = 1;
            remove_arguments(argc, argv, i, 1);
            continue;
        }
        if (strcmp(argv[i], "--help") == 0) {
            options->help = 1;
            remove_arguments(argc, argv, i, 1);
            continue;
        }
        if (strcmp(argv[i], "--scrcpy-host") == 0) {
            if (i + 1 >= *argc) {
                fprintf(stderr, "--scrcpy-host requires an address\n");
                return -1;
            }
            options->scrcpy_host = argv[i + 1];
            remove_arguments(argc, argv, i, 2);
            continue;
        }
        if (strcmp(argv[i], "--scrcpy-port") == 0
                || strcmp(argv[i], "--clipboard-port") == 0) {
            if (i + 1 >= *argc) {
                fprintf(stderr, "%s requires a port\n", argv[i]);
                return -1;
            }
            char *end;
            long port = strtol(argv[i + 1], &end, 10);
            if (!argv[i + 1][0] || *end || port <= 0 || port > 65535) {
                fprintf(stderr, "invalid %s: %s\n", argv[i], argv[i + 1]);
                return -1;
            }
            if (strcmp(argv[i], "--scrcpy-port") == 0) {
                options->scrcpy_port = (uint16_t) port;
            } else {
                options->clipboard_port = (uint16_t) port;
            }
            remove_arguments(argc, argv, i, 2);
            continue;
        }
        ++i;
    }
    return 0;
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

static uint64_t pending_frames_locked(uint64_t next_sequence) {
    return next_sequence < frame_next_sequence
         ? frame_next_sequence - next_sequence
         : 0;
}

/* A client with no decoder state yet (waiting_for_key_frame) joins a queued
 * IDR only when it is near the live edge; a distant one is abandoned for a
 * fresh IDR, so a new client cannot burst a whole stale GOP before reaching
 * live video. It is parked meanwhile. A client that is already mid-stream is
 * never parked - that would strand it until the screen next changes - and any
 * keyframe ahead of its cursor is a shortcut, so distance does not matter. */
static void resync_h264_client_locked(struct client_state *state) {
    uint64_t latest_key = latest_key_sequence_locked();
    if (latest_key > state->next_sequence
            && (!state->waiting_for_key_frame
                || pending_frames_locked(latest_key)
                        <= H264_MAX_CATCH_UP_FRAMES)) {
        state->next_sequence = latest_key;
        state->waiting_for_key_frame = 0;
        return;
    }
    if (state->waiting_for_key_frame) {
        state->next_sequence = frame_next_sequence;
    }
}

/* If another client just requested an IDR, the global rate limit may suppress
 * this client's request. Replaying the cached GOP is a correctness fallback
 * for a static screen, where no later frame would wake the client again. */
static int replay_latest_gop_locked(struct client_state *state) {
    uint64_t latest_key = latest_key_sequence_locked();
    if (!latest_key) {
        return 0;
    }
    state->next_sequence = latest_key;
    state->waiting_for_key_frame = 0;
    return 1;
}

static int send_control(const void *message, size_t size) {
    pthread_mutex_lock(&control_mutex);
    int result = send_all(control_fd, message, size);
    pthread_mutex_unlock(&control_mutex);
    return result;
}

static void encode_display_power(uint8_t message[2], int on) {
    message[0] = SCRCPY_MSG_SET_DISPLAY_POWER;
    message[1] = on ? 1 : 0;
}

static int send_display_power(int on) {
    uint8_t message[2];
    encode_display_power(message, on);
    return send_control(message, sizeof(message));
}

/* Ask scrcpy for a fresh config packet and keyframe. Unless forced, at most
 * one request per second goes out no matter how many clients are waiting.
 * Returns 1 when sent, 0 when rate-limited, and -1 on a send failure. */
static int request_video_reset(int force) {
    const uint8_t message = SCRCPY_MSG_RESET_VIDEO;
    uint64_t now = monotonic_ns();
    int send_reset = 0;

    pthread_mutex_lock(&reset_mutex);
    if (force || !last_reset_request_ns
            || now - last_reset_request_ns >= RESET_REQUEST_INTERVAL_NS) {
        last_reset_request_ns = now;
        send_reset = 1;
    }
    pthread_mutex_unlock(&reset_mutex);

    if (!send_reset) {
        return 0;
    }
    if (send_control(&message, sizeof(message)) < 0) {
        fprintf(stderr, "failed to request a video reset\n");
        return -1;
    }
    return 1;
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

/* Non-blocking: copy the next access unit this client should send, resyncing
 * it when it has fallen behind (see resync_h264_client_locked). Returns 0 when
 * it must wait for a later enqueue; *needs_keyframe means the stream cannot
 * resume without a fresh keyframe, *more_pending that another eligible frame
 * is already queued behind the returned one. */
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

        if (pending_frames_locked(state->next_sequence)
                > H264_MAX_CATCH_UP_FRAMES) {
            resync_h264_client_locked(state);
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
    pending_frame = av_frame_alloc();
    publish_frame = av_frame_alloc();
    if (!decoder_context || !decoder_frame || !pending_frame || !publish_frame
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
            /* A rotated phone produces this for every frame from here on, so
             * log the transition rather than the frame rate. */
            if (decoder_frame->width != mismatch_width
                    || decoder_frame->height != mismatch_height) {
                mismatch_width = decoder_frame->width;
                mismatch_height = decoder_frame->height;
                fprintf(stderr,
                        "fallback decoder size mismatch: %dx%d "
                        "(session is %dx%d; resize requires restart)\n",
                        decoder_frame->width, decoder_frame->height,
                        video_width, video_height);
            }
            av_frame_unref(decoder_frame);
            continue;
        }
        mismatch_width = 0;
        mismatch_height = 0;

        pthread_mutex_lock(&fallback_mutex);
        av_frame_unref(pending_frame);
        av_frame_move_ref(pending_frame, decoder_frame);
        fallback_frame_ready = 1;
        if (!fallback_decoder_ready_logged) {
            fprintf(stderr,
                    "ordinary VNC decoder ready: %dx%d format=%d\n",
                    pending_frame->width, pending_frame->height,
                    pending_frame->format);
            fallback_decoder_ready_logged = 1;
        }
        pthread_mutex_unlock(&fallback_mutex);
    }
}

static void clear_decode_queue_locked(void) {
    while (decode_queue_count) {
        free_frame(&decode_queue[decode_queue_head]);
        decode_queue_head = (decode_queue_head + 1) % DECODE_QUEUE_CAPACITY;
        --decode_queue_count;
    }
}

enum decode_resync_action {
    DECODE_RESYNC_PROCESS,
    DECODE_RESYNC_DROP,
    DECODE_RESYNC_REQUEST_KEYFRAME,
};

/* A full fallback queue means the decoder is no longer at the live edge.
 * Discard the stale GOP and wait briefly for the next encoder keyframe. The
 * scrcpy server is reset only if an active stream fails to provide one within
 * the bounded wait; resetting immediately on every overflow tears down the
 * capture pipeline and can turn ordinary VNC load into a reset storm.
 * DECODE_KEYFRAME_WAIT_NS therefore has to exceed the encoder's configured
 * i-frame-interval, or the wait expires before the keyframe it waits for. */
static enum decode_resync_action decode_resync_action_locked(int key_frame,
                                                            uint64_t now_ns) {
    if (!decode_skip_until_key) {
        return DECODE_RESYNC_PROCESS;
    }
    if (key_frame) {
        decode_skip_until_key = 0;
        decode_keyframe_wait_started_ns = 0;
        return DECODE_RESYNC_PROCESS;
    }
    if (now_ns - decode_keyframe_wait_started_ns
            >= DECODE_KEYFRAME_WAIT_NS) {
        decode_keyframe_wait_started_ns = now_ns;
        return DECODE_RESYNC_REQUEST_KEYFRAME;
    }
    return DECODE_RESYNC_DROP;
}

/* Hand a copy of the packet to the decoder thread. When decoding cannot keep
 * up, drop the backlog and resynchronize from the next keyframe instead of
 * stalling the video socket. */
static void decode_submit(const uint8_t *data, size_t size, int key_frame) {
    uint8_t *copy;
    size_t tail;
    uint64_t now_ns;

    if (!fallback_mode) {
        return;
    }

    now_ns = monotonic_ns();
    pthread_mutex_lock(&decode_mutex);
    enum decode_resync_action action =
            decode_resync_action_locked(key_frame, now_ns);
    if (action != DECODE_RESYNC_PROCESS) {
        pthread_mutex_unlock(&decode_mutex);
        if (action == DECODE_RESYNC_REQUEST_KEYFRAME
                && request_video_reset(0) > 0) {
            fprintf(stderr,
                    "fallback decoder waited %.1f ms for a keyframe; "
                    "requested one\n",
                    (double) DECODE_KEYFRAME_WAIT_NS / 1000000.0);
        }
        return;
    }

    if (decode_queue_count == DECODE_QUEUE_CAPACITY) {
        clear_decode_queue_locked();
        fallback_decoder_reset = 1;
        if (!key_frame) {
            decode_skip_until_key = 1;
            decode_keyframe_wait_started_ns = now_ns;
            uint64_t drop_count = ++decode_backlog_drop_count;
            pthread_mutex_unlock(&decode_mutex);
            if (drop_count <= 4 || !(drop_count & (drop_count - 1))) {
                fprintf(stderr,
                        "fallback decode backlog dropped at live edge "
                        "(count=%llu); waiting for next keyframe\n",
                        (unsigned long long) drop_count);
            }
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

        uint64_t pts_flags = AV_RB64(header);
        if (pts_flags & SCRCPY_PACKET_FLAG_SESSION) {
            int width = (int) AV_RB32(header + 4);
            int height = (int) AV_RB32(header + 8);
            fprintf(stderr, "scrcpy session: %dx%d%s\n", width, height,
                    (width == video_width && height == video_height)
                        ? "" : " (resize requires restart)");
            continue;
        }

        uint32_t packet_size = AV_RB32(header + 8);
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

/* Reads the device message after its type byte: the Android clipboard into
 * *text (NUL-terminated, the caller frees it). The bridge sets the clipboard
 * without a sequence and creates no UHID device, so any other type is -1, as
 * is a read error. */
static int read_device_message(int fd, uint8_t type, char **text,
                               uint32_t *length) {
    uint8_t header[4];
    if (type != SCRCPY_DEVICE_MSG_CLIPBOARD
            || recv_all(fd, header, sizeof(header)) <= 0) {
        return -1;
    }
    *length = AV_RB32(header);
    if (*length > SCRCPY_MESSAGE_MAX_SIZE) {
        return -1;
    }
    *text = malloc((size_t) *length + 1);
    if (!*text || (*length && recv_all(fd, *text, *length) <= 0)) {
        free(*text);
        *text = NULL;
        return -1;
    }
    (*text)[*length] = 0;
    return 0;
}

static size_t utf8_to_latin1(const uint8_t *in, size_t length, char *out) {
    size_t n = 0;
    size_t i = 0;
    while (i < length) {
        uint8_t lead = in[i];
        size_t extra = lead < 0x80 ? 0
                     : (lead & 0xe0) == 0xc0 ? 1
                     : (lead & 0xf0) == 0xe0 ? 2
                     : (lead & 0xf8) == 0xf0 ? 3 : 4;
        uint32_t code_point = extra ? lead & (0x3fU >> extra) : lead;
        size_t j = 1;
        while (extra < 4 && j <= extra && i + j < length
                && (in[i + j] & 0xc0) == 0x80) {
            code_point = (code_point << 6) | (in[i + j] & 0x3f);
            ++j;
        }
        if (extra == 4 || j <= extra) {
            out[n++] = '?';
            ++i;
            continue;
        }
        out[n++] = code_point <= 0xff ? (char) code_point : '?';
        i += j;
    }
    return n;
}

static void remember_client_clipboard(const char *text, size_t length) {
    char *copy = malloc(length ? length : 1);
    if (copy) {
        memcpy(copy, text, length);
    }
    pthread_mutex_lock(&clipboard_mutex);
    free(clipboard_text);
    clipboard_text = copy;
    clipboard_length = copy ? length : 0;
    clipboard_set_ns = monotonic_ns();
    pthread_mutex_unlock(&clipboard_mutex);
}

/* An Android clipboard equal to the text a client set within
 * CLIPBOARD_ECHO_NS is that set coming back. */
static int clipboard_echo(const char *text, size_t length) {
    pthread_mutex_lock(&clipboard_mutex);
    int echo = clipboard_text && length == clipboard_length
            && !memcmp(text, clipboard_text, length)
            && monotonic_ns() - clipboard_set_ns < CLIPBOARD_ECHO_NS;
    pthread_mutex_unlock(&clipboard_mutex);
    return echo;
}

static void send_server_clipboard(char *text, uint32_t length) {
    char *latin1 = malloc((size_t) length + 1);
    if (!latin1) {
        return;
    }
    size_t latin1_length = utf8_to_latin1((const uint8_t *) text, length,
                                          latin1);
    rfbSendServerCutTextUTF8(rfb_screen, text, (int) length, latin1,
                             (int) latin1_length);
    free(latin1);
}

/* Sends each new Android clipboard on fd to every client as ServerCutText
 * until the stream ends (0) or turns unreadable (-1). */
static int read_device_messages(int fd) {
    while (running) {
        uint8_t type;
        char *text = NULL;
        uint32_t length = 0;
        if (recv_all(fd, &type, 1) <= 0) {
            return 0;
        }
        if (read_device_message(fd, type, &text, &length) < 0) {
            fprintf(stderr, "unreadable scrcpy device message type %u\n", type);
            return -1;
        }
        if (rfb_ready && !clipboard_echo(text, length)) {
            send_server_clipboard(text, length);
        }
        free(text);
    }
    return 0;
}

/* Once the control stream is unreadable, the rest is drained so the server
 * never blocks on a full socket. */
static void *control_reader_main(void *arg) {
    int fd = (int) (intptr_t) arg;
    uint8_t buffer[4096];
    if (read_device_messages(fd) == 0) {
        return NULL;
    }
    while (running) {
        ssize_t n = recv(fd, buffer, sizeof(buffer), 0);
        if (n <= 0) {
            break;
        }
    }
    return NULL;
}

static void pause_while_running(unsigned ms) {
    for (unsigned waited = 0; running && waited < ms; waited += 50) {
        struct timespec delay = {.tv_sec = 0, .tv_nsec = 50 * 1000 * 1000};
        nanosleep(&delay, NULL);
    }
}

/* The shell session carries Android's clipboard; the box restarts it. */
static void *clipboard_reader_main(void *arg) {
    (void) arg;
    unsigned retry_ms = CLIPBOARD_RETRY_MIN_MS;
    while (running) {
        pthread_mutex_lock(&clipboard_fd_mutex);
        int fd = clipboard_fd;
        pthread_mutex_unlock(&clipboard_fd_mutex);
        uint64_t started_ns = monotonic_ns();
        int status = fd >= 0 ? read_device_messages(fd) : 0;
        if (!running) {
            break;
        }
        if (monotonic_ns() - started_ns >= CLIPBOARD_SESSION_STEADY_NS) {
            retry_ms = CLIPBOARD_RETRY_MIN_MS;
        }
        fprintf(stderr, "scrcpy clipboard session %s; reconnecting in %u ms\n",
                status < 0 ? "is unreadable" : fd >= 0 ? "ended" : "did not connect",
                retry_ms);
        pause_while_running(retry_ms);
        retry_ms = retry_ms * 2 < CLIPBOARD_RETRY_MAX_MS
                 ? retry_ms * 2 : CLIPBOARD_RETRY_MAX_MS;
        int next = running ? connect_tcp(clipboard_host, clipboard_port) : -1;
        pthread_mutex_lock(&clipboard_fd_mutex);
        if (clipboard_fd >= 0) {
            close(clipboard_fd);
        }
        clipboard_fd = next;
        if (!running && clipboard_fd >= 0) {
            shutdown(clipboard_fd, SHUT_RDWR);
        }
        pthread_mutex_unlock(&clipboard_fd_mutex);
    }
    return NULL;
}

/* Non-blocking Open H.264 frame hook. A client stuck waiting for a keyframe
 * (connected while the screen was static) triggers a rate-limited encoder
 * reset so its stream starts without waiting for on-screen motion. */
static rfbBool h264_frame_hook(rfbClientPtr client, char **buffer,
                               size_t *size) {
    struct frame frame = {0};
    int needs_keyframe = 0;
    int more_pending = 0;
    struct client_state *state = client->clientData;

    log_client_mode_once(client, state);
    if (!copy_next_frame(state, &frame, &needs_keyframe, &more_pending)) {
        if (needs_keyframe) {
            int reset_sent = request_video_reset(0);
            if (reset_sent <= 0) {
                pthread_mutex_lock(&frame_mutex);
                int replay_stale = replay_latest_gop_locked(state);
                pthread_mutex_unlock(&frame_mutex);
                if (replay_stale) {
                    rfbNotifyH264FrameAvailable(rfb_screen);
                }
            }
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
    state->waiting_for_key_frame = 1;
    resync_h264_client_locked(state);
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
    if (state && state->previous_right_button) {
        send_keycode(1, ANDROID_KEYCODE_BACK, 0);
    }
    free(client->clientData);
    client->clientData = NULL;
    fprintf(stderr, "VNC client disconnected\n");
}

static int send_touch(uint8_t action, int x, int y, int pressed) {
    uint8_t message[32] = {0};
    message[0] = SCRCPY_MSG_INJECT_TOUCH;
    message[1] = action;
    AV_WB64(message + 2, SCRCPY_POINTER_ID_MOUSE);
    AV_WB32(message + 10, (uint32_t) x);
    AV_WB32(message + 14, (uint32_t) y);
    AV_WB16(message + 18, (uint16_t) video_width);
    AV_WB16(message + 20, (uint16_t) video_height);
    AV_WB16(message + 22, pressed ? UINT16_MAX : 0);
    AV_WB32(message + 24, pressed ? 1U : 0U);
    AV_WB32(message + 28, pressed ? 1U : 0U);
    return send_control(message, sizeof(message));
}

/* hscroll/vscroll are one wheel click each. scrcpy's i16 fixed-point spans
 * [-16, 16] clicks full-scale (the server multiplies the [-1, 1] decode by
 * 16), so one click is 32768 / 16 = 2048. Positive vscroll scrolls up. */
static int send_scroll(int x, int y, int hscroll, int vscroll) {
    uint8_t message[21] = {0};
    message[0] = SCRCPY_MSG_INJECT_SCROLL;
    AV_WB32(message + 1, (uint32_t) x);
    AV_WB32(message + 5, (uint32_t) y);
    AV_WB16(message + 9, (uint16_t) video_width);
    AV_WB16(message + 11, (uint16_t) video_height);
    AV_WB16(message + 13, (uint16_t) (int16_t) (hscroll * 2048));
    AV_WB16(message + 15, (uint16_t) (int16_t) (vscroll * 2048));
    AV_WB32(message + 17, 0);
    return send_control(message, sizeof(message));
}

static void pointer_event(int button_mask, int x, int y, rfbClientPtr client) {
    struct client_state *state = client->clientData;

    /* RFB buttons 4-7 (mask bits 3-6) are wheel up/down/left/right; inject
     * on the press edge, independent of pointer-drag ownership. */
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

    int right_button = !!(button_mask & 4);
    if (right_button != state->previous_right_button) {
        state->previous_right_button = right_button;
        send_keycode(right_button ? 0 : 1, ANDROID_KEYCODE_BACK, 0);
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

static void encode_keycode(uint8_t message[14], uint8_t action, int keycode,
                           uint32_t meta) {
    memset(message, 0, 14);
    message[0] = SCRCPY_MSG_INJECT_KEYCODE;
    message[1] = action;
    AV_WB32(message + 2, (uint32_t) keycode);
    AV_WB32(message + 10, meta);
}

static int send_keycode(uint8_t action, int keycode, uint32_t meta) {
    uint8_t message[14];
    encode_keycode(message, action, keycode, meta);
    return send_control(message, sizeof(message));
}

static void send_key_press(int keycode, uint32_t meta) {
    send_keycode(0, keycode, meta);
    send_keycode(1, keycode, meta);
}

/* The Android META_*_LEFT_ON or META_*_RIGHT_ON flag of a modifier keysym,
 * 0 for any other key. Super is Android's Meta. */
static uint32_t modifier_side_flag(rfbKeySym key) {
    switch (key) {
        case XK_Shift_L: return ANDROID_META_SHIFT_LEFT_ON;
        case XK_Shift_R: return ANDROID_META_SHIFT_RIGHT_ON;
        case XK_Control_L: return ANDROID_META_CTRL_LEFT_ON;
        case XK_Control_R: return ANDROID_META_CTRL_RIGHT_ON;
        case XK_Alt_L: return ANDROID_META_ALT_LEFT_ON;
        case XK_Alt_R: return ANDROID_META_ALT_RIGHT_ON;
        case XK_Meta_L:
        case XK_Super_L: return ANDROID_META_META_LEFT_ON;
        case XK_Meta_R:
        case XK_Super_R: return ANDROID_META_META_RIGHT_ON;
        default: return 0;
    }
}

static uint32_t android_meta_state(uint32_t held) {
    uint32_t meta = held;
    if (held & (ANDROID_META_SHIFT_LEFT_ON | ANDROID_META_SHIFT_RIGHT_ON)) {
        meta |= ANDROID_META_SHIFT_ON;
    }
    if (held & (ANDROID_META_ALT_LEFT_ON | ANDROID_META_ALT_RIGHT_ON)) {
        meta |= ANDROID_META_ALT_ON;
    }
    if (held & (ANDROID_META_CTRL_LEFT_ON | ANDROID_META_CTRL_RIGHT_ON)) {
        meta |= ANDROID_META_CTRL_ON;
    }
    if (held & (ANDROID_META_META_LEFT_ON | ANDROID_META_META_RIGHT_ON)) {
        meta |= ANDROID_META_META_ON;
    }
    return meta;
}

static int shortcut_keycode(rfbKeySym key, uint32_t meta,
                            uint32_t *shortcut_meta) {
    *shortcut_meta = ANDROID_META_CTRL_ON | ANDROID_META_CTRL_LEFT_ON;
    if (key == XK_Insert && (meta & ANDROID_META_CTRL_ON)) {
        return ANDROID_KEYCODE_C;
    }
    if (key == XK_Insert && (meta & ANDROID_META_SHIFT_ON)) {
        return ANDROID_KEYCODE_V;
    }
    if (key == XK_Delete && (meta & ANDROID_META_SHIFT_ON)) {
        return ANDROID_KEYCODE_X;
    }
    if (!(meta & (ANDROID_META_CTRL_ON | ANDROID_META_META_ON))) {
        return -1;
    }
    *shortcut_meta = meta;
    if (key >= XK_a && key <= XK_z) {
        return ANDROID_KEYCODE_A + (int) (key - XK_a);
    }
    if (key >= XK_A && key <= XK_Z) {
        return ANDROID_KEYCODE_A + (int) (key - XK_A);
    }
    if (key >= XK_0 && key <= XK_9) {
        return ANDROID_KEYCODE_0 + (int) (key - XK_0);
    }
    return -1;
}

static int keysym_to_android_keycode(rfbKeySym key) {
    if (key >= XK_F1 && key <= XK_F12) {
        return ANDROID_KEYCODE_F1 + (int) (key - XK_F1);
    }
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
        case XK_End: return 123;
        case XK_Insert: return 124;
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

/* A paste sent while the client's announced text is still being requested
 * waits for that text; each press counts. A press after PASTE_DEFER_NS
 * without the text ends the wait and pastes Android's clipboard. */
static int defer_paste(rfbClientPtr client, struct client_state *state) {
    if (!client->extClipboardRequested) {
        return 0;
    }
    uint64_t now = monotonic_ns();
    if (state->pastes_deferred
            && now - state->paste_deferred_ns >= PASTE_DEFER_NS) {
        client->extClipboardRequested = FALSE;
        state->pastes_deferred = 0;
        return 0;
    }
    if (!state->pastes_deferred) {
        state->paste_deferred_ns = now;
    }
    ++state->pastes_deferred;
    return 1;
}

/* Copies text into a SET_CLIPBOARD message without its trailing NULs and
 * with CRLF as LF, cut to the scrcpy limit on a UTF-8 boundary. message
 * holds SCRCPY_SET_CLIPBOARD_HEADER + min(length, SCRCPY_CLIPBOARD_TEXT_MAX)
 * bytes; returns the message size. */
static size_t encode_set_clipboard(const char *text, size_t length,
                                   uint8_t *message) {
    while (length && !text[length - 1]) {
        --length;
    }
    uint8_t *out = message + SCRCPY_SET_CLIPBOARD_HEADER;
    size_t n = 0;
    size_t i = 0;
    for (; i < length && n < SCRCPY_CLIPBOARD_TEXT_MAX; ++i) {
        if (text[i] == '\r' && i + 1 < length && text[i + 1] == '\n') {
            continue;
        }
        out[n++] = (uint8_t) text[i];
    }
    if (i < length) {
        uint8_t next = (uint8_t) text[i];
        while (n && (next & 0xc0) == 0x80) {
            next = out[--n];
        }
    }
    memset(message, 0, SCRCPY_SET_CLIPBOARD_HEADER);
    message[0] = SCRCPY_MSG_SET_CLIPBOARD;
    AV_WB32(message + 10, (uint32_t) n);
    return SCRCPY_SET_CLIPBOARD_HEADER + n;
}

/* The client's clipboard becomes Android's, with no paste of its own: the
 * client's paste key does that, or the paste deferred for this text. */
static void forward_clipboard(const char *text, size_t length,
                              rfbClientPtr client) {
    size_t capacity = SCRCPY_SET_CLIPBOARD_HEADER
                    + (length < SCRCPY_CLIPBOARD_TEXT_MAX
                           ? length : SCRCPY_CLIPBOARD_TEXT_MAX);
    uint8_t *message = malloc(capacity);
    if (!message) {
        return;
    }
    size_t size = encode_set_clipboard(text, length, message);
    remember_client_clipboard(
            (const char *) message + SCRCPY_SET_CLIPBOARD_HEADER,
            size - SCRCPY_SET_CLIPBOARD_HEADER);
    send_control(message, size);
    free(message);
    struct client_state *state = client->clientData;
    int pastes = monotonic_ns() - state->paste_deferred_ns < PASTE_DEFER_NS
               ? state->pastes_deferred : 0;
    state->pastes_deferred = 0;
    for (int i = 0; i < pastes; ++i) {
        send_key_press(ANDROID_KEYCODE_V,
                       ANDROID_META_CTRL_ON | ANDROID_META_CTRL_LEFT_ON);
    }
}

static void set_clipboard_utf8(char *text, int length, rfbClientPtr client) {
    forward_clipboard(text, (size_t) length, client);
}

static void set_clipboard_latin1(char *text, int length,
                                 rfbClientPtr client) {
    char *utf8 = malloc(2 * (size_t) length + 1);
    if (!utf8) {
        return;
    }
    size_t n = 0;
    for (int i = 0; i < length; ++i) {
        n += utf8_encode((uint8_t) text[i], (uint8_t *) utf8 + n);
    }
    forward_clipboard(utf8, n, client);
    free(utf8);
}

/* Modifiers set the meta state of the keys that follow. Printable keysyms
 * become scrcpy UTF-8 text injection: ASCII and Latin-1 map directly, X11
 * Unicode keysyms carry the code point in the low bits. */
static void keyboard_event(rfbBool down, rfbKeySym key, rfbClientPtr client) {
    struct client_state *state = client->clientData;
    uint32_t side = modifier_side_flag(key);
    if (side) {
        if (down) {
            state->held_modifiers |= side;
        } else {
            state->held_modifiers &= ~side;
        }
        return;
    }

    uint32_t meta = android_meta_state(state->held_modifiers);
    uint32_t shortcut_meta;
    int keycode = shortcut_keycode(key, meta, &shortcut_meta);
    if (keycode >= 0) {
        if (down && !(keycode == ANDROID_KEYCODE_V
                      && defer_paste(client, state))) {
            send_key_press(keycode, shortcut_meta);
        }
        return;
    }
    keycode = keysym_to_android_keycode(key);
    if (keycode >= 0) {
        send_keycode(down ? 0 : 1, keycode, meta);
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
    AV_WB32(message + 1, (uint32_t) length);
    send_control(message, 5 + length);
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

    /* Rows outside [top,bottom) are never read, so no zero-fill needed */
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

/* Scratch for detect_vertical_scroll, allocated once at the session height:
 * the detector runs per published frame and must not pay four malloc/free
 * cycles each time. --self-test returns from main before any session exists,
 * so the two never coexist. */
static int reserve_scroll_rows(int height) {
    if (height <= scroll_rows_capacity) {
        return 1;
    }
    struct scroll_row *grown_old = realloc(scroll_old_rows,
                                           (size_t) height * sizeof(*grown_old));
    if (!grown_old) {
        return 0;
    }
    scroll_old_rows = grown_old;
    struct scroll_row *grown_new = realloc(scroll_new_rows,
                                           (size_t) height * sizeof(*grown_new));
    if (!grown_new) {
        return 0;
    }
    scroll_new_rows = grown_new;
    scroll_rows_capacity = height;
    return 1;
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

    if (!reserve_scroll_rows(height)) {
        return 0;
    }
    struct scroll_row *old_rows = scroll_old_rows;
    struct scroll_row *new_rows = scroll_new_rows;
    /* the |dy| < SCROLL_MIN_SHIFT band is skipped by the scoring loop but
     * still read when picking the best, so it must start zeroed */
    memset(scroll_scores, 0, sizeof(scroll_scores));
    memset(scroll_matches, 0, sizeof(scroll_matches));
    int *scores = scroll_scores;
    int *matches = scroll_matches;

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

    if (best_matches < 10 || best_score < 450
            || (best_score < 800 && best_score - second_score < 100)) {
        return 0;
    }
    return best_dy;
}

static void reset_frame_queue_for_test(void) {
    pthread_mutex_lock(&frame_mutex);
    clear_frame_queue_locked();
    frame_queue_head = 0;
    frame_next_sequence = 1;
    pthread_mutex_unlock(&frame_mutex);
}

static int enqueue_test_frame(uint8_t marker, int key_frame) {
    uint8_t *data = malloc(1);
    if (!data) {
        return -1;
    }
    data[0] = marker;
    enqueue_frame(data, 1, key_frame);
    return 0;
}

static int h264_live_edge_self_test(void) {
    struct client_state recent = {.waiting_for_key_frame = 1};
    struct client_state new_stale = {.waiting_for_key_frame = 1};
    struct client_state mid_skip = {.next_sequence = 1};
    struct client_state desynced = {.next_sequence = 1,
                                    .waiting_for_key_frame = 1};
    struct frame frame = {0};
    const char *failure = NULL;
    int needs_keyframe = 0;
    int more_pending = 0;
    uint64_t stale_live_edge;
    uint64_t fresh_key;

    reset_frame_queue_for_test();
    if (enqueue_test_frame(0x11, 1) < 0) {
        failure = "failed to allocate recent keyframe";
        goto failed;
    }
    for (int i = 1; i < H264_MAX_CATCH_UP_FRAMES; ++i) {
        if (enqueue_test_frame((uint8_t) (0x11 + i), 0) < 0) {
            failure = "failed to allocate recent delta frame";
            goto failed;
        }
    }

    pthread_mutex_lock(&frame_mutex);
    resync_h264_client_locked(&recent);
    pthread_mutex_unlock(&frame_mutex);
    if (recent.next_sequence != 1 || recent.waiting_for_key_frame
            || !copy_next_frame(&recent, &frame, &needs_keyframe,
                                &more_pending)
            || !frame.key_frame || frame.sequence != 1
            || frame.size != 1 || frame.data[0] != 0x11
            || recent.next_sequence != 2 || recent.waiting_for_key_frame
            || needs_keyframe || !more_pending) {
        failure = "recent keyframe was not used for bounded catch-up";
        goto failed;
    }
    free_frame(&frame);

    if (enqueue_test_frame(0x21, 0) < 0) {
        failure = "failed to extend the GOP past the catch-up limit";
        goto failed;
    }
    if (enqueue_test_frame(0x22, 0) < 0) {
        failure = "failed to extend the cached GOP replay";
        goto failed;
    }

    pthread_mutex_lock(&frame_mutex);
    stale_live_edge = frame_next_sequence;
    resync_h264_client_locked(&new_stale);
    pthread_mutex_unlock(&frame_mutex);
    if (new_stale.next_sequence != stale_live_edge
            || !new_stale.waiting_for_key_frame) {
        failure = "new client did not abandon a stale GOP";
        goto failed;
    }

    pthread_mutex_lock(&frame_mutex);
    int replay_stale = replay_latest_gop_locked(&new_stale);
    pthread_mutex_unlock(&frame_mutex);
    if (!replay_stale
            || !copy_next_frame(&new_stale, &frame, &needs_keyframe,
                                &more_pending)
            || !frame.key_frame || frame.sequence != 1
            || new_stale.next_sequence != 2
            || new_stale.waiting_for_key_frame
            || needs_keyframe || !more_pending) {
        failure = "cached GOP replay did not start from its keyframe";
        goto failed;
    }
    free_frame(&frame);

    if (!copy_next_frame(&new_stale, &frame, &needs_keyframe, &more_pending)
            || frame.key_frame || frame.sequence != 2
            || new_stale.next_sequence != 3
            || new_stale.waiting_for_key_frame
            || needs_keyframe || !more_pending) {
        failure = "cached GOP replay did not drain past the threshold";
        goto failed;
    }
    free_frame(&frame);

    if (copy_next_frame(&desynced, &frame, &needs_keyframe, &more_pending)
            || desynced.next_sequence != stale_live_edge
            || !desynced.waiting_for_key_frame
            || !needs_keyframe || more_pending) {
        failure = "desynced client did not move to the live edge";
        goto failed;
    }

    if (enqueue_test_frame(0x30, 0) < 0
            || copy_next_frame(&desynced, &frame, &needs_keyframe,
                               &more_pending)
            || desynced.next_sequence != stale_live_edge
            || !desynced.waiting_for_key_frame || !needs_keyframe) {
        failure = "parked client took a delta it cannot decode";
        goto failed;
    }

    pthread_mutex_lock(&frame_mutex);
    fresh_key = frame_next_sequence;
    pthread_mutex_unlock(&frame_mutex);
    if (enqueue_test_frame(0x31, 1) < 0
            || !copy_next_frame(&desynced, &frame, &needs_keyframe,
                                &more_pending)
            || !frame.key_frame || frame.sequence != fresh_key
            || frame.size != 1 || frame.data[0] != 0x31
            || desynced.next_sequence != fresh_key + 1
            || desynced.waiting_for_key_frame
            || needs_keyframe || more_pending) {
        failure = "client did not resume from a fresh keyframe";
        goto failed;
    }
    free_frame(&frame);

    if (copy_next_frame(&desynced, &frame, &needs_keyframe, &more_pending)
            || needs_keyframe || more_pending) {
        failure = "drained client asked for a keyframe it does not need";
        goto failed;
    }

    /* Push that keyframe past the catch-up limit: a mid-stream client must
     * still skip to it, because any keyframe ahead is less to send than its
     * backlog. Only a client waiting for one cares how recent it is. */
    for (int i = 0; i <= H264_MAX_CATCH_UP_FRAMES; ++i) {
        if (enqueue_test_frame(0x41, 0) < 0) {
            failure = "failed to push the fresh keyframe past the limit";
            goto failed;
        }
    }
    if (!copy_next_frame(&mid_skip, &frame, &needs_keyframe, &more_pending)
            || !frame.key_frame || frame.sequence != fresh_key
            || mid_skip.next_sequence != fresh_key + 1
            || mid_skip.waiting_for_key_frame
            || needs_keyframe || !more_pending) {
        failure = "mid-stream client did not skip to the keyframe ahead";
        goto failed;
    }

    free_frame(&frame);
    reset_frame_queue_for_test();
    return 0;

failed:
    fprintf(stderr, "H.264 live-edge self-test failed: %s\n", failure);
    free_frame(&frame);
    reset_frame_queue_for_test();
    return 1;
}

static int fallback_decode_resync_self_test(void) {
    int saved_skip_until_key = decode_skip_until_key;
    uint64_t saved_wait_started_ns = decode_keyframe_wait_started_ns;
    uint64_t saved_backlog_drop_count = decode_backlog_drop_count;
    uint64_t saved_last_reset_request_ns = last_reset_request_ns;
    int saved_fallback_mode = fallback_mode;
    int saved_fallback_decoder_reset = fallback_decoder_reset;
    int saved_control_fd = control_fd;
    int control_pair[2] = {-1, -1};
    const uint64_t started_ns = UINT64_C(1000000000);
    const char *failure = NULL;
    uint8_t packet = 0x42;

    pthread_mutex_lock(&decode_mutex);
    decode_skip_until_key = 1;
    decode_keyframe_wait_started_ns = started_ns;
    enum decode_resync_action before_wait = decode_resync_action_locked(
            0, started_ns + DECODE_KEYFRAME_WAIT_NS - 1);
    enum decode_resync_action at_wait = decode_resync_action_locked(
            0, started_ns + DECODE_KEYFRAME_WAIT_NS);
    uint64_t wait_restarted_ns = decode_keyframe_wait_started_ns;
    enum decode_resync_action after_request = decode_resync_action_locked(
            0, started_ns + 2 * DECODE_KEYFRAME_WAIT_NS - 1);
    enum decode_resync_action on_key_frame = decode_resync_action_locked(
            1, started_ns + 2 * DECODE_KEYFRAME_WAIT_NS);
    int skipping_after_key = decode_skip_until_key;
    uint64_t waiting_after_key = decode_keyframe_wait_started_ns;
    pthread_mutex_unlock(&decode_mutex);

    if (before_wait != DECODE_RESYNC_DROP) {
        failure = "requested a keyframe before the bounded wait elapsed";
        goto done;
    }
    if (at_wait != DECODE_RESYNC_REQUEST_KEYFRAME
            || wait_restarted_ns != started_ns + DECODE_KEYFRAME_WAIT_NS) {
        failure = "did not request a keyframe after the bounded wait";
        goto done;
    }
    if (after_request != DECODE_RESYNC_DROP) {
        failure = "repeated the keyframe request without another wait";
        goto done;
    }
    if (on_key_frame != DECODE_RESYNC_PROCESS
            || skipping_after_key || waiting_after_key) {
        failure = "natural keyframe did not resume fallback decoding";
        goto done;
    }

    pthread_mutex_lock(&decode_mutex);
    clear_decode_queue_locked();
    decode_queue_head = 0;
    for (size_t i = 0; i < DECODE_QUEUE_CAPACITY; ++i) {
        decode_queue[i].data = malloc(1);
        if (!decode_queue[i].data) {
            pthread_mutex_unlock(&decode_mutex);
            failure = "failed to allocate the overflow test queue";
            goto done;
        }
        decode_queue[i].data[0] = (uint8_t) i;
        decode_queue[i].size = 1;
        ++decode_queue_count;
    }
    pthread_mutex_unlock(&decode_mutex);

    fallback_mode = 1;
    decode_skip_until_key = 0;
    decode_keyframe_wait_started_ns = 0;
    last_reset_request_ns = started_ns;
    uint64_t backlog_drop_count = decode_backlog_drop_count;
    decode_submit(&packet, sizeof(packet), 0);
    if (decode_queue_count || !decode_skip_until_key
            || !decode_keyframe_wait_started_ns
            || decode_backlog_drop_count != backlog_drop_count + 1
            || last_reset_request_ns != started_ns) {
        failure = "queue overflow did not wait for a natural keyframe";
        goto done;
    }

    /* decode_submit's resync branch holds the only keyframe request this path
     * adds, so drive it over a real control socket and read the message back
     * rather than inferring it from the rate-limiter timestamp. */
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, control_pair) < 0) {
        failure = "failed to create the control socket pair";
        goto done;
    }
    control_fd = control_pair[0];

    last_reset_request_ns = 0;
    pthread_mutex_lock(&decode_mutex);
    decode_keyframe_wait_started_ns = monotonic_ns();
    pthread_mutex_unlock(&decode_mutex);
    decode_submit(&packet, sizeof(packet), 0);
    if (decode_queue_count || last_reset_request_ns) {
        failure = "packet was queued or reset the encoder during the wait";
        goto done;
    }

    pthread_mutex_lock(&decode_mutex);
    decode_keyframe_wait_started_ns = monotonic_ns() - DECODE_KEYFRAME_WAIT_NS;
    pthread_mutex_unlock(&decode_mutex);
    decode_submit(&packet, sizeof(packet), 0);
    uint8_t reset_message = 0;
    if (decode_queue_count || !last_reset_request_ns
            || recv(control_pair[1], &reset_message, 1, MSG_DONTWAIT) != 1
            || reset_message != SCRCPY_MSG_RESET_VIDEO) {
        failure = "no keyframe was requested after the bounded wait elapsed";
        goto done;
    }

done:
    pthread_mutex_lock(&decode_mutex);
    clear_decode_queue_locked();
    decode_queue_head = 0;
    pthread_mutex_unlock(&decode_mutex);
    decode_skip_until_key = saved_skip_until_key;
    decode_keyframe_wait_started_ns = saved_wait_started_ns;
    decode_backlog_drop_count = saved_backlog_drop_count;
    last_reset_request_ns = saved_last_reset_request_ns;
    fallback_mode = saved_fallback_mode;
    fallback_decoder_reset = saved_fallback_decoder_reset;
    control_fd = saved_control_fd;
    if (control_pair[0] >= 0) {
        close(control_pair[0]);
        close(control_pair[1]);
    }
    if (failure) {
        fprintf(stderr, "fallback decode resync self-test failed: %s\n",
                failure);
        return 1;
    }
    return 0;
}

static int expect_control(int fd, const uint8_t *expected, size_t size) {
    uint8_t message[64];
    return size <= sizeof(message)
        && recv(fd, message, size, MSG_DONTWAIT) == (ssize_t) size
        && !memcmp(message, expected, size);
}

static int expect_key_press(int fd, int keycode, uint32_t meta) {
    uint8_t down[14];
    uint8_t up[14];
    encode_keycode(down, 0, keycode, meta);
    encode_keycode(up, 1, keycode, meta);
    return expect_control(fd, down, sizeof(down))
        && expect_control(fd, up, sizeof(up));
}

static int control_idle(int fd) {
    uint8_t byte;
    return recv(fd, &byte, 1, MSG_DONTWAIT) < 0;
}

static int input_self_test(void) {
    const uint32_t ctrl = ANDROID_META_CTRL_ON | ANDROID_META_CTRL_LEFT_ON;
    const uint32_t shift = ANDROID_META_SHIFT_ON | ANDROID_META_SHIFT_LEFT_ON;
    int saved_control_fd = control_fd;
    int control_pair[2];
    struct client_state state = {0};
    struct _rfbClientRec client;
    const char *failure = NULL;
    uint8_t message[14];

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, control_pair) < 0) {
        fprintf(stderr, "input self-test: no socket pair\n");
        return 1;
    }
    control_fd = control_pair[0];
    memset(&client, 0, sizeof(client));
    client.clientData = &state;
    int fd = control_pair[1];

    keyboard_event(TRUE, XK_Control_L, &client);
    keyboard_event(TRUE, XK_a, &client);
    keyboard_event(FALSE, XK_a, &client);
    keyboard_event(FALSE, XK_Control_L, &client);
    if (!expect_key_press(fd, ANDROID_KEYCODE_A, ctrl) || !control_idle(fd)) {
        failure = "Control+A is not one KEYCODE_A press with Control";
        goto done;
    }

    keyboard_event(TRUE, XK_Shift_L, &client);
    keyboard_event(TRUE, XK_Tab, &client);
    encode_keycode(message, 0, 61, shift);
    if (!expect_control(fd, message, sizeof(message))) {
        failure = "Shift+Tab lost its Shift meta state";
        goto done;
    }
    keyboard_event(FALSE, XK_Tab, &client);
    encode_keycode(message, 1, 61, shift);
    if (!expect_control(fd, message, sizeof(message))) {
        failure = "Shift+Tab release lost its Shift meta state";
        goto done;
    }
    keyboard_event(TRUE, XK_Insert, &client);
    keyboard_event(FALSE, XK_Insert, &client);
    if (!expect_key_press(fd, ANDROID_KEYCODE_V, ctrl)) {
        failure = "Shift+Insert is not Control+V";
        goto done;
    }
    keyboard_event(TRUE, XK_Delete, &client);
    keyboard_event(FALSE, XK_Delete, &client);
    keyboard_event(FALSE, XK_Shift_L, &client);
    if (!expect_key_press(fd, ANDROID_KEYCODE_X, ctrl)) {
        failure = "Shift+Delete is not Control+X";
        goto done;
    }
    keyboard_event(TRUE, XK_Control_R, &client);
    keyboard_event(TRUE, XK_Insert, &client);
    keyboard_event(FALSE, XK_Insert, &client);
    keyboard_event(FALSE, XK_Control_R, &client);
    if (!expect_key_press(fd, ANDROID_KEYCODE_C, ctrl) || !control_idle(fd)) {
        failure = "Control+Insert is not Control+C";
        goto done;
    }

    const struct {
        rfbKeySym key;
        int keycode;
    } keys[] = {{XK_End, 123}, {XK_Insert, 124}, {XK_F1, 131}, {XK_F12, 142}};
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
        keyboard_event(TRUE, keys[i].key, &client);
        keyboard_event(FALSE, keys[i].key, &client);
        encode_keycode(message, 0, keys[i].keycode, 0);
        if (!expect_control(fd, message, sizeof(message))) {
            failure = "End, Insert or an F-key has no Android keycode";
            goto done;
        }
        encode_keycode(message, 1, keys[i].keycode, 0);
        if (!expect_control(fd, message, sizeof(message))) {
            failure = "End, Insert or an F-key has no release";
            goto done;
        }
    }

    keyboard_event(TRUE, XK_Shift_L, &client);
    keyboard_event(TRUE, XK_B, &client);
    keyboard_event(FALSE, XK_B, &client);
    keyboard_event(FALSE, XK_Shift_L, &client);
    const uint8_t text_b[] = {SCRCPY_MSG_INJECT_TEXT, 0, 0, 0, 1, 'B'};
    if (!expect_control(fd, text_b, sizeof(text_b)) || !control_idle(fd)) {
        failure = "Shift+B is not the text B";
        goto done;
    }

    pointer_event(4, 10, 20, &client);
    pointer_event(0, 10, 20, &client);
    if (!expect_key_press(fd, ANDROID_KEYCODE_BACK, 0) || !control_idle(fd)) {
        failure = "the right button is not Back";
        goto done;
    }

done:
    control_fd = saved_control_fd;
    close(control_pair[0]);
    close(control_pair[1]);
    if (failure) {
        fprintf(stderr, "input self-test failed: %s\n", failure);
        return 1;
    }
    return 0;
}

static int clipboard_self_test(void) {
    const uint32_t ctrl = ANDROID_META_CTRL_ON | ANDROID_META_CTRL_LEFT_ON;
    int saved_control_fd = control_fd;
    int control_pair[2];
    int device_pair[2] = {-1, -1};
    struct client_state state = {0};
    struct _rfbClientRec client;
    const char *failure = NULL;
    uint8_t *big = NULL;

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, control_pair) < 0) {
        fprintf(stderr, "clipboard self-test: no socket pair\n");
        return 1;
    }
    control_fd = control_pair[0];
    memset(&client, 0, sizeof(client));
    client.clientData = &state;
    int fd = control_pair[1];

    char utf8[] = "\xe4\xbd\xa0\xe5\xa5\xbd abc\r\n";
    client.extClipboardRequested = TRUE;
    keyboard_event(TRUE, XK_Shift_L, &client);
    keyboard_event(TRUE, XK_Insert, &client);
    keyboard_event(FALSE, XK_Insert, &client);
    keyboard_event(FALSE, XK_Shift_L, &client);
    if (!control_idle(fd)) {
        failure = "a paste ran before the requested text arrived";
        goto done;
    }
    client.extClipboardRequested = FALSE;
    set_clipboard_utf8(utf8, sizeof(utf8), &client);
    const uint8_t set_utf8[] = {
        SCRCPY_MSG_SET_CLIPBOARD, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 11,
        0xe4, 0xbd, 0xa0, 0xe5, 0xa5, 0xbd, ' ', 'a', 'b', 'c', '\n',
    };
    if (!expect_control(fd, set_utf8, sizeof(set_utf8))
            || !expect_key_press(fd, ANDROID_KEYCODE_V, ctrl)
            || !control_idle(fd)) {
        failure = "the deferred paste did not follow its UTF-8 text";
        goto done;
    }

    client.extClipboardRequested = TRUE;
    for (int press = 0; press < 2; ++press) {
        keyboard_event(TRUE, XK_Control_L, &client);
        keyboard_event(TRUE, XK_v, &client);
        keyboard_event(FALSE, XK_v, &client);
        keyboard_event(FALSE, XK_Control_L, &client);
    }
    client.extClipboardRequested = FALSE;
    set_clipboard_utf8(utf8, sizeof(utf8), &client);
    if (!expect_control(fd, set_utf8, sizeof(set_utf8))
            || !expect_key_press(fd, ANDROID_KEYCODE_V, ctrl)
            || !expect_key_press(fd, ANDROID_KEYCODE_V, ctrl)
            || !control_idle(fd)) {
        failure = "two pastes deferred for one text did not both run";
        goto done;
    }


    char latin1[] = "caf\xe9";
    set_clipboard_latin1(latin1, 4, &client);
    const uint8_t set_latin1[] = {
        SCRCPY_MSG_SET_CLIPBOARD, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5,
        'c', 'a', 'f', 0xc3, 0xa9,
    };
    if (!expect_control(fd, set_latin1, sizeof(set_latin1))
            || !control_idle(fd)) {
        failure = "Latin-1 cut text did not arrive as UTF-8 without a paste";
        goto done;
    }
    if (!clipboard_echo("caf\xc3\xa9", 5) || clipboard_echo("x", 1)) {
        failure = "the echo of a client's clipboard was not recognized";
        goto done;
    }
    clipboard_set_ns -= CLIPBOARD_ECHO_NS;
    if (clipboard_echo("caf\xc3\xa9", 5)) {
        failure = "a later copy of a client's clipboard counted as its echo";
        goto done;
    }

    size_t big_length = SCRCPY_CLIPBOARD_TEXT_MAX + 3;
    big = malloc(SCRCPY_SET_CLIPBOARD_HEADER + 2 * big_length);
    if (!big) {
        failure = "no memory for the long clipboard";
        goto done;
    }
    uint8_t *long_text = big + SCRCPY_SET_CLIPBOARD_HEADER + big_length;
    for (size_t i = 0; i < big_length; ++i) {
        long_text[i] = "\xe4\xbd\xa0"[i % 3];
    }
    size_t size = encode_set_clipboard((const char *) long_text, big_length,
                                       big);
    size_t kept = size - SCRCPY_SET_CLIPBOARD_HEADER;
    if (kept > SCRCPY_CLIPBOARD_TEXT_MAX || kept % 3
            || AV_RB32(big + 10) != kept
            || kept + 3 <= SCRCPY_CLIPBOARD_TEXT_MAX) {
        failure = "a long clipboard was not cut on a UTF-8 boundary";
        goto done;
    }

    char decoded[16];
    const uint8_t mixed[] = {0xe4, 0xbd, 0xa0, ' ', 'a', 0xc3, 0xa9, 0xff, 0xe4};
    size_t decoded_length = utf8_to_latin1(mixed, sizeof(mixed), decoded);
    if (decoded_length != 6 || memcmp(decoded, "? a\xe9??", 6)) {
        failure = "UTF-8 to Latin-1 fallback is wrong";
        goto done;
    }

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, device_pair) < 0) {
        failure = "no device socket pair";
        goto done;
    }
    const uint8_t device[] = {
        SCRCPY_DEVICE_MSG_CLIPBOARD, 0, 0, 0, 5, 'h', 'e', 'l', 'l', 'o', 0x7f,
    };
    if (send_all(device_pair[0], device, sizeof(device)) < 0) {
        failure = "device messages were not written";
        goto done;
    }
    int statuses[2];
    char *text = NULL;
    uint32_t length = 0;
    for (int i = 0; i < 2; ++i) {
        uint8_t type;
        statuses[i] = recv_all(device_pair[1], &type, 1) > 0
                    ? read_device_message(device_pair[1], type, &text, &length)
                    : -2;
    }
    int read_hello = text && length == 5 && !strcmp(text, "hello");
    free(text);
    if (statuses[0] || statuses[1] != -1 || !read_hello) {
        failure = "device messages were misread";
        goto done;
    }

done:
    free(big);
    free(clipboard_text);
    clipboard_text = NULL;
    clipboard_length = 0;
    clipboard_set_ns = 0;
    control_fd = saved_control_fd;
    close(control_pair[0]);
    close(control_pair[1]);
    if (device_pair[0] >= 0) {
        close(device_pair[0]);
        close(device_pair[1]);
    }
    if (failure) {
        fprintf(stderr, "clipboard self-test failed: %s\n", failure);
        return 1;
    }
    return 0;
}

static int listen_loopback(uint16_t *port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr = {0};
    socklen_t length = sizeof(addr);
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (fd < 0 || bind(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0
            || listen(fd, 4) < 0
            || getsockname(fd, (struct sockaddr *) &addr, &length) < 0) {
        if (fd >= 0) {
            close(fd);
        }
        return -1;
    }
    *port = ntohs(addr.sin_port);
    return fd;
}

static int accept_within(int listener, int ms) {
    struct timeval timeout = {.tv_sec = ms / 1000, .tv_usec = (ms % 1000) * 1000};
    fd_set ready;
    FD_ZERO(&ready);
    FD_SET(listener, &ready);
    if (select(listener + 1, &ready, NULL, NULL, &timeout) <= 0) {
        return -1;
    }
    return accept(listener, NULL, NULL);
}

static int clipboard_session_self_test(void) {
    const char *failure = NULL;
    uint16_t port;
    int listener = listen_loopback(&port);
    int first = -1;
    int second = -1;
    pthread_t reader;
    int started = 0;
    if (listener < 0) {
        fprintf(stderr, "clipboard session self-test: no listener\n");
        return 1;
    }
    clipboard_host = "127.0.0.1";
    clipboard_port = port;
    clipboard_fd = connect_tcp(clipboard_host, clipboard_port);
    first = accept_within(listener, 1000);
    if (clipboard_fd < 0 || first < 0) {
        failure = "the first session did not connect";
        goto done;
    }
    started = !pthread_create(&reader, NULL, clipboard_reader_main, NULL);
    close(first);
    second = accept_within(listener, 3000);
    if (second < 0) {
        failure = "an ended session was not reconnected";
        goto done;
    }

done:
    running = 0;
    pthread_mutex_lock(&clipboard_fd_mutex);
    if (clipboard_fd >= 0) {
        shutdown(clipboard_fd, SHUT_RDWR);
    }
    pthread_mutex_unlock(&clipboard_fd_mutex);
    if (started) {
        pthread_join(reader, NULL);
    }
    if (clipboard_fd >= 0) {
        close(clipboard_fd);
    }
    clipboard_fd = -1;
    clipboard_host = NULL;
    clipboard_port = 0;
    running = 1;
    if (second >= 0) {
        close(second);
    }
    close(listener);
    if (failure) {
        fprintf(stderr, "clipboard session self-test failed: %s\n", failure);
        return 1;
    }
    return 0;
}

struct provided_text {
    char *text;
    int length;
};

static void record_provided(char *text, int length, rfbClientPtr client) {
    struct provided_text *provided = client->screen->screenData;
    free(provided->text);
    provided->text = malloc((size_t) length + 1);
    if (provided->text) {
        memcpy(provided->text, text, (size_t) length);
        provided->text[length] = 0;
    }
    provided->length = length;
}

/* Returns 1 when a provide of text declaring declared bytes reached the
 * text hook, 0 when it did not, -1 when the test connection failed. */
static int provide_reaches_hook(rfbScreenInfoPtr screen, const char *text,
                                uint32_t declared) {
    uint16_t port;
    int listener = listen_loopback(&port);
    int peer = listener >= 0 ? connect_tcp("127.0.0.1", port) : -1;
    int server = peer >= 0 ? accept_within(listener, 1000) : -1;
    int reached = -1;
    if (server >= 0) {
        rfbClientPtr cl = rfbNewClient(screen, server);
        if (cl) {
            cl->state = RFB_NORMAL;
            cl->enableExtendedClipboard = TRUE;
            uint8_t plain[64];
            size_t length = strlen(text);
            AV_WB32(plain, declared);
            memcpy(plain + 4, text, length);
            uint8_t packed[128];
            uLongf packed_length = sizeof(packed);
            if (compress(packed, &packed_length, plain, length + 4) == Z_OK) {
                uint8_t message[8 + 4 + sizeof(packed)] = {rfbClientCutText};
                AV_WB32(message + 4, (uint32_t) -(int32_t) (4 + packed_length));
                AV_WB32(message + 8, rfbExtendedClipboard_Provide
                                     | rfbExtendedClipboard_Text);
                memcpy(message + 12, packed, packed_length);
                struct provided_text *provided = screen->screenData;
                provided->length = -1;
                if (send_all(peer, message, 12 + packed_length) == 0) {
                    rfbProcessClientMessage(cl);
                    reached = provided->length >= 0;
                }
            }
            rfbCloseClient(cl);
            rfbClientConnectionGone(cl);
        } else {
            close(server);
        }
    }
    if (peer >= 0) {
        close(peer);
    }
    if (listener >= 0) {
        close(listener);
    }
    return reached;
}

static int short_provide_self_test(void) {
    int argc = 1;
    char *argv[] = {"scrcpy-rfb-self-test", NULL};
    rfbScreenInfoPtr screen = rfbGetScreen(&argc, argv, 16, 16, 8, 3, 4);
    struct provided_text provided = {NULL, -1};
    const char *failure = NULL;
    if (!screen) {
        fprintf(stderr, "short provide self-test: no screen\n");
        return 1;
    }
    screen->screenData = &provided;
    screen->setXCutTextUTF8 = record_provided;
    if (provide_reaches_hook(screen, "abc", 64) != 0) {
        failure = "a provide shorter than its declared size reached the hook";
    } else if (provide_reaches_hook(screen, "abc", 3) != 1
            || provided.length != 3 || !provided.text
            || strcmp(provided.text, "abc")) {
        failure = "a whole provide did not reach the hook with its text";
    }
    free(provided.text);
    screen->screenData = NULL;
    rfbScreenCleanup(screen);
    if (failure) {
        fprintf(stderr, "short provide self-test failed: %s\n", failure);
        return 1;
    }
    return 0;
}

static int run_self_test(void) {
    const int width = 320;
    const int height = 480;
    const int expected_dy = -24;
    if (h264_live_edge_self_test()) {
        return 1;
    }
    if (fallback_decode_resync_self_test()) {
        return 1;
    }
    if (input_self_test() || clipboard_self_test()
            || clipboard_session_self_test() || short_provide_self_test()) {
        return 1;
    }
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

    uint8_t display_power[2];
    encode_display_power(display_power, 0);
    if (display_power[0] != SCRCPY_MSG_SET_DISPLAY_POWER
            || display_power[1] != 0) {
        fprintf(stderr, "display power off message self-test failed\n");
        return 1;
    }
    encode_display_power(display_power, 1);
    if (display_power[0] != SCRCPY_MSG_SET_DISPLAY_POWER
            || display_power[1] != 1) {
        fprintf(stderr, "display power on message self-test failed\n");
        return 1;
    }

    char arg0[] = "scrcpy-rfb";
    char arg1[] = "--turn-screen-off";
    char arg2[] = "--scrcpy-host";
    char arg3[] = "127.0.0.2";
    char arg4[] = "--scrcpy-port";
    char arg5[] = "12345";
    char arg6[] = "-rfbport";
    char arg7[] = "5901";
    char arg8[] = "--clipboard-port";
    char arg9[] = "27184";
    char *test_argv[] = {
        arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, NULL,
    };
    int test_argc = (int) (sizeof(test_argv) / sizeof(test_argv[0])) - 1;
    struct bridge_options test_options;
    if (parse_bridge_options(&test_argc, test_argv, &test_options) < 0
            || !test_options.turn_screen_off
            || strcmp(test_options.scrcpy_host, "127.0.0.2") != 0
            || test_options.scrcpy_port != 12345
            || test_options.clipboard_port != 27184
            || test_argc != 3
            || strcmp(test_argv[1], "-rfbport") != 0
            || strcmp(test_argv[2], "5901") != 0
            || test_argv[3] != NULL) {
        fprintf(stderr, "bridge option parsing self-test failed\n");
        return 1;
    }

    fprintf(stderr, "scrcpy-rfb self-test passed\n");
    return 0;
}

/* The Open H.264 patch assigns preferredEncoding without going through
 * libvncserver's own "Using %s encoding" log, so a client that negotiated
 * passthrough leaves no trace of it. Say so once per client. */
static void log_client_mode_once(rfbClientPtr client,
                                 struct client_state *state) {
    if (state->mode_logged) {
        return;
    }
    state->mode_logged = 1;
    fprintf(stderr, "VNC client %s serving %s\n", client->host,
            is_h264_encoding(client->preferredEncoding)
                ? "H.264 passthrough"
                : client->supportsH264Encoding
                    ? "ordinary (H.264-capable, chose another encoding)"
                    : "ordinary");
}

static void adapt_client_jpeg(rfbClientPtr client,
                              struct client_state *state) {
    if (client->preferredEncoding != rfbEncodingTight) {
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
 * decoder owns a single pending frame on purpose: when VNC output is slower
 * than Android, the newest frame replaces the pending one instead of stale
 * decoded frames queueing up. */
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
     * decoder thread can keep replacing the pending frame, and H.264
     * passthrough stays independent of ordinary-client backpressure. */
    pthread_rwlock_wrlock(&screen_buffer_lock);
    pthread_mutex_lock(&fallback_mutex);
    if (!fallback_frame_ready) {
        pthread_mutex_unlock(&fallback_mutex);
        pthread_rwlock_unlock(&screen_buffer_lock);
        return 0;
    }
    av_frame_move_ref(publish_frame, pending_frame);
    fallback_frame_ready = 0;
    pthread_mutex_unlock(&fallback_mutex);

    /* YUV to RGBA conversion runs at the publish cadence, not the decode
     * cadence: frames superseded while a publish was pending are never
     * converted at all. fallback_buffer is only ever written here. */
    sws_context = sws_getCachedContext(
            sws_context,
            publish_frame->width, publish_frame->height,
            (enum AVPixelFormat) publish_frame->format,
            video_width, video_height, AV_PIX_FMT_RGBA,
            SWS_FAST_BILINEAR, NULL, NULL, NULL);
    if (!sws_context) {
        av_frame_unref(publish_frame);
        pthread_rwlock_unlock(&screen_buffer_lock);
        return 0;
    }
    uint8_t *destinations[] = {fallback_buffer, NULL, NULL, NULL};
    int strides[] = {video_width * 4, 0, 0, 0};
    sws_scale(sws_context,
              (const uint8_t *const *) publish_frame->data,
              publish_frame->linesize, 0, publish_frame->height,
              destinations, strides);
    av_frame_unref(publish_frame);
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

    pthread_mutex_lock(&screen_ready_mutex);
    screen_frame_ready = 1;
    pthread_cond_broadcast(&screen_buffer_cond);
    pthread_mutex_unlock(&screen_ready_mutex);
    pthread_rwlock_unlock(&screen_buffer_lock);
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

    /* Do not advertise a resizeable desktop: the scrcpy session size is
     * fixed and several viewers terminate when their automatic
     * SetDesktopSize request is rejected. H.264 clients need this as much as
     * ordinary ones - they advertise ExtDesktopSize too. */
    client->useExtDesktopSize = FALSE;
    client->useNewFBSize = FALSE;
    client->newFBSizePending = FALSE;

    log_client_mode_once(client, state);
    if (is_h264_encoding(client->preferredEncoding)) {
        return;
    }
    adapt_client_jpeg(client, state);

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
    state->ordinary_update_started_ns = monotonic_ns();
    pthread_mutex_unlock(&metrics_mutex);
    state->framebuffer_lock_held = 1;
}

static void display_finished_hook(rfbClientPtr client, int result) {
    (void) result;
    struct client_state *state = client->clientData;
    if (state->framebuffer_lock_held) {
        uint64_t finished_ns = monotonic_ns();
        pthread_mutex_lock(&metrics_mutex);
        uint64_t sample = (finished_ns - state->ordinary_update_started_ns)
                        / 1000;
        if (!state->ordinary_update_us_ema) {
            state->ordinary_update_us_ema = sample;
        } else {
            state->ordinary_update_us_ema =
                    (state->ordinary_update_us_ema * 3 + sample) / 4;
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
            pthread_mutex_lock(&metrics_mutex);
            uint64_t update_us = state->ordinary_update_us_ema;
            pthread_mutex_unlock(&metrics_mutex);
            if (update_us > *slowest_update_us) {
                *slowest_update_us = update_us;
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
            "  --scrcpy-host addr   scrcpy server IPv4 address (default %s)\n"
            "  --scrcpy-port port   scrcpy server TCP port (default %d)\n"
            "  --clipboard-port port\n"
            "                       control-only scrcpy session that reads the\n"
            "                       Android clipboard (one running as shell)\n"
            "  --turn-screen-off    turn the physical display off while running\n"
            "  --self-test          run built-in self tests and exit\n"
            "  --help               show this help\n"
            "\n"
            "libvncserver options (-listen, -rfbport, -passwd, -rfbauth, ...):\n",
            program, SCRCPY_DEFAULT_HOST, SCRCPY_DEFAULT_PORT);
    rfbUsage();
}

int main(int argc, char **argv) {
    struct bridge_options options;
    if (parse_bridge_options(&argc, argv, &options) < 0) {
        return 1;
    }
    if (options.self_test) {
        return run_self_test();
    }
    if (options.help) {
        usage(argv[0]);
        return 0;
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    signal(SIGPIPE, SIG_IGN);

    pthread_condattr_t monotonic_attr;
    pthread_condattr_init(&monotonic_attr);
    pthread_condattr_setclock(&monotonic_attr, CLOCK_MONOTONIC);
    pthread_cond_init(&screen_buffer_cond, &monotonic_attr);
    pthread_condattr_destroy(&monotonic_attr);

    video_fd = connect_tcp(options.scrcpy_host, options.scrcpy_port);
    if (video_fd < 0) {
        fprintf(stderr, "failed to connect scrcpy video socket\n");
        return 1;
    }
    control_fd = connect_tcp(options.scrcpy_host, options.scrcpy_port);
    if (control_fd < 0) {
        fprintf(stderr, "failed to connect scrcpy control socket\n");
        return 1;
    }
    if (options.clipboard_port) {
        clipboard_host = options.scrcpy_host;
        clipboard_port = options.clipboard_port;
        clipboard_fd = connect_tcp(options.scrcpy_host, options.clipboard_port);
        if (clipboard_fd < 0) {
            fprintf(stderr, "failed to connect scrcpy clipboard socket\n");
            return 1;
        }
    }

    uint8_t codec_header[4];
    uint8_t session_header[12];
    if (recv_all(video_fd, codec_header, sizeof(codec_header)) <= 0
            || AV_RB32(codec_header) != SCRCPY_CODEC_H264) {
        fprintf(stderr, "scrcpy did not negotiate H.264\n");
        return 1;
    }
    if (recv_all(video_fd, session_header, sizeof(session_header)) <= 0
            || !(AV_RB64(session_header) & SCRCPY_PACKET_FLAG_SESSION)) {
        fprintf(stderr, "missing initial scrcpy session metadata\n");
        return 1;
    }
    video_width = (int) AV_RB32(session_header + 4);
    video_height = (int) AV_RB32(session_header + 8);
    fprintf(stderr, "scrcpy H.264 stream: %dx%d\n", video_width, video_height);

    framebuffer_size = (size_t) video_width * (size_t) video_height * 4;
    fallback_buffer = calloc(1, framebuffer_size);
    if (!fallback_buffer || init_fallback_decoder() < 0) {
        free(fallback_buffer);
        return 1;
    }

    pthread_t video_thread;
    pthread_t control_thread;
    pthread_t clipboard_thread;
    pthread_t decoder_thread;
    pthread_create(&video_thread, NULL, video_reader_main, NULL);
    pthread_create(&control_thread, NULL, control_reader_main,
                   (void *) (intptr_t) control_fd);
    int clipboard_session = clipboard_fd >= 0;
    if (clipboard_session) {
        pthread_create(&clipboard_thread, NULL, clipboard_reader_main, NULL);
    }
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
    rfb_screen->setXCutText = set_clipboard_latin1;
    rfb_screen->setXCutTextUTF8 = set_clipboard_utf8;
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

    int restore_display_power = 0;
    if (options.turn_screen_off && running) {
        if (send_display_power(0) < 0) {
            fprintf(stderr, "failed to turn the Android display off\n");
        } else {
            restore_display_power = 1;
            fprintf(stderr,
                    "Android display power-off requested while mirroring\n");
        }
    }

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
                av_frame_unref(pending_frame);
                pthread_mutex_unlock(&fallback_mutex);
                pthread_mutex_lock(&decode_mutex);
                decode_skip_until_key = 0;
                decode_keyframe_wait_started_ns = 0;
                decode_backlog_drop_count = 0;
                pthread_mutex_unlock(&decode_mutex);
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

        /* Without ordinary clients there is no publish to pace; a coarser
         * tick only delays fallback enable by up to 50 ms. */
        struct timespec delay = {.tv_sec = 0,
                                 .tv_nsec = standard_clients > 0
                                     ? 2 * 1000 * 1000
                                     : 50 * 1000 * 1000};
        nanosleep(&delay, NULL);
    }

    if (restore_display_power) {
        if (send_display_power(1) < 0) {
            fprintf(stderr, "failed to restore the Android display power\n");
        } else {
            fprintf(stderr, "Android display power-on restore requested\n");
        }
    }
    running = 0;
    rfbShutdownServer(rfb_screen, TRUE);
    shutdown(video_fd, SHUT_RDWR);
    shutdown(control_fd, SHUT_RDWR);
    pthread_mutex_lock(&clipboard_fd_mutex);
    if (clipboard_fd >= 0) {
        shutdown(clipboard_fd, SHUT_RDWR);
    }
    pthread_mutex_unlock(&clipboard_fd_mutex);
    pthread_mutex_lock(&decode_mutex);
    pthread_cond_broadcast(&decode_cond);
    pthread_mutex_unlock(&decode_mutex);
    pthread_mutex_lock(&screen_ready_mutex);
    pthread_cond_broadcast(&screen_buffer_cond);
    pthread_mutex_unlock(&screen_ready_mutex);
    pthread_join(video_thread, NULL);
    pthread_join(control_thread, NULL);
    if (clipboard_session) {
        pthread_join(clipboard_thread, NULL);
    }
    if (clipboard_fd >= 0) {
        close(clipboard_fd);
    }
    pthread_join(decoder_thread, NULL);

    close(video_fd);
    close(control_fd);
    free(codec_config);
    sws_freeContext(sws_context);
    av_frame_free(&decoder_frame);
    av_frame_free(&pending_frame);
    av_frame_free(&publish_frame);
    avcodec_free_context(&decoder_context);
    free(fallback_buffer);
    free(damage_rects);
    free(scroll_old_rows);
    free(scroll_new_rows);
    free(clipboard_text);
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
