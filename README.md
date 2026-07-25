# scrcpy-rfb

`scrcpy-rfb` bridges the packetized H.264 video and control sockets of a
scrcpy 4.x server to one RFB/VNC port.

- Clients that negotiate the Open H.264 encoding (50) receive the original
  Android H.264 access units without server-side transcoding. TigerVNC and
  noVNC decode this encoding natively.
- Other VNC clients automatically receive Tight/JPEG, ZRLE, Hextile or Raw
  from a lazily decoded framebuffer on the same port.

## Building

The project depends on upstream [LibVNC/libvncserver](https://github.com/LibVNC/libvncserver)
at a pinned commit, fetched at configure time, plus the two patches in
`patches/` (the Open H.264 server-side encoding and a `deferUpdateTime=0`
busy-loop fix). The patches have been submitted upstream; once merged, the
directory empties and the pin moves to a release tag.

Release binaries (Ubuntu 22.04 ABI, static FFmpeg H.264 decoder):

```sh
docker build -t scrcpy-rfb-builder -f docker/Dockerfile.builder .
docker run --rm -v "$PWD:/src" scrcpy-rfb-builder
# -> dist/scrcpy-rfb-linux-<arch>
```

Local build (needs cmake, git, libjpeg-turbo, zlib and FFmpeg dev packages):

```sh
make        # cmake configure + build into build/
make test   # build + run the self tests
```

Pushes to `master` publish, via GitHub Actions:

- `scrcpy-rfb-linux-amd64` / `-arm64` binaries, checksums and
  `build-info.json` to the moving `master` prerelease;
- a multi-arch container image at `ghcr.io/cocoonstack/scrcpy-rfb:master`
  (also tagged with the commit SHA):

```sh
docker run --rm --network host ghcr.io/cocoonstack/scrcpy-rfb:master \
  --scrcpy-host 127.0.0.1 -rfbport 5900
```

## Running

Start a scrcpy 4.x server with H.264 video and a TCP listener, then:

```sh
scrcpy-rfb [--scrcpy-host 127.0.0.1] [--scrcpy-port 27183] \
           [--turn-screen-off] \
           [libvncserver options]
```

`--turn-screen-off` asks the scrcpy server to power off the physical display
after the video, control, and RFB paths are ready. Mirroring continues through
the virtual display. On every normal exit and handled `SIGINT`/`SIGTERM`, the
bridge sends a best-effort display-on message before closing the control
socket. A forced kill or a broken control connection cannot guarantee restore.

All standard libvncserver options are accepted, notably:

```
-rfbport port       RFB listen port (default 5900)
-listen ipaddr      bind to one interface, e.g. -listen 127.0.0.1
-passwd password    require a VNC password (plain, use at your own risk)
-rfbauth file       require a VNC password from a vncpasswd file
```

The bridge runs unauthenticated on all IPv4 interfaces by default; anyone who
can reach the port controls the Android device. In production either bind to
localhost and tunnel, or set `-passwd`/`-rfbauth`.

## Client notes

For an H.264-enabled TigerVNC viewer, disable automatic encoding selection so
it does not override the explicit H.264 preference, and disable remote
resizing since the bridge exposes the fixed scrcpy session size:

```sh
vncviewer -AutoSelect=0 -PreferredEncoding=H.264 -RemoteResize=0 -Shared=1 \
  host::5900
```

Ordinary VNC clients work with their defaults. Apache Guacamole does not
include Tight in its default VNC encoding list; for the optimized ordinary
path configure the connection with:

```text
encodings: tight copyrect zrle hextile raw
compress-level: 1
quality-level: 8
disable-display-resize: true
```

## Behavior details

- When the first ordinary VNC client connects, the bridge asks scrcpy to
  reset the video encoder so a static Android screen still produces an
  immediate config packet and keyframe. An H.264 client that joins while the
  packet queue holds no keyframe triggers the same rate-limited reset, so it
  never waits for on-screen motion to start its stream.
- H.264 passthrough is push-based: a new access unit wakes only the H.264
  clients (no per-frame RFB request/response round trip), each client keeps
  its own frame cursor, and a slow client skips to a later keyframe without
  blocking others.
- The fallback decode (FFmpeg + swscale) runs on its own thread with a
  bounded queue; if decoding cannot keep up, the backlog is dropped and the
  decoder resynchronizes from a fresh keyframe instead of stalling the
  passthrough socket reads.
- Ordinary clients get change-detection at 32-pixel tile granularity, RFB
  CopyRect for recognized vertical scrolling, adaptive Tight/JPEG quality
  (Q92/Q86/Q80, never above the client's request) and an adaptive 60/30/20
  fps publish rate driven by the slowest client's measured update time. Each
  client socket is capped to a 256 KiB send buffer and, on Linux, a 64 KiB
  `TCP_NOTSENT_LOWAT`, so slow connections apply backpressure instead of
  queueing stale updates.
- Input: left button drags map to touch events with single-owner drag
  arbitration across clients, wheel buttons map to Android scroll injection,
  common keys map to Android keycodes and printable keysyms are injected as
  UTF-8 text.
- The scrcpy session size is fixed; device rotation or resolution change
  requires a restart.
