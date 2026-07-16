#!/usr/bin/env bash
# Build a Linux scrcpy-to-RFB server. FFmpeg is static; libjpeg/zlib/glibc are
# intentionally linked from the Ubuntu 22.04 ABI baseline.
set -euo pipefail

case "$(dpkg --print-architecture)" in
  amd64) artifact=scrcpy-rfb-linux-amd64 ;;
  arm64) artifact=scrcpy-rfb-linux-arm64 ;;
  *) echo "unsupported architecture: $(dpkg --print-architecture)" >&2; exit 1 ;;
esac

rm -rf /tmp/ffmpeg-build /tmp/ffmpeg-install /tmp/scrcpy-rfb-build
mkdir -p /tmp/ffmpeg-build /src/dist

cd /tmp/ffmpeg-build
/opt/ffmpeg/configure \
  --prefix=/tmp/ffmpeg-install \
  --disable-everything \
  --disable-autodetect \
  --disable-doc \
  --disable-network \
  --disable-programs \
  --disable-shared \
  --disable-avdevice \
  --disable-avfilter \
  --disable-avformat \
  --disable-swresample \
  --enable-static \
  --enable-pic \
  --enable-avcodec \
  --enable-avutil \
  --enable-swscale \
  --enable-decoder=h264 \
  --enable-parser=h264
make -j"$(nproc)"
make install

cmake \
  -S /src \
  -B /tmp/scrcpy-rfb-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DFFMPEG_ROOT=/tmp/ffmpeg-install
cmake --build /tmp/scrcpy-rfb-build --parallel

cp /tmp/scrcpy-rfb-build/scrcpy-rfb "/src/dist/${artifact}"
"/src/dist/${artifact}" --self-test
strip "/src/dist/${artifact}"
file "/src/dist/${artifact}"
(
  cd /src/dist
  sha256sum "${artifact}" > "${artifact}.sha256"
)
