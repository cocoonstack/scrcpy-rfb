FROM ubuntu:22.04

ARG FFMPEG_COMMIT=998de74adf861c26df557e220996faa959419549
# ports.ubuntu.com (arm64) flakes regularly; CI overrides this with a mirror.
ARG UBUNTU_PORTS_MIRROR=http://ports.ubuntu.com/ubuntu-ports
ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i "s|http://ports.ubuntu.com/ubuntu-ports|${UBUNTU_PORTS_MIRROR}|g" /etc/apt/sources.list \
    && echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       build-essential ca-certificates cmake curl file git nasm pkg-config \
       libjpeg-turbo8-dev zlib1g-dev \
    && rm -rf /var/lib/apt/lists/* \
    && curl -fsSL \
       "https://github.com/FFmpeg/FFmpeg/archive/${FFMPEG_COMMIT}.tar.gz" \
       | tar xz -C /opt \
    && mv "/opt/FFmpeg-${FFMPEG_COMMIT}" /opt/ffmpeg

WORKDIR /src
ENTRYPOINT ["/src/build-linux.sh"]
