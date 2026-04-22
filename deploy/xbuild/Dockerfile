# Cross-compile environment for the Raspberry Pi target. Debian Bookworm
# matches Pi OS Bookworm 64-bit ABI (gstreamer 1.22 line), so libs linked
# here run unchanged on the Pi. Kept off the host because Ubuntu multiarch
# + conflicting amd64 dev packages is a known wedge.
#
# Build (from repo root):  docker build -t rover-xbuild deploy/xbuild/
# Build via make:          make xbuild
FROM debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive

RUN dpkg --add-architecture arm64 && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        git \
        make \
        pkg-config \
        g++-aarch64-linux-gnu \
        libgstrtspserver-1.0-dev:arm64 \
        libgstreamer1.0-dev:arm64 && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /src
