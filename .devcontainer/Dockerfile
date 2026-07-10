FROM ubuntu:22.04




RUN apt-get update && apt-get install -y \
    gcc-arm-none-eabi \
    gdb-multiarch \
    build-essential \
    cmake \
    git \
    openocd \
    usbutils \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /workspace