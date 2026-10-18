FROM ros:lyrical-ros-base

ENV DEBIAN_FRONTEND=noninteractive

ARG ZEPHYR_SDK_VERSION=1.0.1

# Install system dependencies for both Zephyr and ROS 2
RUN apt-get update && apt-get install -y \
    python3-pip python3-venv python3-setuptools \
    python3-colcon-common-extensions \
    cmake ninja-build gperf build-essential \
    ccache dfu-util device-tree-compiler wget \
    python3-dev python3-tk python3-wheel xz-utils file \
    make gcc libsdl2-dev libmagic1 git \
    libudev-dev libusb-1.0-0-dev pkg-config \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    ros-${ROS_DISTRO}-rosidl-generator-c \
    ros-${ROS_DISTRO}-rosidl-generator-cpp \
    && rm -rf /var/lib/apt/lists/*

# Install Python workspace dependencies
RUN --mount=type=bind,source=requirements.txt,target=/tmp/requirements.txt \
    pip3 install --ignore-installed --break-system-packages -r /tmp/requirements.txt

# Set up Zephyr workspace with zenoh-pico preloaded
WORKDIR /zephyr_ws
COPY docker/image-west.yml        manifest-repo/west.yml
COPY zenbedded_transport/west.yml manifest-repo/zenbedded_transport.yml
RUN cd manifest-repo && git init -q && git add . \
    && git -c user.email=ci@local -c user.name=ci commit -qm "image manifest"
RUN west init -l manifest-repo && west update && west zephyr-export

# Install Zephyr SDK(MINIMAL)
RUN ARCH=$(uname -m) && \
    if [ "$ARCH" = "aarch64" ]; then SDK_ARCH="aarch64"; else SDK_ARCH="x86_64"; fi && \
    wget -q https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZEPHYR_SDK_VERSION}/zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-${SDK_ARCH}_minimal.tar.xz \
    && tar xf zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-${SDK_ARCH}_minimal.tar.xz \
    && rm zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-${SDK_ARCH}_minimal.tar.xz \
    && cd zephyr-sdk-${ZEPHYR_SDK_VERSION} \
    && ./setup.sh -t xtensa-espressif_esp32_zephyr-elf -t xtensa-espressif_esp32s3_zephyr-elf -t arm-zephyr-eabi -c -h

# Install all python dependencies for all subpackages
RUN --mount=type=bind,source=requirements.txt,target=/tmp/requirements.txt \
    PIP_CONSTRAINT=/tmp/requirements.txt \
    west packages pip --install --ignore-venv-check -- --break-system-packages --ignore-installed
RUN west blobs fetch hal_espressif

# Set environment variables
ENV ZEPHYR_BASE=/zephyr_ws/zephyr
ENV ZEPHYR_SDK_INSTALL_DIR=/zephyr_ws/zephyr-sdk-${ZEPHYR_SDK_VERSION}
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Set up ROS 2 workspace structure
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws


RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /zephyr_ws/zephyr/zephyr-env.sh" >> /root/.bashrc

RUN usermod -a -G dialout root

CMD ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && exec /bin/bash"]
