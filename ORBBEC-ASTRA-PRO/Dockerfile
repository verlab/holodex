FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    git \
    libusb-1.0-0-dev \
    libuvc-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libeigen3-dev \
    libopencv-dev \
    ros-humble-camera-info-manager \
    ros-humble-image-transport \
    ros-humble-image-publisher \
    ros-humble-image-geometry \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-sensor-msgs-py \
    ros-humble-tf2 \
    ros-humble-tf2-eigen \
    ros-humble-tf2-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-sensor-msgs \
    ros-humble-message-filters \
    ros-humble-rclcpp \
    ros-humble-rclcpp-components \
    ros-humble-rcutils \
    ros-humble-rosidl-default-generators \
    python3-colcon-common-extensions \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
