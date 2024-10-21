FROM stereolabs/zed:4.1-devel-jetson-jp5.1.2
# Set the working directory
WORKDIR /app/src
SHELL ["/bin/bash", "-c"]
RUN apt update -y
# Install dependencies and ROS Noetic
RUN apt-get update && apt-get install -y \
python3-pip \
python3-opencv \
wget \
lsb-release \
gnupg2 \
&& rm -rf /var/lib/apt/lists/*

RUN pip install pyyaml
RUN pip install witmotion

# Setup sources.list for ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Noetic Desktop Full
RUN apt-get update && apt-get install -y ros-noetic-desktop-full

# Install ROS dependencies for Python
RUN pip3 install rospkg catkin_pkg pypcd open3d rosnumpy

# Install ZED ROS Wrapper (for communicating ZED with ROS)
RUN apt-get update && apt-get install -y \
    ros-noetic-rviz \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-image-transport \
    ros-noetic-diagnostic-updater \
    ros-noetic-message-filters

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /app/devel/setup.bash" >> ~/.bashrc
RUN apt-get install -y ros-noetic-rosserial libsdl-image1.2-dev ros-noetic-move-base libqt5serialport5-dev libpcap-dev python3-catkin-tools ros-noetic-rtcm-msgs ros-noetic-nmea-msgs ros-noetic-amcl
