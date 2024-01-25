# Use tiryoh/ros2-desktop-vnc:humble as the base image
FROM tiryoh/ros2-desktop-vnc:humble-20230611T1926

# Set Environment Variables
ENV DEBIAN_FRONTEND noninteractive

# Install required packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
        python3-pip \
        ros-humble-cv-bridge \
        ros-humble-vision-msgs \
        ros-humble-image-view \
        ros-humble-pcl-* && \
    apt-get clean && \
    rm -r /var/lib/apt/lists/*

# Initialize colcon workspace
RUN mkdir -p ~/colcon_ws/src && \
    /bin/bash -c "source /opt/ros/humble/setup.bash; cd ~/colcon_ws/; colcon build" && \
    echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc

# Install Git LFS
RUN apt-get update && \
    apt-get install -y git-lfs && \
    rm -rf /var/lib/apt/lists/* && \
    git lfs install

# Clone repository and install using requirements.txt
RUN cd ~/colcon_ws/src && \
    git clone -b humble-devel https://github.com/Alpaca-zip/ultralytics_ros.git && \
    cd ultralytics_ros && \
    python3 -m pip install -r requirements.txt

# Build the ROS2 package
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; cd ~/colcon_ws/; colcon build"
