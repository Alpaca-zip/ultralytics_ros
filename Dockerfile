# Use tiryoh/ros-desktop-vnc:noetic as the base image
FROM tiryoh/ros-desktop-vnc:noetic

# Set Environment Variables
ENV DEBIAN_FRONTEND noninteractive

# Install required packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
        python3-pip \
        ros-noetic-vision-msgs && \
    apt-get clean && \
    rm -r /var/lib/apt/lists/*

# Initialize catkin workspace
RUN mkdir -p ~/catkin_ws/src && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash ; cd ~/catkin_ws/src ; catkin_init_workspace" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash ; cd ~/catkin_ws && catkin build" && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Install Git LFS
RUN apt-get update && \
    apt-get install -y git-lfs && \
    rm -rf /var/lib/apt/lists/* && \
    git lfs install

# Clone repository and install using requirements.txt
RUN cd ~/catkin_ws/src && \
    git clone -b noetic-devel https://github.com/Alpaca-zip/ultralytics_ros.git && \
    python3 -m pip install -r ultralytics_ros/requirements.txt

# Build the ROS package
RUN cd ~/catkin_ws && catkin build
