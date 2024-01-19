# Use tiryoh/ros-desktop-vnc:melodic as the base image
FROM tiryoh/ros-desktop-vnc:melodic

# Set Environment Variables
ENV LC_ALL=C.UTF-8 \
    LANG=C.UTF-8 \
    DEBIAN_FRONTEND=noninteractive

# Install required packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
        python3-pip \
        python3.8 \
        ros-melodic-ros-numpy \
        ros-melodic-vision-msgs && \
    apt-get clean && \
    rm -r /var/lib/apt/lists/*

# Initialize catkin workspace
RUN mkdir -p ~/catkin_ws/src && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash ; cd ~/catkin_ws/src ; catkin_init_workspace" && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash ; cd ~/catkin_ws && catkin build" && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Install Git LFS
RUN apt-get update && \
    apt-get install -y git-lfs && \
    rm -rf /var/lib/apt/lists/* && \
    git lfs install

# Clone repository and install using pipenv
RUN cd ~/catkin_ws/src && \
    git clone -b melodic-devel https://github.com/Alpaca-zip/ultralytics_ros.git && \
    python3 -m pip install pipenv==2021.5.29 && \
    cd ultralytics_ros && \
    pipenv install

# Build the ROS package
RUN cd ~/catkin_ws && catkin build
