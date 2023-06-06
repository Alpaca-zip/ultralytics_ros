FROM tiryoh/ros-desktop-vnc:noetic
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
    python3-pip \
    ros-noetic-ros-numpy \
    ros-noetic-vision-msgs && \
    apt-get clean && \
    rm -r /var/lib/apt/lists/*
RUN mkdir -p ~/catkin_ws/src && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash ; cd ~/catkin_ws/src ; catkin_init_workspace" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash ; cd ~/catkin_ws && catkin build" && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN cd ~/catkin_ws/src && \
    git clone -b noetic-devel https://github.com/Alpaca-zip/ultralytics_ros.git && \
    python3 -m pip install -r ultralytics_ros/requirements.txt && \
    cd ~/catkin_ws && \
    catkin build
RUN cd ~/. && \
    python3 -m pip install --extra-index-url https://rospypi.github.io/simple/ kitti2bag \
    rosbag rospy tf tf2_ros tf2_msgs actionlib_msgs geometry_msgs sensor_msgs cv_bridge roslz4 && \
    wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip && \
    wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0106/2011_09_26_drive_0106_sync.zip && \
    unzip "2011_09_26_drive_0106_sync.zip" && \
    unzip "2011_09_26_calib.zip" && \
    kitti2bag -t 2011_09_26 -r 0106 raw_synced . && \
    rm -rf 2011_09_26*
