FROM tiryoh/ros2-desktop-vnc:humble
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-image-view && \
    apt-get clean && \
    rm -r /var/lib/apt/lists/*
RUN mkdir -p ~/colcon_ws/src && \
    /bin/bash -c "source /opt/ros/humble/setup.bash; cd ~/colcon_ws/; colcon build" && \
    echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
RUN cd ~/colcon_ws/src && \
    git clone -b humble-devel https://github.com/Alpaca-zip/ultralytics_ros.git && \
    python3 -m pip install -r ultralytics_ros/requirements.txt && \
    cd ~/colcon_ws && \
    colcon build
RUN cd ~/ && \
    wget --load-cookies /tmp/cookies.txt "https://drive.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://drive.google.com/uc?export=download&id=1kN8boJxfMIl7hjytxdgmzANZBhWV7YXU' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1kN8boJxfMIl7hjytxdgmzANZBhWV7YXU" -O kitti_2011_09_26_drive_0106_synced.zip && \
    rm -rf /tmp/cookies.txt && \
    unzip kitti_2011_09_26_drive_0106_synced.zip && \
    rm kitti_2011_09_26_drive_0106_synced.zip
