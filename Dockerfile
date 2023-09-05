FROM tiryoh/ros-desktop-vnc:melodic
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
    python3-pip \
    ros-melodic-ros-numpy \
    ros-melodic-vision-msgs && \
    apt-get clean && \
    rm -r /var/lib/apt/lists/*
RUN mkdir -p ~/catkin_ws/src && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash ; cd ~/catkin_ws/src ; catkin_init_workspace" && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash ; cd ~/catkin_ws && catkin build" && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN cd ~/catkin_ws/src && \
    git clone -b melodic-devel https://github.com/Alpaca-zip/ultralytics_ros.git && \
    pip install pipenv && \
    cd ultralytics_ros && \
    pipenv install && \
    pipenv shell && \
    cd ~/catkin_ws && \
    catkin build
RUN cd ~/ && \
    wget --load-cookies /tmp/cookies.txt "https://drive.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://drive.google.com/uc?export=download&id=1FWW3yHq1ZVps5gtm0VGrSmEAX0gGRM2t' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1FWW3yHq1ZVps5gtm0VGrSmEAX0gGRM2t" -O kitti_2011_09_26_drive_0106_synced.bag && \
    rm -rf /tmp/cookies.txt
