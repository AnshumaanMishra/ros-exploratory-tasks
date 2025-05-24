FROM osrf/ros:humble-desktop-full

# Setup user for GUI to work without xhost
ARG UNAME=user
ARG UID=1000
ARG GID=1000

RUN groupadd -g ${GID} -o ${UNAME} && \
    useradd -m -u ${UID} -g ${GID} -o -s /bin/bash ${UNAME} && \
    usermod -aG sudo ${UNAME} && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER ${UNAME}
ENV HOME=/home/${UNAME}
WORKDIR $HOME

# Create a custom ros2 overlay workspace and workspace for exercises
ENV ROS2_WS=$HOME/ros2_ws
ENV EXERCISES_WS=$HOME/exercises_ws
ENV TURTLEBOT3_WS=$HOME/turtlebot3_ws

RUN mkdir -p $ROS2_WS/src $EXERCISES_WS/src $TURTLEBOT3_WS/src

# Disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive

# Make default shell in Dockerfile bash instead of sh
SHELL ["/bin/bash", "-c"]

# Copy andino requirement files and install dependencies
COPY docker/andino_requirements.txt .

RUN sudo apt-get update &&\ 
    sudo apt-get install --no-install-recommends -y $(cat andino_requirements.txt)

RUN sudo rm andino_requirements.txt

# Install Gazebo Fortress
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get update
RUN sudo apt-get install ignition-fortress -y

# Install dependencies
RUN sudo apt-get update && \
    sudo apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-turtle-nest \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-gazebo-* \
    ros-$ROS_DISTRO-cartographer \
    ros-$ROS_DISTRO-cartographer-ros \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-ament-cmake \
    ros-$ROS_DISTRO-desktop

RUN sudo apt-get update && sudo apt-get install -y \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-core \
    ros-${ROS_DISTRO}-ament-cmake-auto \
    ros-${ROS_DISTRO}-ament-cmake-python \
    ros-${ROS_DISTRO}-ament-cmake-export-interfaces \
    python3-colcon-common-extensions

# NVIDIA
RUN sudo apt-get update && \
    sudo apt-get install -y --no-install-recommends \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libxext6 \
    libxrender1 \
    libxtst6 \
    libxi6 \
    libnvidia-gl-570 \
    vim

RUN sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# Install andino_gz and other packages with their dependencies
COPY ./packages/ ${ROS2_WS}/src
RUN cd ${ROS2_WS}/src && \
    git clone https://github.com/Ekumen-OS/andino_gz.git -b 0.1.1 && \
    sudo apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd .. && \
    rosdep update && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* && \
    colcon build --symlink-install

# Build exercises_ws
RUN cd ${EXERCISES_WS}/src && \
    sudo apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd .. && \
    rosdep update && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* && \
    colcon build --symlink-install

# Source ROS workspace automatically when new terminal is opened
RUN sudo echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    sudo echo ". ${EXERCISES_WS}/install/setup.bash" >> ~/.bashrc && \
    sudo echo ". ${ROS2_WS}/install/setup.bash" >> ~/.bashrc

# RUN sudo apt update && sudo apt -y upgrade

# Install TutleBot3

RUN git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git $TURTLEBOT3_WS/src/DynamixelSDK && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git $TURTLEBOT3_WS/src/turtlebot3_msgs && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git $TURTLEBOT3_WS/src/turtlebot3 && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git $TURTLEBOT3_WS/src/turtlebot3_simulations &&\
    git clone -b humble https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git $TURTLEBOT3_WS/src/hls_lfcd_lds_driver
    
RUN source /opt/ros/humble/setup.bash && \
    cd ~/turtlebot3_ws &&\
    # Added Line
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble && \
    colcon build --symlink-install &&\
    echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc &&\
    source ~/.bashrc   

RUN echo "source $TURTLEBOT3_WS/install/setup.bash" >> $HOME/.bashrc && \
    echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc && \
    echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc


RUN source /opt/ros/${ROS_DISTRO}/setup.bash &&\
    source ${ROS2_WS}/install/setup.bash &&\
    source ${EXERCISES_WS}/install/setup.bash &&\
    source ~/.bashrc


RUN cd ~/turtlebot3_ws/src/ &&\
    cd ~/turtlebot3_ws && colcon build --symlink-install

WORKDIR /home/user/exercises_ws

# Source ROS in the main terminal
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh

# Source ROS in the main terminal
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
