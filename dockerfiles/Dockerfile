FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

ENV DISPLAY host.docker.internal:0


#copy install scripts
COPY ./dockerfiles/dependencies.sh /root/install/

#update and install dependencies
RUN chmod +x /root/install/*

RUN /bin/sh -e -c /root/install/dependencies.sh

COPY ./ /root/catkin_ws/src/


    
RUN apt-get update && apt-get install -y \
    gazebo \
    libgazebo-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*


RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo 'source /usr/share/gazebo/setup.sh' >> /root/.bashrc


# this line installs catkin_ws as a working directory, all commands are gonna be executed from here
WORKDIR /root/catkin_ws 

# update the ros dependencies and update the ones that are found in the src directory
RUN rosdep update 
#RUN rosdep install --from-paths src --ignore-src -r -y --skip-keys "ros1_bridge"


