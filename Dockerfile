FROM osrf/ros:noetic-desktop-full

# add source to bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN apt update && apt install -y git

# mount src folder from desktop to /home/ros_ws/src
COPY src/jenga_packages /home/ros_ws/src/jenga_packages

# RUN mkdir -p /home/ros_ws/src/moveit \
# && cd /home/ros_ws/src/moveit && wstool init . \
# && wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall \
# && wstool remove moveit_tutorials \
# && wstool update -t .

RUN sudo apt install ros-noetic-moveit -y

RUN mkdir -p /home/ros_ws/src/git_packages 
# RUN cd /home/ros_ws/src/git_packages && git clone https://github.com/ros-planning/moveit_tutorials.git -b master
# && git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel

#rosdep install on src folder
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /home/ros_ws; rosdep install --from-paths src --ignore-src -r -y"

# build workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /home/ros_ws; catkin_make"

RUN echo "source /home/ros_ws/devel/setup.bash" >> ~/.bashrc

RUN apt install nano -y
# Install dependencies

# set workdir as home/ros_ws
WORKDIR /home/ros_ws

CMD [ "bash" ]