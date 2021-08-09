# Import base ROS Melodic image to build from
FROM ros:melodic

# Update packages
RUN apt update \ 
        && apt upgrade -y \
        && rm -rf /var/lib/apt/lists/*

# Create workspace directory 
RUN mkdir -p /root/catkin_ws/src/

# Add your package
COPY . /root/catkin_ws/src/ros_battleship

# Source Melodic environment when opening new shell
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

# Build catkin_ws
RUN bash -c "source /opt/ros/melodic/setup.bash; cd /root/catkin_ws; catkin_make"

RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# run launch file 
ENTRYPOINT ["/root/catkin_ws/src/ros_battleship/ros_entrypoint.sh"]
