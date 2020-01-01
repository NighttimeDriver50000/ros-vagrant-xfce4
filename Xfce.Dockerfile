FROM ros-vagrant-base
RUN apt-get update
RUN apt-get upgrade -y
# Install a window manager
RUN DEBIAN_FRONTEND="noninteractive" apt-get install -y xubuntu-desktop
# For vim
ENV TERM xterm-256color
# Most things will happen in the catkin workspace
WORKDIR /home/ros/catkin_ws
# Index the file system
RUN updatedb
# WM on start
CMD ["bash", "/StartInWM.bash", "xfce4-session", "--900", "bash", "/roscore.bash"]
