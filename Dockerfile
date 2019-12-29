FROM ros-vagrant-base
RUN apt-get update
RUN apt-get upgrade -y
# Install a window manager
RUN apt-get install -y software-properties-common
RUN add-apt-repository -y ppa:klaus-vormweg/awesome
RUN apt-get update
RUN apt-get install -y awesome
# Set up awesome
RUN apt-get install -y awesome-extra roxterm
RUN mkdir -p /root/.config/awesome
RUN cp -r /etc/xdg/awesome/debian /root/.config/awesome
RUN ln -s /home/ros/catkin_ws/rc.lua /root/.config/awesome/rc.lua
ADD av8pves.jpg /
# For vim
ENV TERM xterm-256color
# Most things will happen in the catkin workspace
WORKDIR /home/ros/catkin_ws
# Index the file system
RUN updatedb
# WM on start
CMD ["bash", "/StartInWM.bash", "awesome", "bash", "/roscore.bash"]
