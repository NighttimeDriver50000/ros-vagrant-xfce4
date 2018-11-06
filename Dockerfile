FROM osrf/ros:kinetic-desktop-full-xenial
RUN apt-get update
RUN apt-get upgrade -y
# Install xvfb
RUN apt-get install -y xvfb
# Install a VNC server
RUN apt-get install -y x11vnc iproute2
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
# Create a user
RUN useradd -m ros
# Install various system utilities
RUN apt-get install -y sudo build-essential psmisc
# Install stuff for zsh
RUN apt-get install -y zsh powerline python3-powerline mlocate
# Install stuff for antigen
RUN apt-get install -y curl git
# Set up zshrc
RUN sudo -H -u ros curl -L git.io/antigen >/home/ros/antigen.zsh
RUN sudo -H -u ros ln -s catkin_ws/zshrc /home/ros/.zshrc
# Install vim
RUN apt-get install -y vim-gtk3
# Set up vimrc
ENV TERM xterm-256color
RUN touch /home/ros/.vimrc
RUN cp /home/ros/.vimrc /home/ros/.vimrc.source
RUN echo 'source ~/catkin_ws/vimrc' >>/home/ros/.vimrc.source
ADD catkin_ws/vimrc /
RUN cat /vimrc >>/home/ros/.vimrc
RUN rm /vimrc
# Prepare for Pathogen
RUN mkdir -p /home/ros/.vim/autoload /home/ros/.vim/bundle
# Chown home
WORKDIR /home/ros
RUN chown ros:ros .
RUN chown ros:ros *
RUN chown -R ros:ros .[rv]*
# Install Pathogen
RUN curl -LSso /home/ros/.vim/autoload/pathogen.vim https://tpo.pe/pathogen.vim
# Install YouCompleteMe
RUN apt-get install -y git cmake python-dev python3-dev
WORKDIR /home/ros/.vim/bundle
RUN git clone https://github.com/Valloric/YouCompleteMe.git
WORKDIR /home/ros/.vim/bundle/YouCompleteMe
RUN git submodule update --init --recursive
RUN ./install.py --clang-completer
# Run vim once
RUN mv /home/ros/.vimrc.source /home/ros/.vimrc
WORKDIR /home/ros/.vim/bundle
RUN sh -c 'echo ":quit" | vim -E'
# Install ardupilot prerequisites
RUN apt-get install -y python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml
RUN apt-get install -y python-scipy python-opencv ccache gawk git python-pip python-pexpect
RUN pip install --upgrade pip
RUN pip install future
RUN pip install pymavlink MAVProxy
# Download ardupilot
WORKDIR /home/ros
RUN sudo -H -u ros git clone git://github.com/ArduPilot/ardupilot.git
WORKDIR /home/ros/ardupilot
RUN sudo -H -u ros git submodule update --init --recursive
# Install firmware proxy prerequisites
RUN apt-get install -y python3-requests python3-dnspython python3-bottle
# Set up firmware proxy
RUN echo '127.0.0.1 firmware.ardupilot.org' >>/etc/hosts
# Allow firmware proxy to bind to low-numbered ports
#RUN setcap CAP_NET_BIND_SERVICE=+eip /home/ros/catkin_ws/firmware_proxy.py
RUN apt-get install -y authbind
RUN touch /etc/authbind/byport/80
RUN chmod 777 /etc/authbind/byport/80
# Install mavros
RUN apt-get install -y nmap ros-kinetic-mavros
RUN /opt/ros/kinetic/lib/mavros/install_geographiclib_datasets.sh
# Set up SSH
RUN mkdir -p /home/ros/.ssh
RUN ln -s /home/ros/catkin_ws/ssh_config /home/ros/.ssh/config
RUN ln -s /home/ros/catkin_ws/id_rsa /home/ros/.ssh/id_rsa
RUN ln -s /home/ros/catkin_ws/id_rsa.pub /home/ros/.ssh/id_rsa.pub
# Chown home
WORKDIR /home/ros
RUN chown ros:ros .
RUN chown ros:ros *
RUN chown -R ros:ros .[rv]*
# Install PyGObject
RUN apt-get install -y libgtk-3-0 python-gi python-gi-dev python-gi-cairo python-pil
RUN apt-get install -y gir1.2-gtk-3.0 libgtk-3-dev
RUN apt-get install -y python-requests python-requests-cache
# Install catkin tools
RUN apt-get install -y python-catkin-tools
# Install less
RUN apt-get install -y less
# Install turtlebot3 prereqs
RUN sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy \
    ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc \
    ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan \
    ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python \
    ros-kinetic-rosserial-server ros-kinetic-rosserial-client \
    ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server \
    ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro \
    ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view \
    ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
# Add the default roxterm config
RUN mkdir -p /root/.config/roxterm.sourceforge.net/Profiles
ADD Default /root/.config/roxterm.sourceforge.net/Profiles/
# Wipe the Pixhawk memory
ADD wipe_test_rover.bash /
# Most things will happen in the catkin workspace
WORKDIR /home/ros/catkin_ws
# Login with zsh for ros user
RUN chsh -s /bin/zsh ros
# Run a command in a window manager on startup
ADD StartInWM.bash /
ADD roscore.bash /
# Index the file system
RUN updatedb
# Use the SITL sim (Comment to disable)
ENV SITL 1
# WM on start
#CMD ["bash", "/StartInWM.bash", "awesome"]
CMD ["bash", "/StartInWM.bash", "awesome", "bash", "/roscore.bash"]
