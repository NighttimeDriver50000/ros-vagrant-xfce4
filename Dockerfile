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
# Install stuff for zsh
RUN apt-get install -y zsh powerline python3-powerline mlocate
# Install stuff for antigen
RUN apt-get install -y curl git
# Set up zshrc
RUN curl -L git.io/antigen >/home/ros/antigen.zsh
RUN ln -s catkin_ws/zshrc /home/ros/.zshrc
# Install vim
RUN apt-get install -y vim-gtk3
# Set up vimrc
ENV TERM xterm-256color
RUN echo 'source ~/catkin_ws/vimrc' >>/home/ros/.vimrc
# Set up Pathogen
RUN mkdir -p /home/ros/.vim/autoload /home/ros/.vim/bundle
RUN curl -LSso /home/ros/.vim/autoload/pathogen.vim https://tpo.pe/pathogen.vim
WORKDIR /home/ros/.vim/bundle
RUN sh -c 'echo ":quit" | vim -E'
# Install various system utilities
RUN apt-get install -y sudo build-essential psmisc
# Install ardupilot prerequisites
RUN apt-get install -y python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml
RUN apt-get install -y python-scipy python-opencv ccache gawk git python-pip python-pexpect
RUN pip install --upgrade pip
RUN pip install future
RUN pip install pymavlink MAVProxy
# Download ardupilot
WORKDIR /home/ros
RUN git clone git://github.com/ArduPilot/ardupilot.git
WORKDIR /home/ros/ardupilot
RUN git submodule update --init --recursive
# Install firmware proxy prerequisites
RUN apt-get install -y python3-requests python3-dnspython python3-bottle
# Set up firmware proxy
RUN echo '127.0.0.1 firmware.ardupilot.org' >>/etc/hosts
# Allow firmware proxy to bind to low-numbered ports
RUN setcap CAP_NET_BIND_SERVICE=+eip /home/ros/apm_ws/test_rover/firmware_proxy.py
# Chown home
WORKDIR /home/ros
RUN chown -R ros:ros .
# Most things will happen in the catkin workspace
WORKDIR /home/ros/catkin_ws
# Login with zsh for ros ser
RUN chsh -s /bin/zsh ros
# Run a command in a window manager on startup
ADD StartInWM.bash /
ADD roscore.bash /
#CMD ["bash", "/StartInWM.bash", "awesome"]
CMD ["bash", "/StartInWM.bash", "awesome", "bash", "/roscore.bash"]
