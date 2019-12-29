# Shell flags
set -x
set -e
# Username
USERNAME=vagrant
# Move files into place
ls /tmp/*
mv /tmp/* /
# Initial update
apt-get update
apt-get upgrade -y
# Install xvfb
apt-get install -y xvfb
# Install a VNC server
apt-get install -y x11vnc iproute2
# Install a window manager
if [ "$1" = xfce ]; then
    export RV_XFCE=1
    DEBIAN_FRONTEND="noninteractive" apt-get install -y xubuntu-desktop
else
    apt-get install -y software-properties-common
    add-apt-repository -y ppa:klaus-vormweg/awesome
    apt-get update
    apt-get install -y awesome
fi
# Install sudo
apt-get install -y sudo
# Create a user
if [ "$USERNAME" != vagrant ]; then
    useradd -m "$USERNAME"
    printf '%s\n%s\n' "$USERNAME" "$USERNAME" | passwd "$USERNAME"
fi
echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >>/etc/sudoers
# Set up awesome
if ! [ "$RV_XFCE" ]; then
    apt-get install -y awesome-extra roxterm
    cd "/home/$USERNAME"
    mkdir -p .config/awesome
    cp -r /etc/xdg/awesome/debian .config/awesome
    ln -s "/home/$USERNAME/catkin_ws/rc.lua" .config/awesome/rc.lua
    #ADD av8pves.jpg /
fi
# Install ROS
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
apt-get update
apt-get upgrade -y
apt-get install -y ros-kinetic-desktop-full
rosdep init
# Chown home
chown "$USERNAME:$USERNAME" .
chown "$USERNAME:$USERNAME" *
chown -R "$USERNAME:$USERNAME" .?*
# Set up rosdep
sudo -H -u "$USERNAME" rosdep update
apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
# Install various system utilities
apt-get install -y build-essential psmisc
# Install stuff for zsh
apt-get install -y zsh powerline python3-powerline mlocate
# Install stuff for antigen
apt-get install -y curl git
# Set up zshrc
sudo -H -u "$USERNAME" curl -L git.io/antigen >antigen.zsh
sudo -H -u "$USERNAME" ln -s catkin_ws/zshrc .zshrc
# Install vim
apt-get install -y vim-gtk3
# Set up vimrc
#ENV TERM xterm-256color
touch .vimrc
cp .vimrc .vimrc.source
echo 'source ~/catkin_ws/vimrc' >>.vimrc.source
#ADD catkin_ws/vimrc /
cat /vimrc >>.vimrc
rm /vimrc
# Prepare for Pathogen
mkdir -p .vim/autoload .vim/bundle
# Chown home
chown "$USERNAME:$USERNAME" .
chown "$USERNAME:$USERNAME" *
chown -R "$USERNAME:$USERNAME" .[rv]*
# Install Pathogen
curl -LSso .vim/autoload/pathogen.vim https://tpo.pe/pathogen.vim
# Install YouCompleteMe
apt-get install -y git cmake python-dev python3-dev
cd .vim/bundle
git clone https://github.com/Valloric/YouCompleteMe.git
cd YouCompleteMe
git submodule update --init --recursive
./install.py --clang-completer
cd ../../..
# Run vim once
mv .vimrc.source .vimrc
cd .vim/bundle
sh -c 'echo ":quit" | vim -E'
cd ../..
# Install ardupilot prerequisites
apt-get install -y python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml
apt-get install -y python-scipy python-opencv ccache gawk git python-pip python-pexpect
pip install --upgrade pip
apt-get remove -y python-pip
hash -d pip
pip -V
pip install future
pip install pymavlink MAVProxy
# Download ardupilot
sudo -H -u "$USERNAME" git clone git://github.com/ArduPilot/ardupilot.git
cd ardupilot
sudo -H -u "$USERNAME" git submodule update --init --recursive
cd ..
# Install firmware proxy prerequisites
apt-get install -y python3-requests python3-dnspython python3-bottle
# Set up firmware proxy
echo '127.0.0.1 firmware.ardupilot.org' >>/etc/hosts
# Allow firmware proxy to bind to low-numbered ports
#RUN setcap CAP_NET_BIND_SERVICE=+eip /home/ros/catkin_ws/firmware_proxy.py
apt-get install -y authbind
touch /etc/authbind/byport/80
chmod 777 /etc/authbind/byport/80
# Install mavros
apt-get install -y nmap ros-kinetic-mavros
/opt/ros/kinetic/lib/mavros/install_geographiclib_datasets.sh
# Set up SSH
mkdir -p .ssh
#ln -s ../catkin_ws/ssh_config .ssh/config
#ln -s ../catkin_ws/id_rsa .ssh/id_rsa
#ln -s ../catkin_ws/id_rsa.pub .ssh/id_rsa.pub
# Chown home
chown "$USERNAME:$USERNAME" .
chown "$USERNAME:$USERNAME" *
chown -R "$USERNAME:$USERNAME" .[rv]*
# Install PyGObject
apt-get install -y libgtk-3-0 python-gi python-gi-dev python-gi-cairo python-pil
apt-get install -y gir1.2-gtk-3.0 libgtk-3-dev
apt-get install -y python-requests python-requests-cache
# Install catkin tools
apt-get install -y python-catkin-tools
# Install less
apt-get install -y less
# Install turtlebot3 prereqs
sudo apt-get install -y ros-kinetic-joy ros-kinetic-teleop-twist-joy \
    ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc \
    ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan \
    ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python \
    ros-kinetic-rosserial-server ros-kinetic-rosserial-client \
    ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server \
    ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro \
    ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view \
    ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
# Add the default roxterm config
mkdir -p .config/roxterm.sourceforge.net/Profiles
#ADD Default /home/ros/.config/roxterm.sourceforge.net/Profiles/
mv /Default .config/roxterm.sourceforge.net/Profiles/
# Wipe the Pixhawk memory
#ADD wipe_test_rover.bash /
# Most things will happen in the catkin workspace
#cd /home/ros/catkin_ws
# Login with zsh for ros user
chsh -s /bin/zsh "$USERNAME"
# Run a command in a window manager on startup
#ADD StartInWM.bash /
#ADD roscore.bash /
# Index the file system
updatedb
# Use the SITL sim (Comment to disable)
#ENV SITL 1
# WM on start
#CMD ["bash", "/StartInWM.bash", "awesome"]
#CMD ["bash", "/StartInWM.bash", "awesome", "bash", "/roscore.bash"]
apt-get install -y xorg xinit
apt-get install -y virtualbox-guest-x11
apt-get install -y sddm
if [ "$RV_XFCE" ]; then
    echo 'exec xfce4-session' >.xinitrc
else
    echo 'exec awesome' >.xinitrc
fi
systemctl enable sddm.service
systemctl set-default graphical.target
poweroff
