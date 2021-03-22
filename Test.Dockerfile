FROM osrf/ros:kinetic-desktop-full-xenial
RUN apt-get update
RUN apt-get upgrade -y
# Install xvfb
RUN apt-get install -y xvfb
# Install a VNC server
RUN apt-get install -y x11vnc iproute2
# Create a user
RUN useradd -m ros
# Install various system utilities
RUN apt-get install -y sudo build-essential psmisc
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
# Install Pathogen
RUN curl -LSso /home/ros/.vim/autoload/pathogen.vim https://tpo.pe/pathogen.vim
# Install Python
RUN apt-get install -y git cmake python-dev
ADD get-dpkg.sh /
RUN chmod +x /get-dpkg.sh
# - - -
# python3-minimal deps
RUN /get-dpkg.sh libc6_2.27-3ubuntu1.2_amd64.deb http://security.ubuntu.com/ubuntu/pool/main/g/glibc
RUN /get-dpkg.sh libssl1.1_1.1.1-1ubuntu2.1~18.04.8_amd64.deb http://security.ubuntu.com/ubuntu/pool/main/o/openssl
RUN /get-dpkg.sh libpython3.6-minimal_3.6.9-1~18.04ubuntu1.4_amd64.deb security
RUN /get-dpkg.sh python3.6-minimal_3.6.9-1~18.04ubuntu1.4_amd64.deb security
# python3 deps
RUN /get-dpkg.sh libreadline7_7.0-3_amd64.deb http://mirrors.kernel.org/ubuntu/pool/main/r/readline
RUN /get-dpkg.sh python3-minimal_3.6.5-3_amd64.deb
RUN /get-dpkg.sh libpython3.6-stdlib_3.6.9-1~18.04ubuntu1.4_amd64.deb security
RUN /get-dpkg.sh libpython3-stdlib_3.6.5-3_amd64.deb
RUN /get-dpkg.sh python3.6_3.6.9-1~18.04ubuntu1.4_amd64.deb security
# libpython3-dev deps
RUN /get-dpkg.sh libpython3.6_3.6.9-1~18.04ubuntu1.4_amd64.deb security
RUN /get-dpkg.sh libpython3.6-dev_3.6.9-1~18.04ubuntu1.4_amd64.deb security
# python3-dev deps
RUN /get-dpkg.sh python3_3.6.5-3_amd64.deb
RUN /get-dpkg.sh libpython3-dev_3.6.5-3_amd64.deb
RUN /get-dpkg.sh python3.6-dev_3.6.9-1~18.04ubuntu1.4_amd64.deb security
RUN /get-dpkg.sh python3-lib2to3_3.6.5-3_all.deb http://mirrors.kernel.org/ubuntu/pool/main/p/python3-stdlib-extensions
RUN /get-dpkg.sh python3-distutils_3.6.5-3_all.deb http://mirrors.kernel.org/ubuntu/pool/main/p/python3-stdlib-extensions
# python3-dev
RUN /get-dpkg.sh python3-dev_3.6.5-3_amd64.deb
# - - -
# Install stuff for zsh
RUN apt-get install -y zsh powerline python3-powerline mlocate
# Install stuff for antigen
RUN apt-get install -y curl git
# Set up zshrc
RUN sudo -H -u ros curl -L git.io/antigen >/home/ros/antigen.zsh
RUN sudo -H -u ros ln -s catkin_ws/zshrc /home/ros/.zshrc
# Chown home
WORKDIR /home/ros
RUN chown ros:ros .
RUN chown ros:ros *
RUN chown -R ros:ros .[rv]*
# Install YouCompleteMe
