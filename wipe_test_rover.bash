#!/bin/bash
sudo -H -u ros authbind --deep /home/ros/catkin_ws/apm_ws/firmware_proxy.py &
sudo -H -u ros zsh -c 'source ~/.zshrc && ~/catkin_ws/apm_ws/test_rover/sim_here_rover_skid.sh -w </dev/null'
