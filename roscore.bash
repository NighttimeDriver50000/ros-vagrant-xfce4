#!/bin/bash
bash /wipe_test_rover.bash
sudo -H -u ros authbind --deep /home/ros/catkin_ws/apm_ws/firmware_proxy.py &
sudo -H -u ros bash -c 'source /opt/ros/kinetic/setup.bash && roscore'
