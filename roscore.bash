#!/bin/bash
if [ $SITL ]; then
    bash /wipe_test_rover.bash
    sudo -H -u ros authbind --deep /home/ros/catkin_ws/apm_ws/firmware_proxy.py &
fi
if [ "$GCS_CLIENT_MODE" ]; then
    sudo -H -u ros sleep infinity
else
    sudo -H -u ros bash -c 'source /opt/ros/kinetic/setup.bash && roscore'
fi
