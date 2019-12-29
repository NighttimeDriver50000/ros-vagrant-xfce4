#!/bin/sh
if [ "$GCS_CLIENT_MODE" ]; then
    roslaunch jetyak_multi_gcs_py apm2.launch
else
    roslaunch mavros apm2.launch 'fcu_url:=udp://127.0.0.1:14551@'
fi
