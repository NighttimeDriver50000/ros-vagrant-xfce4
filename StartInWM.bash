#!/usr/bin/env bash
set -ex
mkdir -p logs
chmod 777 logs
exec >logs/StartInWM.log 2>&1
TZ='America/New_York' date
rm -f /tmp/.X1-lock
export DISPLAY=:1
wm="$1"
shift
if [ "$1" = "--1080" ]; then
    Xvfb "$DISPLAY" -screen 0 1920x1080x24 &>logs/Xvfb.log &
    shift
elif [ "$1" = "--900" ]; then
    Xvfb "$DISPLAY" -screen 0 1600x900x24 &>logs/Xvfb.log &
    shift
else
    Xvfb "$DISPLAY" -screen 0 1280x720x24 &>logs/Xvfb.log &
fi
sleep 1
x11vnc -display "$DISPLAY" -nevershared -forever &>logs/x11vnc.log &
sleep 1
"$wm" &>logs/wm.log &
sleep 1
addr="$(ip -4 -o addr show eth0 | awk '{print $4}' | cut -d/ -f1)"
echo
echo "VNC now open at $addr:5900"
if [ "$1" ]; then
    if [ "$1" = '--hold' ]; then
        shift
        "$@" &>logs/at.log
        wait %3
    else
        echo "When '$*' dies, the VNC server will close."
        "$@" &>logs/at.log
    fi
else
    wait %3
fi
kill $(jobs -p)
sleep 1
