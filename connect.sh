#!/bin/sh
cd "$(dirname "$0")"
if [ "$1" == 'u' ]; then
    vagrant up
    docker exec -it ros-vagrant sudo -H -u ros zsh
elif [ "$1" == 'r' ]; then
    vagrant up
    docker exec -it ros-vagrant bash
elif [ "$1" == 'v' ]; then
    vagrant status | grep '^default' | grep running
    running="$?"
    vagrant up
    if [ "$running" -ne 0 ]; then
        sleep 4
    fi
    addr="$(docker exec ros-vagrant sh -c 'ip -4 -o addr show eth0 | awk '"'"'{print $4}'"'"' | cut -d/ -f1')"
    nohup vinagre "$addr:5900" 2>&1 >/dev/null &disown
else
    echo "usage: $0 <u|r|v>"
    echo '  Start and connect to the ros-vagrant container.'
    echo '      u: connect text-mode to user ros.'
    echo '      r: connect text-mode to root.'
    echo '      v: connect graphically over VNC.'
fi
