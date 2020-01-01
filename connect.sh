#!/bin/bash
cd "$(dirname "$0")"
function usage {
    printf 'usage: %s <u|r|v> [container]\n' "$0"
    printf '  Start and connect to the named container (by default, whichever is running).\n'
    printf '      u: connect text-mode to user ros.\n'
    printf '      r: connect text-mode to root.\n'
    printf '      v: connect graphically over VNC.\n'
    exit
}
if ! ( [ "$1" = 'u' ] || [ "$1" = 'r' ] || [ "$1" = 'v' ] ); then
    usage
fi
if [ "$2" ]; then
    container="$2"
else
    vstatus="$(vagrant status | grep -E '\(docker\)')"
    running="$(printf '%s\n' "$vstatus" | grep -E '\srunning\s' | awk '{print $1}')"
    count="$(printf '%s\n' "$running" | wc -l)"
    if [ "$count" = 0 ]; then
        printf 'No running containers. Options:\n\n'
        printf '%s\n' "$vstatus" | awk '{print "  " $1}'
        printf '\n'
        exit
    elif [ "$count" = 1 ]; then
        container="$running"
    else
        printf 'Multiple running containers:\n\n'
        printf '%s\n' "$running" | awk '{print "  " $1}'
        printf '\n'
        exit
    fi
fi
printf '%s\n' "$container"
if [ "$1" = 'u' ]; then
    vagrant up "$container"
    docker exec -it "$container" sudo -H -u ros zsh
elif [ "$1" = 'r' ]; then
    vagrant up "$container"
    docker exec -it "$container" bash
elif [ "$1" = 'v' ]; then
    vagrant status "$container" | head -n 3 | grep -E '\srunning\s+\(docker\)'
    isrunning="$?"
    vagrant up "$container"
    if [ "$isrunning" -ne 0 ]; then
        sleep 4
    fi
    addr="$(docker exec "$container" sh -c 'ip -4 -o addr show eth0 | awk '"'"'{print $4}'"'"' | cut -d/ -f1')"
    nohup vinagre "$addr:5900" 2>/dev/null >/dev/null &disown
else
    usage
fi
