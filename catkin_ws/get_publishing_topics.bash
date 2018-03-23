#!/bin/bash
for topic in $(rostopic list); do
  export topic
  if [ "$(timeout 1 rostopic echo "$topic")" ]; then
    echo "$topic"
    if [ "$1" = '-v' ]; then
      rostopic info "$topic"
    fi
  fi
done
