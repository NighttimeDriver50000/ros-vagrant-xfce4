#!/bin/bash
rosservice list | xargs -L1 sh -c 'echo "$0" ; rosservice info "$0" ; echo'
