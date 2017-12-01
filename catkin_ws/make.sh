#!/bin/sh
cd "$(dirname "$0")"
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"
