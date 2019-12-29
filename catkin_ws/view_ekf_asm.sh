#!/bin/sh
cd "$(dirname "$0")"
g++ -Wall -std=c++11 -O3 -S -o - -I src/mckinnc_ekf/include -I /usr/include/eigen3 src/mckinnc_ekf/src/ekf.cpp 2>&1 | less
