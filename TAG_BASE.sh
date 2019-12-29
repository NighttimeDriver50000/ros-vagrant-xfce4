#!/bin/sh
cd "$(dirname "$0")"
docker build -t ros-vagrant-base -f Base.Dockerfile .
