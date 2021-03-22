#!/bin/sh
set -x
set -e
url='http://mirrors.kernel.org/ubuntu/pool/main/p/python3-defaults'
if [ "$2" = security ]; then
  url='http://security.ubuntu.com/ubuntu/pool/main/p/python3.6'
elif [ "$2" ]; then
  url="$2"
fi
pkg="$1"
mkdir -p /packages
cd /packages
curl -LSsO "$url/$pkg"
dpkg -i "$pkg"
