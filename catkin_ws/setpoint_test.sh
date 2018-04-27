#!/bin/bash
set -e
cd "$(dirname "$0")"
./make.sh
kill_test() {
  pkill -f ardurover
  pkill -f mavproxy.py
}
set +e
kill_test
set -e
trap kill_test EXIT
roxterm --disable-sm --fork -e ./sim_test_rover.sh --console --map
roxterm --disable-sm --fork -e ./mavros.sh
echo 'Running setpoint_test_node. ^C to quit.'
#rosrun setpoint_test setpoint_test_node
sleep infinity
