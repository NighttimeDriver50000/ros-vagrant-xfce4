#!/bin/sh
cd "$(dirname "$0")"
sim_vehicle.py --vehicle=APMrover2 --frame=rover-skid "$@"
