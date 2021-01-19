#!/bin/bash

while [[ true ]]; do
	echo "waiting for mrs_drone_spawner"
	if rosnode list | grep -q "mrs_drone_spawner"; then
		# rosservice call /mrs_simulation_spawner/spawn "1 --f450 --run --delete --enable-rangefinder --enable-rangefinder-up --enable-rplidar --enable-ground-truth --enable-bluefox-camera --enable-realsense-front --pos 8.1 2.0 1.0 3.14"
		rosservice call /mrs_drone_spawner/spawn "1 --f450 --enable-rangefinder --enable-rangefinder-up --enable-rplidar --enable-ground-truth --enable-bluefox-camera --enable-realsense-front --pos 8.1 2.0 1.0 3.14"
		break
	fi
done
