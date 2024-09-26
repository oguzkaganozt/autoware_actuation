#!/bin/bash

xhost +
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --net=host -v /dev/shm:/dev/shm -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ghcr.io/autowarefoundation/openadkit_demo.autoware:aws-reinvent-simulator-monolithic-amd64 ros2 launch scenario_test_runner scenario_test_runner.launch.py   architecture_type:=awf/universe/20240605   record:=false   scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample.yaml'   sensor_model:=sample_sensor_kit   vehicle_model:=sample_vehicle initialize_duration:=90