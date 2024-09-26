#!/bin/bash

xhost +

docker run -it --net=host -v /dev/shm:/dev/shm -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e ROS_DOMAIN_ID=99 \
    -e ROS_LOCALHOST_ONLY=0 \
    ghcr.io/autowarefoundation/autoware:universe-amd64 ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:=/opt/autoware/share/kashiwanoha_map \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit \
    scenario_simulation:=true \
    rviz:=false