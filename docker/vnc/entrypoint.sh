#!/usr/bin/env bash

# Source ROS2 and Autoware
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/autoware/setup.bash

# Start VNC server
vncserver :1 -auth $HOME/.Xauthority -geometry 1024x768 -depth 16

sleep 5

# Start NoVNC
websockify --web=/usr/share/novnc/ --cert=/etc/ssl/certs/novnc.crt --key=/etc/ssl/private/novnc.key 6080 localhost:5901
