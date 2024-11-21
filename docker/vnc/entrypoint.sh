#!/usr/bin/env bash

# Source ROS2 and Autoware
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/autoware/setup.bash

# Start VNC server
vncserver :1 -auth $HOME/.Xauthority -geometry 1024x768 -depth 16 > /dev/null 2>&1
VNC_RESULT=$?

if [ $VNC_RESULT -ne 0 ]; then
    echo "Failed to start VNC server (exit code: $VNC_RESULT)"
    exit $VNC_RESULT
fi

sleep 2

# Start NoVNC
websockify --web=/usr/share/novnc/ --cert=/etc/ssl/certs/novnc.crt --key=/etc/ssl/private/novnc.key 6080 localhost:5901 & > /dev/null 2>&1

# Configure ngrok
ngrok config add-authtoken $NGROK_AUTH_TOKEN

# Start ngrok tunnel
ngrok tcp 6080 --url=simviz.openadkit.ngrok.app

# Print message
echo -e "\033[32m-------------------------------------------------------------------------\033[0m"
echo -e "\033[32mVNC server is running and accessible at localhost:5901\033[0m"
echo -e "\033[32mNoVNC web interface available at localhost:6080\033[0m"
echo -e "\033[32mYou can connect to the VNC server with a VNC viewer\033[0m"
echo -e "\033[32mYou can connect to the NoVNC web interface with a web browser\033[0m"
echo -e "\033[32m-------------------------------------------------------------------------\033[0m"

# Run command
exec "$@"
