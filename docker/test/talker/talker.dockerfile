FROM ros:humble-ros-base-jammy

# Install network tools and Python
RUN apt-get update && apt-get install -y \
    python3-pip \
    curl \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# Install tailscale
RUN curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.noarmor.gpg | sudo tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null
RUN curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.tailscale-keyring.list | sudo tee /etc/apt/sources.list.d/tailscale.list
RUN apt-get update
RUN apt-get install -y tailscale

# Create a workspace for our listener script
WORKDIR /ros2_ws
COPY talker/talker.py /ros2_ws/talker.py

# Install ufw
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    apt-get install -yq --no-install-recommends ufw && \
    rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# CMD ["ddsperf", "ping", "50Hz", "2048", "waitset"]
CMD ["python3", "/ros2_ws/talker.py"]
# CMD ["sleep", "infinity"]