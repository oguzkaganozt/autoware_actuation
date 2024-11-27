FROM ros:humble-ros-base-jammy

# Install packages one at a time to avoid QEMU issues
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    apt-get install -yq --no-install-recommends python3-pip \
    wget \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Install tailscale
RUN wget -O- https://pkgs.tailscale.com/stable/ubuntu/jammy.noarmor.gpg | tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null && \
    wget -O- https://pkgs.tailscale.com/stable/ubuntu/jammy.tailscale-keyring.list | tee /etc/apt/sources.list.d/tailscale.list && \
    apt-get update && \
    apt-get install -y tailscale && \
    rm -rf /var/lib/apt/lists/*

# Create a workspace for our listener script
WORKDIR /ros2_ws
COPY listener/listener.py /ros2_ws/listener.py

# Install ufw
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    apt-get install -yq --no-install-recommends ufw && \
    rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# CMD ["ddsperf", "pong", "waitset"]
# CMD ["sleep", "infinity"]
CMD ["python3", "/ros2_ws/listener.py"]
