FROM ros:humble-ros-base-jammy

# Install network tools and Python
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    apt-get install -y --no-install-recommends python3-pip \
    curl \
    net-tools \
    iputils-ping \
    ufw \
    iperf3 \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Install tailscale
RUN curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.noarmor.gpg | sudo tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null && \
    curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.tailscale-keyring.list | sudo tee /etc/apt/sources.list.d/tailscale.list && \
    apt-get update && \
    apt-get install -y tailscale && \
    rm -rf /var/lib/apt/lists/*

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/shared/fast.xml
COPY etc/fast.xml /root/shared/fast.xml

# Create a workspace for our node
WORKDIR /ros2_ws
COPY src/node.py /ros2_ws/node.py

COPY etc/entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "/ros2_ws/node.py"]
