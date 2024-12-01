ARG ARCH
FROM ghcr.io/autowarefoundation/demo_packages:planning-control-pass-${ARCH}

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

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/shared/fastdds.xml
COPY etc/fastdds.xml /root/shared/fastdds.xml
ENV CYCLONEDDS_URI=/root/shared/cyclonedds.xml
COPY etc/cyclonedds.xml /root/shared/cyclonedds.xml

COPY etc/simulation /autoware/simulation
COPY etc/map /autoware/map

# Copy entrypoint script
COPY etc/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
