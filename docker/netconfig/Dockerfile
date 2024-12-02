FROM ros:humble-ros-base

# Install network tools and Python
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip \
    curl \
    net-tools \
    iputils-ping \
    ufw \
    iperf3 \
    avahi-utils \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Enable avahi-daemon
RUN systemctl enable avahi-daemon

# Install tailscale
RUN curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.noarmor.gpg | sudo tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null && \
    curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.tailscale-keyring.list | sudo tee /etc/apt/sources.list.d/tailscale.list && \
    apt-get update && \
    apt-get install -y tailscale && \
    rm -rf /var/lib/apt/lists/*

# Install openvpn
RUN apt-get update && apt-get install -y openvpn easy-rsa && \
    rm -rf /var/lib/apt/lists/*

# Copy cyclonedds.xml
ENV CYCLONEDDS_URI=/root/shared/cyclonedds.xml
COPY etc/cyclonedds.xml /root/shared/cyclonedds.xml

# Copy entrypoint script
COPY etc/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]