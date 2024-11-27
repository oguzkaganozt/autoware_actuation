#! /bin/bash

set -e

set_static_ip() {
    # Get default network interface
    if [ -z "$INTERFACE" ]; then
        INTERFACE=$(ip route get 1.1.1.1 | awk '{for(i=1;i<=NF;i++) if ($i=="dev") print $(i+1)}' | head -n1)
    fi

    STATIC_IP="192.168.1.${NETWORK_ID}"

    echo "Using interface: $INTERFACE"
    echo "Using static ip: $STATIC_IP"

    # Set static ip
    ip addr add $STATIC_IP/24 dev $INTERFACE

    # Set default route
    # ip route add default via $STATIC_IP dev $INTERFACE
}

run_tailscale() {
    echo "Configuring tailscale.."
    tailscaled &
    tailscale up
}

configure_local_network() {
    echo "Running local network.."
    set_static_ip
}

configure_wan_network() {
    echo "Running wan network.."
    run_tailscale
}

# cleanup() {
#     echo "Cleaning up.."
#     echo "Logging out from tailscale.."
#     tailscale logout
#     echo "Exiting.."
#     sleep infinity
#     exit 0
# }

run() {
    # # Network settings with configurable values from environment variables
    # IPFRAG_TIME=${IPFRAG_TIME:-3}  # Default 3 seconds
    # IPFRAG_HIGH_THRESH=${IPFRAG_HIGH_THRESH:-134217728}  # Default 128 MiB

    # # Apply network settings
    # sysctl -w net.ipv4.ipfrag_time=$IPFRAG_TIME
    # sysctl -w net.ipv4.ipfrag_high_thresh=$IPFRAG_HIGH_THRESH
    # sysctl net.ipv4.ipfrag_time net.ipv4.ipfrag_high_thresh

    ufw allow 41641/udp

    if [ -z "$NETWORK_ID" ]; then
        echo "NETWORK_ID is not given defaulting to WEB config"
        configure_wan_network
    else
        configure_local_network
    fi
    sleep 5

    echo "\n\n ----------------------------------------"
    echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
    echo "ROS_DISTRO: $ROS_DISTRO"
    echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
    echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    echo "RUN_ID: $RUN_ID"
    echo "HOSTNAME: $HOSTNAME"
    source /opt/ros/$ROS_DISTRO/setup.bash

    if [ -n "${TAILSCALE_TEST}" ]; then
        echo "TAILSCALE_TEST: $TAILSCALE_TEST"
        while true; do
            if [ "${HOSTNAME}" == "talker-${RUN_ID}" ]; then
                echo "Pinging listener-${RUN_ID}"
                tailscale ping --verbose listener-${RUN_ID} || true
            fi
            sleep 1
        done
    else
        exec "$@"
    fi

    sleep infinity
}

run "$@"
