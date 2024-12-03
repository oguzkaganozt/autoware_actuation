#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/../.."

print_help() {
    echo -e "${GREEN}Usage: configure_interface.sh [OPTIONS]${NC}"
    echo "Options:"
    echo "  --help              Display this help message"
    echo "  -h                  Display this help message"
    echo "  -i                  Interface to configure (e.g. eth0)"
    echo ""
}

parse_arguments() {
    while [ "$1" != "" ]; do
        case "$1" in
        --help | -h)
            print_help
            exit 0
            ;;
        -i)
            interface="$2"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            print_help
            exit 1
            ;;
        esac
        shift
    done
}

get_default_interface() {
    return $(ip route get 1.1.1.1 | sed -n 's/.*dev \([^\ ]*\).*/\1/p' | head -1)
}

get_gateway() {
    return $(ip route show dev "$1" | grep default | awk '{print $3}')
}

set_static_ip() { 
    echo "Setting static IP for interface: $1"

    # scan for directly connected machine IPs
    USED_IPS=$(ip -o -4 addr list | awk '{print $4}' | cut -d/ -f1 | grep -E '^(192\.168\.1\.)')
    
     # set first available IP
    for i in {100..254}; do
        IP="192.168.1.$i"
        if ! echo "$USED_IPS" | grep -q "$IP"; then
            echo "Assigning IP: $IP"
            ip addr add "$IP/24" dev "$1"

            # check ip is up
            ip addr show dev "$1" | grep "$IP"
            break
        fi
    done
}

# parse arguments
parse_arguments "$@"

# set default interface if not specified
if [ -z "$interface" ]; then
    echo "No interface specified"
    interface=$(get_default_interface)
    echo "Using default interface: $interface"
fi

# check if we have a gateway
if get_gateway "$interface"; then
    echo "Gateway detected at: $GATEWAY"
else
    echo "No gateway found - configuring static IP"
    set_static_ip "$interface"
fi



