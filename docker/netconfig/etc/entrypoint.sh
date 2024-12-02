#! /bin/bash

set -e

# Colors
GREEN='\033[32m'
RED='\033[31m'
NC='\033[0m' # No Color

# Handshake port
HANDSHAKE_PORT=41645

cyclone_dds_add_interface() {
    echo -e "\nAdding cyclone-dds interface"
    
    # Add eth0 interface after the lo interface
    sed -i '/<NetworkInterface.*name="lo".*\/>/a\                <NetworkInterface autodetermine="false" name="eth0" priority="default" multicast="default" />' $CYCLONEDDS_URI
    
    echo -e "\n\n${GREEN}CYCLONE-DDS INTERFACE ADDED${NC}"
    echo -e "\n\n----------------------------------------\n\n"
}

cyclone_dds_add_peer() {
    echo -e "\nEditing cyclonedds.xml"
    
    # Add peer to cyclonedds.xml
    sed -i "/<Peers>/a\                <Peer Address=\"$1\" />" $CYCLONEDDS_URI
    
    echo -e "${GREEN}Updated cyclonedds.xml with peer: $1${NC}"
    echo -e "\n\n----------------------------------------\n\n"
}

add_peer_to_hosts() {
    echo -e "\nAdding peer to /etc/hosts"
    echo "$1 $HOSTNAME" >> /etc/hosts
}

test_peers() {
    echo -e "\nTesting peers on $HOSTNAME"

    # Ping each peer
    for peer in $peers; do
        if [ "$peer" == "$HOSTNAME" ]; then
            continue
        fi

        echo "Testing $peer"
        peer_up=false
        
        # Ping
        if ping -c 3 $peer; then
            peer_up=true
        else
            for i in {1..5}; do
                echo "Retry attempt $i/5"
                if ping -c 3 $peer; then
                    peer_up=true
                    break
                fi
                sleep 1
            done
        fi

        # Fail fast if any peer is unreachable
        if [ "$peer_up" = false ]; then
            echo -e "\n----------------------------------------\n"
            echo -e "${RED}Error: Ping failed for peer: $peer, exiting..${NC}"
            return 1
        fi
    done

    echo -e "\n----------------------------------------\n"
    echo -e "\n${GREEN}Peers: ${peers[@]} - Reachable${NC}"
    echo -e "\n\n----------------------------------------\n\n"
    return 0
}

configure_wan_network() {
    if [ -z "$TS_AUTHKEY" ]; then
        echo -e "\n${RED}No tailscale auth key is given, skipping wan network configuration${NC}"
        echo -e "\n${RED}Run with TS_AUTHKEY=<tailscale-auth-key> to configure wan network${NC}"
        config=tailscale
    elif [ -z "$OPENVPN_CONFIG" ]; then
        echo -e "\n${RED}No openvpn config is given, skipping wan network configuration${NC}"
        echo -e "\n${RED}Run with OPENVPN_CONFIG=<path-to-openvpn-config> to configure wan network${NC}"
        config=openvpn
    else
        echo -e "\n${RED}No tailscale auth key or openvpn config is given, skipping wan network configuration${NC}"
        echo -e "\n${RED}Run with TS_AUTHKEY=<tailscale-auth-key> or OPENVPN_CONFIG=<path-to-openvpn-config> to configure wan network${NC}"
        return 1
    fi

    echo -e "\nConfiguring wan network.."
    if [ "$config" == "tailscale" ]; then
        ufw allow 41641/udp
        ufw allow 41644/udp
        tailscaled &
        tailscale up --authkey $TS_AUTHKEY
        tailscale netcheck
    elif [ "$config" == "openvpn" ]; then
        echo -e "\n${RED}OpenVPN is not supported yet, skipping wan network configuration${NC}"
    fi
    echo -e "\n\n${GREEN}WAN NETWORK CONFIGURED${NC}"
    echo -e "\n\n----------------------------------------\n\n"
    return 0
}

configure_local_network() {
    echo -e "\nConfiguring local network.."
    test_peers
    echo -e "\n\n${GREEN}LOCAL NETWORK CONFIGURED${NC}"
    echo -e "\n\n----------------------------------------\n\n"
}

configure_network() {
    # IP fragmentation settings
    sysctl -w net.ipv4.ipfrag_time=3  # in seconds, default is 30 s
    sysctl -w net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB, default is 256 KiB

    # Configure network and search peers in networks
    if configure_local_network "$@"; then
        if configure_wan_network; then
            exit 1
        fi
    fi
}

run_handshake_server() {
    nc -l $HANDSHAKE_PORT | while read line; do
        if [ "$line" = "REQ" ]; then
            echo "Received REQ from $HOST"
            echo "ACK" | nc $HOST $HANDSHAKE_PORT
        fi
    done &
}

send_handshake_request() {
    for i in {1..10}; do
        echo "Sending REQ to $1 (attempt $i/10)"
        echo "REQ" | nc $1 $HANDSHAKE_PORT
        nc -l $HANDSHAKE_PORT | while read line; do
            if [ "$line" = "ACK" ]; then
                echo "Received ACK from $1"
                return 0
            fi
        done
        sleep 0.5
    done
    
    return 1
}

# Discover all LAN machines
discover_lan_peers() {
    nmap -sn 169.254.1.0/24
}

# Discover all WAN machines
discover_wan_peers() {
    if [ "$config" == "tailscale" ]; then
        # Discover Tailscale peers
        tailscale status | while read -r ip hostname user os status; do
            # Skip if line doesn't start with an IP address (100.*)
            [[ $ip =~ ^100\. ]] || continue
            
            # If status is not "offline", print the IP and hostname
            if [[ ! "$status" =~ "offline" ]]; then
                echo "$ip $hostname"
            fi
        done
    elif [ "$config" == "openvpn" ]; then
        echo -e "\n${RED}OpenVPN is not supported yet, skipping wan network configuration${NC}"
    fi
}

run() {
    # Configure local and wan networks if peers are specified
    if [ -n "$PEERS" ]; then
        get_peers
        configure_network
    else
        echo -e "\n${RED}No peers are given, skipping network configuration${NC}"
        echo -e "\n${RED}Run with PEERS=<comma-separated-peers> to configure network${NC}"
        exit 0
    fi
    
    echo -e "${GREEN}All done!${NC}"
    echo -e "\n\n----------------------------------------\n\n"
    sleep 1
}

run
exec "$@"
