#! /bin/bash

set -e
GREEN='\033[32m'
RED='\033[31m'
NC='\033[0m' # No Color

test_peer_services() {
    echo -e "\nTesting peer services on $HOSTNAME"
    SERVICES=$(echo $SERVICES | tr ',' ' ')

    # Ping each service
    for service in $SERVICES; do
        if [ "$service" == "$HOSTNAME" ]; then
            continue
        fi

        echo "Testing $service"
        service_up=false
        
        # Ping
        if ping -c 3 $service; then
            service_up=true
        else
            for i in {1..5}; do
                echo "Retry attempt $i/5"
                if ping -c 3 $service; then
                    service_up=true
                    break
                fi
                sleep 1
            done
        fi

        # Fail fast if any service is unreachable
        if [ "$service_up" = false ]; then
            echo -e "\n----------------------------------------\n"
            echo -e "${RED}Error: Ping failed for service: $service, exiting..${NC}"
            exit 1
        fi
    done

    echo -e "\n----------------------------------------\n"
    echo -e "\n${GREEN}Services: $SERVICES - Reachable${NC}"
    echo -e "\n\n----------------------------------------\n\n"
    sleep 1
}

update_fast_xml() {
    echo -e "\nEditing fast.xml"
    
    # Add services to fast.xml
    for service in $SERVICES; do
        sed -i "/<initialPeersList>/a\                    <locator>\n                        <udpv4>\n                            <address>$service</address>\n                        </udpv4>\n                    </locator>" $FASTRTPS_DEFAULT_PROFILES_FILE
    done
    
    echo -e "${GREEN}Updated fast.xml with peers: $SERVICES${NC}"
}

configure_wan_network() {
    if [ -z "$ENABLE_WAN" ]; then
        echo -e "\n WAN is disabled, skipping wan network configuration\n"
        return
    fi

    echo -e "\nConfiguring wan network.."
    ufw allow 41641/udp
    ufw allow 41644/udp
    tailscaled &
    tailscale up --authkey $TS_AUTHKEY
    tailscale netcheck
    echo -e "\n\n${GREEN}WAN NETWORK CONFIGURED${NC}"
    echo -e "\n\n----------------------------------------\n\n"
}

run_bandwidth_test() {
    sleep 3

    if [ $HOSTNAME == "node1" ]; then
        echo -e "\nRunning bandwidth test on $HOSTNAME as client"
        iperf3 -c node2 -p 41644 -u -t 10 -b 10M
    elif [ $HOSTNAME == "node2" ]; then
        echo -e "\nRunning bandwidth test on $HOSTNAME as server"
        iperf3 -s -p 41644
    fi
}

run() {
    # Configure local and wan networks if services are specified
    if [ -n "$SERVICES" ]; then
        update_fast_xml
        configure_wan_network
        test_peer_services
    else
        echo -e "\n${RED}No services specified, skipping network configuration${NC}"
    fi

    echo -e "\n${GREEN}Starting application with environment variables:${NC}"
    printenv | sort
    source /opt/ros/$ROS_DISTRO/setup.bash
    echo -e "\n\n----------------------------------------\n\n"
    sleep 2

    exec "$@"
}

run "$@"
