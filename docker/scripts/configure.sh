#!/bin/bash

set -e

# === Colors ===
COLOR_ERROR="\033[31m"      # red
COLOR_NC="\033[0m"          # nocolor
COLOR_SUCCESS="\033[32m"    # green
COLOR_WARN="\033[33m"       # yellow

# === Configuration Variables ===
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
LOG_FILE="${SCRIPT_DIR}/network-config.log"
LOCK_FILE="${SCRIPT_DIR}/network-config.lock"
NETPLAN_CONFIG_FILE="${SCRIPT_DIR}/01-openadkit-netcfg.yaml"
NETPLAN_BACKUP_FILE="${SCRIPT_DIR}/01-backup.yaml"
NETPLAN_TIMEOUT=30

# === Utility ===
log() {
    local message=$1
    local COLOR_CODE=$2
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    
    echo "[${timestamp}] ${message}" >> "$LOG_FILE"
    echo -e "${COLOR_CODE}${message}${COLOR_NC}" >&2
}
log_info() {
    log "$1" "${COLOR_NC}"  
}
log_error() {
    log "$1" "${COLOR_ERROR}"
}
log_success() {
    log "$1" "${COLOR_SUCCESS}"
}
log_warn() {
    log "$1" "${COLOR_WARN}"
}

print_help() {
    echo "Usage: configure.sh [OPTIONS] <server_name|client_name>"
    echo "Options:"
    echo "  -h | --help          Display this help message"
    echo "  -s | --server        Start as server"
    echo "  -c | --client        Start as client"
    echo "  -e | --ethernets     (Optional) Configure all ethernets with static IP addresses"
    echo "  -d | --daemon        (Optional) Run as daemon"
}

parse_args() {
    while [ "$1" != "" ]; do
        case "$1" in
        -h | --help)
            print_help
            exit 1
            ;;
        -s | --server)
            option_server=true
            ;;
        -c | --client)
            option_client=true
            ;;
        -e | --ethernets)
            [[ $EUID -ne 0 ]] && { log_error "Script must be run as root if you want to configure ethernets"; exit 1; }
            option_ethernets=true
            ;;
        -d | --daemon)
            option_daemon=true
            ;;
        *)
            if [[ -n "$1" ]]; then
                option_name=$1
            else
                log_error "Invalid argument: $1"
                exit 1
            fi
            ;;
        esac
        shift 1
    done

    if [[ -n "$option_server" && -n "$option_client" ]]; then
        log_error "Both server and client options cannot be set at the same time"
        print_help
        exit 1
    elif [[ -z "$option_server" && -z "$option_client" ]]; then
        log_error "Either server or client option must be set"
        print_help
        exit 1
    fi
}

daemonize() {
    if [[ "$option_daemon" == true && "$DAEMONIZED" != true ]]; then
        log_info "Starting in daemon mode..."
        export DAEMONIZED=true
        nohup "$0" "${@}" </dev/null >/dev/null 2>&1 &
        exit 0
    fi
    
    exec 1>> /dev/null
    exec 2>> /dev/null
}

check_directories() {
    log_info "Creating log file..."
    touch "$LOG_FILE" && chmod 644 "$LOG_FILE"

    # Create netplan backup file
    mkdir -p "$(dirname "$NETPLAN_CONFIG_FILE")"
    netplan get all > "$NETPLAN_BACKUP_FILE" && chmod 600 "$NETPLAN_BACKUP_FILE"

    # Create netplan config file
    touch "$NETPLAN_CONFIG_FILE" && chmod 600 "$NETPLAN_CONFIG_FILE"
}

cleanup() {
    rm -f "$LOCK_FILE"
    rm -f "$NETPLAN_CONFIG_FILE"

    if [ -f "$NETPLAN_BACKUP_FILE" ]; then
        netplan apply
        rm -f "$NETPLAN_BACKUP_FILE"
    fi

    pkill -P $$ || true
    pkill avahi-publish || true
    exit 0
}

# === Network Configuration ===
validate_yaml() {
    local config=$1
    local temp_file=$(mktemp)
    echo -e "$config" > "$temp_file"
    
    if ! netplan get all &>/dev/null < "$temp_file"; then
        log_error "Invalid YAML configuration"
        rm "$temp_file"
        return 1
    fi
    rm "$temp_file"
    return 0
}

generate_host_num() {
    local mac_address
    mac_address=$(ip link show "$(get_network_interfaces | head -n1)" | awk '/ether/ {print $2; exit}')
    HOST_NUM=$(echo "$mac_address" | tr -d ':' | cksum | cut -d' ' -f1)
    HOST_NUM=$((HOST_NUM % 253 + 2))
    
    log_info "Generated host number: $HOST_NUM from MAC: $mac_address"
}

get_network_interfaces() {
    for iface in /sys/class/net/*; do
        iface=$(basename "$iface")
        [[ -L "/sys/class/net/$iface/device" && ! -d "/sys/class/net/$iface/wireless" ]] && echo "$iface"
    done
}

generate_eth_config() {
    local config="network:\n  renderer: NetworkManager\n  ethernets:\n"
    
    for iface in $(get_network_interfaces); do
        config+="    $iface:\n"
        config+="      dhcp4: true\n"
        config+="      dhcp6: true\n"
        config+="      addresses: [10.0.0.${HOST_NUM}/24]\n"
        config+="      optional: true\n\n"
    done
    log_info "Generated network configuration:"
    log_info "$(echo -e "$config" | sed 's/^/    /')"
    echo -e "$config"
}

apply_netplan() {
    log_info "Applying netplan configuration..."
    local config=$1
    if echo -e "$config" > "$NETPLAN_CONFIG_FILE" && \
        timeout $NETPLAN_TIMEOUT netplan apply; then
        log_success "Netplan configuration successfully applied"
    else
        log_error "Netplan configuration failed"
        fail
    fi
}

configure_ethernets () {
    log_info "Configuring network..."
    generate_host_num
    config="$(generate_eth_config)"
    
    if ! validate_yaml "$config"; then
        log_error "Invalid YAML configuration"
        fail
    fi

    apply_netplan "$config"

    if command -v ufw >/dev/null 2>&1; then
        if ufw status | grep "Status: active"; then
            log_warn "Firewall is enabled, consider disabling it for maximum DDS availability"
        fi
    fi

    # Fine tuning
    sysctl -w net.core.rmem_max=1073741824  # 1 GiB, default is 208 KiB
    sysctl -w net.ipv4.ipfrag_time=3  # in seconds, default is 30 s
    sysctl -w net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB, default is 256 KiB
}

avahi_publish() {
    log_info "Publishing avahi service..."

    if [[ -n "$option_server" ]]; then
        service_name="server-${option_name}"
    else
        service_name="client-${option_name}"
    fi

    while true; do
        avahi-publish -a -s "$service_name" _openadkit._tcp 8888 || log_warn "Avahi publish failed, retrying in 5 seconds..."
        sleep 5
    done
}

avahi_discover() {
    log_info "Discovering avahi services..."
    local ipv4_line
    local interface
    local ip_address
    
    while true; do
        # Get avahi-browse output and extract first IPv4 line starting with "="
        ipv4_line=$(avahi-browse -trlp _openadkit._tcp | grep "^=.*IPv4" | head -n1)
        
        if [[ -n "$ipv4_line" ]]; then
            # Format: =;interface;IPv4;name;service;domain;hostname;ip;port;
            interface=$(echo "$ipv4_line" | awk -F';' '{print $2}')
            ip_address=$(echo "$ipv4_line" | awk -F';' '{print $8}')
            node_name=$(echo "$ipv4_line" | awk -F';' '{print $4}')
            add_interface_xml "$interface"
            add_address_xml "$ip_address" "$node_name"
        fi
        sleep 0.5
    done
}

add_interface_xml () {
    local interface=$1
    grep -q "$interface" "$CYCLONEDDS_URI" 2>/dev/null && return 0
    log_info "Adding new interface to CycloneDDS: $interface"
    sed -i "/<\/Interfaces>/i\                <NetworkInterface autodetermine=\"false\" name=\"$interface\" priority=\"default\" multicast=\"default\" />" "$CYCLONEDDS_URI"
}

declare -g node_list=""

add_address_xml () {
    local addr=$1
    local node_name=$2

    [[ "$addr" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || return 0
    [[ "$node_list" == *"$node_name"* ]] && return 0
    log_info "Adding new node $node_name with address $addr to CycloneDDS"
    sed -i "/<\/Peers>/i\                <Peer Address=\"$addr\" />" "$CYCLONEDDS_URI"
    node_list+="$node_name,"
}

generate_cyclonedds_xml() {
    local cyclonedds_config="${SCRIPT_DIR}/cyclonedds.xml"
    log_info "Generating CycloneDDS configuration at ${cyclonedds_config}"
    
    cat > "$cyclonedds_config" << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface autodetermine="false" name="lo" priority="default" multicast="default" />
            </Interfaces>
            <AllowMulticast>default</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        </General>
        <Internal>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
        <Discovery>
            <Peers>
            </Peers>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
        </Discovery>
    </Domain>
</CycloneDDS>
EOF

    chmod 644 "$cyclonedds_config"
    export CYCLONEDDS_URI="$cyclonedds_config"
}

# === Main ===
main() {
    grep -q "Ubuntu 22" /etc/os-release || { log_error "This script is designed for Ubuntu 22.x . Current system is not compatible."; exit 1; }
    trap cleanup INT TERM QUIT
    parse_args "$@"
    [[ "$option_daemon" == true ]] && daemonize "$@"
    [[ -f "$LOCK_FILE" ]] && { log_error "Lock file exists, another instance is running ?"; exit 1; }
    touch "$LOCK_FILE"

    # Configure ethernet interfaces if requested
    if [[ -z "$(get_network_interfaces)"  || -z "$option_ethernets" ]]; then
        log_warn "Ethernet interfaces is not configured"
    else
        check_directories
        configure_ethernets
    fi

    # Run discovery
    generate_cyclonedds_xml
    avahi_publish & 
    avahi_discover &
    
    [[ -z "$option_daemon" ]] && sleep 2 && log_info "Running discovery..." && for _ in {1..8}; do echo -n "."; sleep 1; done && echo " done" && cleanup
    wait
}

# Run
main "$@"