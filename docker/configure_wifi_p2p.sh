#!/bin/bash

set -e

# === Colors ===
COLOR_WARN="\033[33m"      # yellow
COLOR_ERROR="\033[31m"     # red
COLOR_NC="\033[0m"         # reset
COLOR_SUCCESS="\033[32m"   # green

# === Configuration Variables ===
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
LOG_FILE="${SCRIPT_DIR}/network-config.log"
LOCK_FILE="${SCRIPT_DIR}/network-config.lock"
NETPLAN_CONFIG_FILE="/etc/netplan/02-openadkit-netcfg.yaml"
NETPLAN_BACKUP_FILE="backup.yaml"
WPA_TIMEOUT=30
NETPLAN_TIMEOUT=60
HOST_NUM=99  # Default value if not set

# === Utility ===
log() {
    local message=$1
    local COLOR_CODE=$2
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    
    # Log to file with timestamp and log level
    echo "[${timestamp}] ${message}" >> "$LOG_FILE"
    
    # Print to console with color, timestamp, and formatting
    echo -e "${COLOR_CODE}${message}${COLOR_NC}" >&2
}
log_info() {
    log "$1" "${COLOR_NC}"
}
log_warn() {
    log "$1" "${COLOR_WARN}"
}
log_error() {
    log "$1" "${COLOR_ERROR}"
}
log_success() {
    log "$1" "${COLOR_SUCCESS}"
}

ensure_directories() {
    mkdir -p "$(dirname "$LOG_FILE")"
    touch "$LOG_FILE"
    chmod 644 "$LOG_FILE"

    mkdir -p "$(dirname "$NETPLAN_CONFIG_FILE")"
    touch "$NETPLAN_CONFIG_FILE"
    chmod 600 /etc/netplan/*
}

fail() {
    rm -f "$LOCK_FILE"
    rm -f "$NETPLAN_CONFIG_FILE"
    log_error "Failed to configure network.."
    
    log_info "Restoring network configuration..."
    netplan apply "$NETPLAN_BACKUP_FILE"
    #cleanup_wifi_direct "*"
    rm -f "$NETPLAN_BACKUP_FILE"

    log_success "Restarting NetworkManager..."
    systemctl start NetworkManager

    exit 1
}

success() {
    rm -f "$LOCK_FILE"
    log_success "Success.."
    log_success "Restarting NetworkManager..."
    systemctl start NetworkManager

    exit 0
}

check_dependencies() {
    for cmd in wpa_supplicant wpa_cli iw iwconfig netplan; do
        if ! command -v $cmd &> /dev/null; then
            log_info "Attempting to install $cmd..."
            if ! apt-get update -qq && apt-get install -qq -y "$cmd" >/dev/null 2>&1; then
                log_error "Failed to install $cmd. Please install it manually."
                exit 1
            fi
            # Verify installation was successful
            if ! command -v $cmd &> /dev/null; then
                log_error "$cmd installation failed"
                exit 1
            fi
            log_info "$cmd installed successfully"
        fi
    done
}

# === WiFi Direct Configuration ===
check_concurrent_wifi_support() {
    local iface=$1
    [[ $(iw list | grep -A 4 "valid interface combinations" | grep -E "managed.*P2P-client") != "" ]]
}

configure_wifi_direct() {
    local iface=$1

    cat > /etc/wpa_supplicant/p2p.conf << EOF
ctrl_interface=/var/run/wpa_supplicant
update_config=1
device_name=$(hostname)
device_type=1-0050F204-1
config_methods=keypad display push_button
p2p_go_intent=10
p2p_go_ht40=1
EOF

    # Start wpa_supplicant
    timeout $WPA_TIMEOUT wpa_supplicant -B -i "$iface" -c /etc/wpa_supplicant/p2p.conf || { 
        log_error "wpa_supplicant failed to start within ${WPA_TIMEOUT} seconds"
        cleanup_wifi_direct "$iface"
        return 1
    }

    # Create P2P group
    timeout $WPA_TIMEOUT wpa_cli -i "$iface" p2p_group_add freq=2412 || {
        log_error "P2P group creation failed within ${WPA_TIMEOUT} seconds"
        cleanup_wifi_direct "$iface"
        return 1
    }

    # Check wifi p2p available
    timeout $WPA_TIMEOUT wpa_cli -i "$iface" wpa_cli status

    return 0
}

cleanup_wifi_direct() {
    local iface=$1
    
    timeout $WPA_TIMEOUT killall wpa_supplicant 2>/dev/null
    timeout $WPA_TIMEOUT wpa_cli -i "$iface" p2p_group_remove "*" 2>/dev/null
    rm -f /etc/wpa_supplicant/p2p.conf
}

# === Network Configuration ===
backup_netplan() {
    netplan get all > "$NETPLAN_BACKUP_FILE"
}

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

validate_interface() {
    local iface=$1
    local interface_path="/sys/class/net/$iface"
    
    if [[ ! -e "$interface_path" ]]; then
        log_error "Interface $iface does not exist"
        return 1
    fi
    if [[ ! -d "$interface_path" ]]; then
        log_error "$iface is not a valid network interface"
        return 1
    fi
    
    return 0
}

get_network_interfaces() {
    local type=$1  # "wifi" or "ethernet"
    local interfaces=""
    
    for iface in $(ls /sys/class/net/); do
        if [[ ! -L "/sys/class/net/$iface/device" ]]; then
            continue  # Skip non-physical interfaces
        fi
        
        if [[ $type == "wifi" && -d "/sys/class/net/$iface/wireless" ]] || \
           [[ $type == "ethernet" && ! -d "/sys/class/net/$iface/wireless" ]]; then
            if validate_interface "$iface" >/dev/null 2>&1; then
                interfaces+="$iface "
            fi
        fi
    done

    echo "$interfaces"
}

configure_wifi() {
    local interfaces=$(get_network_interfaces "wifi")
    
    for iface in $interfaces; do
        [[ -z "$iface" ]] && continue
        log_info "Found WiFi interface: $iface"
        
        # WiFi-direct configuration
        if check_concurrent_wifi_support $iface; then
            log_success "WiFi Direct support detected"
            if configure_wifi_direct "$iface"; then
                log_success "WiFi Direct configured successfully"
            else
                log_error "Failed to configure WiFi Direct on $iface"
                cleanup_wifi_direct "$iface"
            fi
        else
            log_warn "$iface does not support concurrent WiFi Direct mode"
        fi
    done
}

generate_eth_config() {
    local config=""
    local interfaces=""
    interfaces=$(get_network_interfaces "ethernet")
    
    [[ -n "$interfaces" ]] && config+="  ethernets:\n"
    for iface in $interfaces; do
        [[ -z "$iface" ]] && continue
        log_info "Found ethernet interface: $iface"
        config+="    $iface:\n"
        config+="      dhcp4: true\n"
        config+="      dhcp6: true\n"
        config+="      addresses:\n"
        config+="        - 10.0.0.${HOST_NUM}/24\n"
        config+="      optional: true\n\n"
    done
    echo -e "$config"
}

apply_netplan() {
    local config=$1
    if echo -e "$config" > "$NETPLAN_CONFIG_FILE" && \
        timeout $NETPLAN_TIMEOUT netplan apply; then
        log_success "Netplan configuration successfully applied"
    else
        log_error "Netplan configuration failed"
        fail
    fi
}

configure_network () {
    log_info "Generating eth configuration..."
    local config="network:\n  renderer: NetworkManager\n"
    config+="$(generate_eth_config)"
    log_info "Generated network configuration:"
    log_info "$(echo -e "$config" | sed 's/^/    /')"
    
    if ! validate_yaml "$config"; then
        log_error "Invalid YAML configuration"
        exit 1
    fi

    apply_netplan "$config"
    configure_wifi
}

# === Main ===
main() {
    trap fail INT TERM QUIT

    # Validation checks
    [[ $EUID -ne 0 ]] && { log_error "This script must be run as root"; exit 1; }
    [[ -f "$LOCK_FILE" ]] && { log_error "Lock file exists, another instance is running ?"; exit 1; }

    # Check Ubuntu version
    if ! grep -q "Ubuntu 22" /etc/os-release; then
        log_error "This script is designed for Ubuntu 22.x . Current system is not compatible."
        exit 1
    fi

    # Lock file
    touch "$LOCK_FILE"
    
    # Initialize
    log_info "Checking dependencies..."
    check_dependencies
    log_success "OK"
    log_info "Backing up netplan configuration..."
    backup_netplan
    log_success "OK"
    log_info "Ensuring directories..."
    ensure_directories
    log_success "OK"
    
    # Stop NetworkManager
    log_info "Stopping NetworkManager..."
    systemctl stop NetworkManager
    log_success "OK"

    # Configure network
    log_info "Configuring network..."
    configure_network
    log_success "OK"
    
    success
}

# Start
main