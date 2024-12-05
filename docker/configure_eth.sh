#!/bin/bash

set -e

# === Colors ===
COLOR_ERROR="\033[31m"     # red
COLOR_NC="\033[0m"         # nocolor
COLOR_SUCCESS="\033[32m"   # green

# === Configuration Variables ===
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
LOG_FILE="${SCRIPT_DIR}/network-config.log"
LOCK_FILE="${SCRIPT_DIR}/network-config.lock"
NETPLAN_CONFIG_FILE="/etc/netplan/02-openadkit-netcfg.yaml"
NETPLAN_BACKUP_FILE="backup.yaml"
NETPLAN_TIMEOUT=30
HOST_NUM=99  # Default value if not set

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

ensure_directories() {
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
    if [ -f "$NETPLAN_BACKUP_FILE" ]; then
        netplan apply "$NETPLAN_BACKUP_FILE"
        rm -f "$NETPLAN_BACKUP_FILE"
    fi

    exit 1
}

success() {
    rm -f "$LOCK_FILE"
    log_success "Success.."

    exit 0
}

check_dependencies() {
    if ! command -v netplan &> /dev/null; then
        log_info "Attempting to install netplan..."
        if ! apt-get update -qq && apt-get install -qq -y netplan >/dev/null 2>&1; then
            log_error "Failed to install netplan. Please install it manually."
            fail
        fi
        # Verify installation was successful
        if ! command -v netplan &> /dev/null; then
            log_error "netplan installation failed"
            fail
        fi
        log_info "netplan installed successfully"
    fi
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
    local interfaces=""
    
    for iface in $(ls /sys/class/net/); do
        if [[ ! -L "/sys/class/net/$iface/device" ]]; then
            continue  # Skip non-physical interfaces
        fi
        
        if [[ ! -d "/sys/class/net/$iface/wireless" ]]; then
            if validate_interface "$iface" >/dev/null 2>&1; then
                interfaces+="$iface "
            fi
        fi
    done

    echo "$interfaces"
}

generate_eth_config() {
    local config=""
    local interfaces=""
    interfaces=$(get_network_interfaces)
    
    if [[ -n "$interfaces" ]]; then
        config+="network:\n  renderer: NetworkManager\n"
        config+="  ethernets:\n"
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
    fi
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
    config+="$(generate_eth_config)"

    if [[ -n "$config" ]]; then
        log_info "Generated network configuration:"
        log_info "$(echo -e "$config" | sed 's/^/    /')"
    else
        log_error "No network interfaces found"
        fail
    fi
    
    if ! validate_yaml "$config"; then
        log_error "Invalid YAML configuration"
        fail
    fi

    apply_netplan "$config"
}

# === Main ===
main() {
    trap fail INT TERM QUIT

    # Validation checks
    [[ $EUID -ne 0 ]] && { echo "This script must be run as root"; exit 1; }
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

    # Configure network
    log_info "Configuring network..."
    configure_network
    log_success "OK"
    
    success
}

# Start
main