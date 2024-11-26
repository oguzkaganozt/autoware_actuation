#!/usr/bin/env bash
# shellcheck disable=SC2086,SC2124

set -e

# Define terminal colors
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/.."

# Function to print help message
print_help() {
    echo "${GREEN}Usage: run.sh [OPTIONS]${NC}"
    echo "Options:"
    echo "  --help          Display this help message"
    echo "  -h              Display this help message"
    echo "  --web           Run web simulation"
    echo "  --platform      Target platform (amd64, arm64)"
    echo ""
}

# Parse arguments
parse_arguments() {
    while [ "$1" != "" ]; do
        case "$1" in
        --help | -h)
            print_help
            exit 1
            ;;
        --web)
            option_web=true
            ;;
        --platform)
            option_platform="$2"
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

# Set platform
set_platform() {
    if [ -n "$option_platform" ]; then
        platform="$option_platform"
    else
        platform="linux/amd64"
        arch="amd64"
        if [ "$(uname -m)" = "aarch64" ]; then
            platform="linux/arm64"
            arch="arm64"
        fi
    fi

    echo "Platform: ${platform}"
    echo "Architecture: ${arch}"
    export ARCH="${arch}"
}

# Run simulation
run_simulation() {
    if [ "$option_web" = "true" ]; then
        docker compose -f "${SCRIPT_DIR}/web-simulation.docker-compose.yaml" up
    else
        xhost +
        docker compose -f "${SCRIPT_DIR}/local-simulation.docker-compose.yaml" up
    fi
}

parse_arguments "$@"
set_platform
run_simulation
