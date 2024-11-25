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
        *)
            echo "Unknown option: $1"
            print_help
            exit 1
            ;;
        esac
        shift
    done
}

# Run simulation
run_simulation() {
    if [ "$option_web" = "true" ]; then
        xhost +
        docker compose -f "${SCRIPT_DIR}/web-simulation.docker-compose.yaml" up
    else
        docker compose -f "${SCRIPT_DIR}/local-simulation.docker-compose.yaml" up
    fi
}

parse_arguments "$@"
run_simulation
