#!/bin/bash

set -e

# Define terminal colors
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")

print_help() {
    echo -e "${GREEN}Usage: run.sh [OPTIONS]${NC}"
    echo "Options:"
    echo "  --help              Display this help message"
    echo "  -h                  Display this help message"
    echo "  --remotes-file      Path to the remotes file"
    echo ""
}

parse_arguments() {
    while [ "$1" != "" ]; do
        case "$1" in
        --help | -h)
            print_help
            exit 0
            ;;
        --remotes-file)
            add_remotes_to_known_hosts "$2"
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

add_remotes_to_known_hosts() {
    while read -r remote; do
        echo "$remote" >> ~/.ssh/known_hosts
    done < "$1"
}

test_docker_remote_context() {
    if ! timeout 5 docker --context remote-machine ps; then
        echo -e "${RED}Failed to connect to remote Docker context${NC}"
        docker context use default
        docker context rm remote-machine
        return 1
    fi
    echo -e "${GREEN}Successfully connected to remote Docker context${NC}"
    return 0
}

create_remote_docker_context() {
    # Test 
    if docker context inspect remote-machine > /dev/null 2>&1; then
        docker context use default
        docker context rm remote-machine
    fi

    # create docker context
    if [ -n "$remote_port" ]; then
        docker context create remote-machine --docker "host=ssh://${remote_host}:${remote_port}"
    else
        docker context create remote-machine --docker "host=ssh://${remote_host}"
    fi

    # Test docker context
    if ! test_docker_remote_context; then
        exit 1
    fi
}

cleanup() {
    echo -e "${BLUE}Cleaning up containers...${NC}"
    docker --context remote-machine ps -aq --filter "name=node" | xargs -r docker --context remote-machine rm -f
    docker --context default ps -aq --filter "name=node" | xargs -r docker --context default rm -f
}

run_simulation() {
    trap cleanup SIGINT SIGTERM

    # if ! check_saved_remote_host; then
    #     configure_remote_host
    # else
    #     echo -e "${BLUE}Reconnecting to known host ${remote_host}...${NC}"
    #     if ! test_docker_remote_context; then
    #         configure_remote_host
    #     fi
    # fi

    cleanup

    # Build and push container
    docker buildx build --platform linux/amd64,linux/arm64 -t ghcr.io/oguzkaganozt/autoware_snapshots:master --push .

    # Run node1 and node4
    echo -e "\n${NC}Starting node1 and node4...${NC}"
    docker context use default
    docker compose -f "${SCRIPT_DIR}/docker-compose.yaml" up node1 node4 --remove-orphans --pull always -d

    # Run node2 and node3
    echo -e "\n${NC}Starting node2 and node3...${NC}"
    docker context use remote-machine
    docker compose -f "${SCRIPT_DIR}/docker-compose.yaml" up node2 node3 --remove-orphans --pull always
}

# parse_arguments "$@"
run_simulation
