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
    echo "  --remote_host       (Optional) Remote target machine user@host (e.g. root@192.168.1.100) to run workloads on"
    echo "  --remote_port       (Optional) Remote target machine port (e.g. 2222)"
    echo "  --remote_arch       (Mandatory if --remote_host is specified) Remote target machine architecture (one of amd64, arm64)"
    echo "  --clear_remote_host Clear all saved remote hosts"
    echo ""
}

parse_arguments() {
    while [ "$1" != "" ]; do
        case "$1" in
        --help | -h)
            print_help
            exit 0
            ;;
        --clear_remote_host)
            rm -f "$HOME/.saved_remote_hosts"
            ;;
        --remote_host)
            remote_host="$2"
            shift
            ;;
        --remote_port)
            remote_port="$2"
            shift
            ;;
        --remote_arch)
            remote_arch="$2"
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

check_remote_arguments() {
    if [ -z "$remote_host" ] || [ -z "$remote_arch" ]; then
        echo -e "${RED}Remote host and architecture are required${NC}"
        print_help
        exit 1
    fi
}

set_architecture() {
    # Validate remote architecture
    if [ "$remote_arch" != "amd64" ] && [ "$remote_arch" != "arm64" ]; then
        echo -e "${RED}Invalid remote architecture: $remote_arch${NC}. Must be one of amd64, arm64."
        exit 1
    fi

    # Set local architecture
    local_arch="amd64"
    if [ "$(uname -m)" = "aarch64" ]; then
        local_arch="arm64"
    fi
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

copy_ssh_key() {
    # Generate SSH key pair if it doesn't exist
    if [ ! -f ~/.ssh/id_rsa ]; then
        echo -e "${BLUE}Generating new SSH key pair...${NC}"
        ssh-keygen -t rsa -b 4096 -f ~/.ssh/id_rsa -N ""
    fi

    # Copy SSH key to remote machine
    if [ -n "$remote_port" ]; then
        echo -e "${BLUE}Running ssh-copy-id -i ~/.ssh/id_rsa.pub -p ${remote_port} ${remote_host}${NC}"
        timeout 10 ssh-copy-id -i ~/.ssh/id_rsa.pub -p "${remote_port}" "${remote_host}" || { echo -e "${RED}SSH key copy timed out${NC}"; exit 1; }
    else
        timeout 10 ssh-copy-id -i ~/.ssh/id_rsa.pub "${remote_host}" || { echo -e "${RED}SSH key copy timed out${NC}"; exit 1; }
    fi
}

check_saved_remote_host() {
    hosts_file="$HOME/.saved_remote_hosts"
    
    # Create the file if it doesn't exist
    if [ ! -f "$hosts_file" ]; then
        touch "$hosts_file"
        return 1
    fi

    if grep -q "^${remote_host}$" "$hosts_file"; then
        return 0  # Host found
    else
        return 1  # Host not found
    fi
}

configure_remote_host() {
    echo -e "${BLUE}First time connecting to ${remote_host}, setting up...${NC}"
    echo -e "${BLUE}Copying SSH key to remote machine...${NC}"
    copy_ssh_key
    echo -e "${BLUE}Creating remote Docker context...${NC}"
    create_remote_docker_context

    # Save the new host to the file
    echo "$remote_host" >> "$hosts_file"
}

cleanup() {
    echo -e "${BLUE}Cleaning up containers...${NC}"
    docker context use remote-machine
    docker ps -aq --filter "name=listener-*" | xargs -r docker rm -f
    docker context use default
    docker ps -aq --filter "name=talker-*" | xargs -r docker rm -f
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

    # generate a random run id
    export RUN_ID=$(uuidgen | tr -dc 'a-zA-Z0-9' | fold -w 8 | head -n 1)
    echo "RUN_ID: $RUN_ID"

    # Edit fast.xml
    sed -i "s/listener-[a-zA-Z0-9]\+/listener-${RUN_ID}/g" "${SCRIPT_DIR}/fast.xml"
    sed -i "s/talker-[a-zA-Z0-9]\+/talker-${RUN_ID}/g" "${SCRIPT_DIR}/fast.xml"

    # Copy fast.xml to remote machine
    chmod 777 "${SCRIPT_DIR}/fast.xml"
    cp "${SCRIPT_DIR}/fast.xml" /var/tmp/fast.xml
    scp -p /var/tmp/fast.xml aadp:/var/tmp/fast.xml

    # cat /tmp/fast.xml 
    # echo "----------------------------------------"
    # ssh aadp "cat /tmp/fast.xml"

    # Build containers
    # docker buildx build --platform linux/amd64,linux/arm64 -f listener/listener.dockerfile -t ghcr.io/oguzkaganozt/autoware_snapshots:listener --push .
    # docker buildx build --platform linux/amd64,linux/arm64 -f talker/talker.dockerfile -t ghcr.io/oguzkaganozt/autoware_snapshots:talker --push .

    
    # Run talker
    echo -e "\n${NC}Starting talker...${NC}"
    local_arch="amd64"
    docker context use default
    ARCH=${local_arch} docker compose -f "${SCRIPT_DIR}/docker-compose.yaml" up talker --remove-orphans --pull always -d

    # Run listener
    echo -e "\n${NC}Starting listener...${NC}"
    docker context use remote-machine
    remote_arch="arm64"
    ARCH=${remote_arch} docker compose -f "${SCRIPT_DIR}/docker-compose.yaml" up listener --remove-orphans --pull always

    sleep infinity

    # # # # Show logs from both containers
    # echo -e "\n${NC}Showing logs from both containers...${NC}"
    # docker --context default logs -f talker-${RUN_ID} | sed 's/^/[Talker] /' | grep -v '^[Talker] [0-9]\{4\}/[0-9]\{2\}/[0-9]\{2\}' &
    # docker --context remote-machine logs -f listener-${RUN_ID} | sed 's/^/[Listener] /' | grep -v '^[Listener] [0-9]\{4\}/[0-9]\{2\}/[0-9]\{2\}'
}

# parse_arguments "$@"
# check_remote_arguments
# set_architecture
run_simulation
