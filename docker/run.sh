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

print_help() {
    echo -e "${GREEN}Usage: run.sh [OPTIONS]${NC}"
    echo "Options:"
    echo "  --help              Display this help message"
    echo "  -h                  Display this help message"
    echo "  --web               Run web visualizer"
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
        --web)
            option_web=true
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
    if [ "$remote_host" ] && [ -z "$remote_arch" ]; then
        echo -e "${RED}Remote architecture is required when remote host is specified${NC}"
        print_help
        exit 1
    fi
}

set_architecture() {
    # Validate remote architecture
    if [ -n "$remote_arch" ]; then
        if [ "$remote_arch" != "amd64" ] && [ "$remote_arch" != "arm64" ]; then
            echo -e "${RED}Invalid remote architecture: $remote_arch${NC}. Must be one of amd64, arm64."
            print_help
            exit 1
        fi
    fi

    # Set local architecture
    arch="amd64"
    if [ "$(uname -m)" = "aarch64" ]; then
        arch="arm64"
    fi

    if [ -n "$remote_arch" ]; then
        export ARCH="${remote_arch}"
        echo "Target Architecture: ${remote_arch}"
    else
        export ARCH="${arch}"
        echo "Local Architecture: ${arch}"
    fi
}

move_files_to_simulator_home() {
    cp -a "${WORKSPACE_ROOT}/docker/etc/map" $SIMULATOR_HOME
    cp -a "${WORKSPACE_ROOT}/docker/etc/simulation" $SIMULATOR_HOME
    chmod -R a+rw $SIMULATOR_HOME/map $SIMULATOR_HOME/simulation
}

cleanup() {
    echo -e "${BLUE}Cleaning up containers...${NC}"
    docker ps -q --filter "name=planning-control" | xargs -r docker kill
    docker ps -q --filter "name=web-visualizer" | xargs -r docker kill
    docker ps -q --filter "name=simulator" | xargs -r docker kill
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

    # Copy all required files to remote machine
    echo -e "${BLUE}Copying required files to remote machine...${NC}"
    if [ -n "$remote_port" ]; then
        scp -p -r -P "${remote_port}" "${WORKSPACE_ROOT}/docker/etc/map" "${remote_host}:/home/${remote_user}"
        scp -p -r -P "${remote_port}" "${WORKSPACE_ROOT}/docker/etc/simulation" "${remote_host}:/home/${remote_user}"
    else
        scp -p -r -o StrictHostKeyChecking=no "${WORKSPACE_ROOT}/docker/etc/map" "${remote_host}:/home/${remote_user}"
        scp -p -r -o StrictHostKeyChecking=no "${WORKSPACE_ROOT}/docker/etc/simulation" "${remote_host}:/home/${remote_user}"
    fi

    # Save the new host to the file
    echo "$remote_host" >> "$hosts_file"
}

run_simulation() {
    trap cleanup SIGINT SIGTERM

    if [ -n "$remote_host" ]; then
        remote_user=$(echo "$remote_host" | cut -d '@' -f 1)
        if ! check_saved_remote_host; then
            configure_remote_host
        else
            echo -e "${BLUE}Reconnecting to known host ${remote_host}...${NC}"
            if ! test_docker_remote_context; then
                configure_remote_host
            fi
        fi

        echo -e "${BLUE}Using remote Docker context...${NC}"
        docker context use remote-machine
        export SIMULATOR_HOME="/home/${remote_user}"
    else
        docker context use default
        export SIMULATOR_HOME="${HOME}"
        move_files_to_simulator_home
    fi
    echo "SIMULATOR_HOME: ${SIMULATOR_HOME}"

    # Kill all running containers
    cleanup

    # Run simulation
    echo -e "\n${NC}Starting planning-control...${NC}"
    docker compose -f "${SCRIPT_DIR}/simulation.docker-compose.yaml" --env-file "${SCRIPT_DIR}/etc/simulator.env" up planning-control --remove-orphans -d

    # Wait for planning-control to start
    echo -e "${NC}Waiting for planning-control to start...${NC}"
    for i in {1..50}; do
        printf "\r[%-50s] %d%%" "$(printf '#%.0s' $(seq 1 $i))" $((i*100/50))
        sleep 1
    done
    echo -e "\n${GREEN}Planning-control is ready!${NC}"
    echo -e "${NC}----------------------------------------${NC}"

    # Run web visualizer    
    echo -e "\n${NC}Starting web visualizer...${NC}"
    docker compose -f "${SCRIPT_DIR}/simulation.docker-compose.yaml" --env-file "${SCRIPT_DIR}/etc/simulator.env" up web-visualizer --remove-orphans -d

    # Wait for web visualizer to start
    echo -e "${NC}Waiting for web visualizer to start...${NC}"
    for i in {1..4}; do
        printf "\r[%-4s] %d%%" "$(printf '#%.0s' $(seq 1 $i))" $((i*100/4))
        sleep 1
    done
    echo -e "\n${GREEN}Web visualizer is ready!${NC}"
    echo -e "${GREEN}Access the visualizer at ${GREEN}https://${NGROK_URL}/vnc.html${NC}"
    echo -e "${NC}----------------------------------------${NC}"

    # Wait for simulator to start
    echo -e "${NC}Waiting for simulator to start...${NC}"
    for i in {1..8}; do
        printf "\r[%-8s] %d%%" "$(printf '#%.0s' $(seq 1 $i))" $((i*100/8))
        sleep 1
    done
    echo -e "\n${NC}Starting simulator...${NC}"
    docker compose -f "${SCRIPT_DIR}/simulation.docker-compose.yaml" --env-file "${SCRIPT_DIR}/etc/simulator.env" up simulator --remove-orphans
    echo -e "\n${GREEN}Simulator is ready!${NC}"
    echo -e "${NC}----------------------------------------${NC}"

    docker compose -f "${SCRIPT_DIR}/simulation.docker-compose.yaml" --env-file "${SCRIPT_DIR}/etc/simulator.env" wait planning-control web-visualizer simulator
}

source "${SCRIPT_DIR}/etc/simulator.env"
parse_arguments "$@"
check_remote_arguments
set_architecture
run_simulation
