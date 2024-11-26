#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/.."

# Function to print help message
print_help() {
    echo "${GREEN}Usage: build.sh [OPTIONS]${NC}"
    echo "Options:"
    echo "  --help          Display this help message"
    echo "  -h              Display this help message"
    echo "  --platform      Target platform (one of linux/amd64, linux/arm64). By default it is the local architecture."
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

# Clone repositories
clone_repositories() {
    pip3 install -y vcstool
    cd "$WORKSPACE_ROOT"
    if [ ! -d "src" ]; then
        mkdir -p src
        vcs import src <simulator.repos
    else
        echo "Source directory already exists. Updating repositories..."
        vcs import src <simulator.repos
        vcs pull src
    fi
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

# Build images
build_images() {
    # https://github.com/docker/buildx/issues/484
    export BUILDKIT_STEP_LOG_MAX_SIZE=10000000

    echo "Building images for platform: $platform"
    echo "Targets: ${targets[*]}"

    set -x
    docker buildx bake --load --progress=plain -f "$SCRIPT_DIR/docker-bake.hcl" \
        --set "*.context=$WORKSPACE_ROOT" \
        --set "*.ssh=default" \
        --set "*.platform=$platform" \
        --set "*.args.ARCH=${arch}" \
        --set "simulator-visualizer.tags=ghcr.io/autowarefoundation/openadkit.autoware:simulator-visualizer-${arch}" \
        --set "planning-control-fail.tags=ghcr.io/autowarefoundation/openadkit.autoware:planning-control-fail-${arch}" \
        --set "planning-control-pass.tags=ghcr.io/autowarefoundation/openadkit.autoware:planning-control-pass-${arch}" \
        "${targets[@]}"
    set +x
}

# Remove dangling images
remove_dangling_images() {
    docker image prune -f
}

# Main script execution
parse_arguments "$@"
set_platform
clone_repositories
build_images
remove_dangling_images
