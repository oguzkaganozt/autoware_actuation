#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/.."

# Clone repositories
clone_repositories() {
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
clone_repositories
build_images
remove_dangling_images
