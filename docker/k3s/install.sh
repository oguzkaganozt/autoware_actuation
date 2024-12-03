#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/../.."

# install docker environment
"$WORKSPACE_ROOT/setup_dev_env.sh" --no-nvidia docker

# install k3s
curl -sfL https://get.k3s.io | sh

# configure interface
"$SCRIPT_DIR/configure_interface.sh"


