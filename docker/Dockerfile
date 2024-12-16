ARG PLATFORM
ARG ROS_DISTRO

# Build Stage
FROM ghcr.io/autowarefoundation/autoware:universe-devel AS builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV CCACHE_DIR="/root/.ccache"

COPY src /autoware/src

# Install dependencies and build the simulator
RUN --mount=type=ssh \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && \
    rosdep update && \
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && \
    source /opt/ros/"$ROS_DISTRO"/setup.bash && \
    source /opt/autoware/setup.bash && \
    colcon build --cmake-args \
      "-Wno-dev" \
      "--no-warn-unused-cli" \
      --install-base /opt/autoware \
      --merge-install \
      --mixin release compile-commands ccache \
      --base-paths /autoware/src/simulator && \
    find /opt/autoware/lib -type f -name "*.py" -exec chmod +x {} \; && \
    find /opt/autoware/share -type f -name "*.py" -exec chmod +x {} \; && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* "$HOME"/.cache

# Simulator
FROM ghcr.io/autowarefoundation/autoware:universe AS simulator
COPY src /autoware/src
COPY --from=builder /opt/autoware /opt/autoware
COPY docker/etc/simulation /autoware/scenario-sim
COPY docker/etc/aws_entrypoint.sh /aws_entrypoint.sh

RUN --mount=type=ssh \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && \
    apt-get install -y curl unzip && \
    rosdep update && \
    cd /autoware && \
    rosdep install -y --from-paths src --ignore-src --dependency-types=exec --rosdistro $ROS_DISTRO && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* "$HOME"/.cache && \
    echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc && \
    chmod +x /aws_entrypoint.sh

ENTRYPOINT ["/aws_entrypoint.sh"]
CMD ["/bin/bash"]

# Planning Control
FROM ghcr.io/autowarefoundation/autoware:universe AS planning-control
COPY docker/etc/aws_entrypoint.sh /aws_entrypoint.sh

RUN --mount=type=ssh \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && \
    apt-get install -y curl unzip && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* "$HOME"/.cache && \
    chmod +x /aws_entrypoint.sh

ENTRYPOINT ["/aws_entrypoint.sh"]
CMD ["/bin/bash"]
