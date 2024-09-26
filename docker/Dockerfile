ARG PLATFORM
FROM ghcr.io/autowarefoundation/autoware:universe-devel-${PLATFORM} AS aws-reinvent-simulator-monolithic-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

COPY src /autoware/src

# Set up rosdep
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && rosdep update \
  && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache

# Build simulator
RUN  source /opt/ros/"$ROS_DISTRO"/setup.bash \
&& source /opt/autoware/setup.bash \
&& colcon build --cmake-args \
  " -Wno-dev" \
  " --no-warn-unused-cli" \
  --install-base /opt/autoware \
  --merge-install \
  --mixin release compile-commands ccache \
  --base-paths /autoware/src/simulator

RUN find /opt/autoware/lib -type f -name "*.py" -exec chmod +x {} \;
RUN find /opt/autoware/share -type f -name "*.py" -exec chmod +x {} \;

FROM ghcr.io/autowarefoundation/autoware:universe-${PLATFORM} AS aws-reinvent-simulator-monolithic

# Copy simulator from simulator-devel
COPY src /autoware/src
COPY --from=aws-reinvent-simulator-monolithic-devel /opt/autoware /opt/autoware

# Set up rosdep
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && rosdep update \
  && cd /autoware \
  && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache

RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

# Create entrypoint
COPY docker/etc/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
