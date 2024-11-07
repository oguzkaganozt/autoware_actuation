# Install all RViz plugins from autoware.universe
# Install all message types of autoware.universe
FROM ghcr.io/autowarefoundation/autoware:universe-devel AS builder

# Build tierIV simulator
COPY simulator.repos simulator.repos
RUN vcs import src < simulator.repos

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

# simulator-visualizer
FROM ghcr.io/autowarefoundation/autoware:universe AS simulator-visualizer
COPY simulator.repos simulator.repos
RUN vcs import src < simulator.repos
COPY --from=builder /opt/autoware /opt/autoware

# Set up rosdep
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && apt-get install -y curl unzip \
  && rosdep update \
  && cd /autoware \
  && rosdep install -y --from-paths src --ignore-src --dependency-types=exec --rosdistro $ROS_DISTRO \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache

RUN echo "source /opt/autoware/setup.bash" >> /etc/bash.bashrc

# Install VNC server with NoVNC
RUN apt-get update && apt-get install -y \
    tigervnc-standalone-server \
    tigervnc-common \
    novnc \
    x11vnc \
    xvfb \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set up NoVNC according to https://github.com/novnc/noVNC/blob/master/docs/EMBEDDING.md

# Set up VNC password
RUN mkdir -p ~/.vnc && \
    echo "password" | vncpasswd -f > ~/.vnc/passwd && \
    chmod 600 ~/.vnc/passwd

# Set up startup script for VNC
COPY scripts/start-vnc.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/start-vnc.sh

# Expose VNC and NoVNC ports
EXPOSE 5900 6080
