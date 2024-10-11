ARG PLATFORM
FROM ghcr.io/autowarefoundation/autoware:universe-devel-20240927 AS aws-reinvent-simulator-devel
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

FROM ghcr.io/autowarefoundation/autoware:universe-20240927 AS aws-reinvent-simulator

# Copy simulator from simulator-devel
COPY src /autoware/src
COPY --from=aws-reinvent-simulator-devel /opt/autoware /opt/autoware

# Set up rosdep
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && apt-get install -y curl unzip \
  && rosdep update \
  && cd /autoware \
  && rosdep install -y --from-paths src --ignore-src --dependency-types=exec --rosdistro $ROS_DISTRO \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache

RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

# Install AWS CLI
RUN curl -fsSL https://awscli.amazonaws.com/awscli-exe-linux-$(uname -m).zip -o /tmp/aws-cli.zip && \
  unzip -q -d /tmp /tmp/aws-cli.zip && \
  /tmp/aws/install && \
  rm -rf /tmp/aws && \
  echo "AWS CLI installed. Checking configuration" && \
  aws --version

# Copy simulation scenario and map
COPY docker/etc/simulation /autoware/scenario-sim

# Create entrypoint
COPY docker/etc/aws_entrypoint.sh /aws_entrypoint.sh
RUN chmod +x /aws_entrypoint.sh
ENTRYPOINT ["/aws_entrypoint.sh"]
CMD ["/bin/bash"]

FROM ghcr.io/autowarefoundation/autoware:universe-20240927 AS aws-reinvent-planning-control

RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && apt-get install -y curl unzip

# Install AWS CLI
RUN curl -fsSL https://awscli.amazonaws.com/awscli-exe-linux-$(uname -m).zip -o /tmp/aws-cli.zip && \
  unzip -q -d /tmp /tmp/aws-cli.zip && \
  /tmp/aws/install && \
  rm -rf /tmp/aws && \
  echo "AWS CLI installed. Checking configuration" && \
  aws --version

# Create entrypoint
COPY docker/etc/aws_entrypoint.sh /aws_entrypoint.sh
RUN chmod +x /aws_entrypoint.sh
ENTRYPOINT ["/aws_entrypoint.sh"]
CMD ["/bin/bash"]
