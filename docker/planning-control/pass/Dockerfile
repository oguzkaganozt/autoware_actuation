FROM ghcr.io/autowarefoundation/autoware:universe AS planning-control-pass

# Copy launch file
COPY docker/planning-control/pass/pass_scenario.launch.xml /opt/autoware/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml

# Copy entrypoint script
COPY docker/etc/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
