# Semantic planning A/B experiments

This package measures the effect of the semantic mask on Nav2 global planning.
It starts only `map_server`, `planner_server`, a lifecycle manager, and a fixed
experiment transform. It never starts a controller, BT navigator, localization,
sensor, or robot-base node and does not publish velocity commands.
The launch defaults to `ROS_DOMAIN_ID=73` and localhost-only discovery so it
does not join the normal robot ROS graph.

The two requests use the same map, start, goal, Hybrid-A* settings, and planner
process:

1. baseline with `mask_layer.enabled=false`;
2. semantic condition with `mask_layer.enabled=true`.

`mask_source:=file` is the default. `mask_source:=topic` converts the same
trinary mask into a `nav_msgs/OccupancyGrid`, waits for the semantic layer
subscriber, and publishes it with reliable, transient-local QoS. This validates
the live topic transport without starting a camera or detector.

`generator_planner_ab.launch.py` goes one step further: a deterministic
synthetic CameraInfo, registered 16UC1 depth image, and person Detection2D pass
through the real `semantic_mask_generator`. The runner consumes that external
topic and records the generated mask geometry before planning.

The global costmap is cleared and allowed to update between conditions.
`cache_obstacle_heuristic` is disabled so the second result cannot reuse an
obstacle heuristic calculated for the baseline costmap.
After writing the report, the runner asks the lifecycle manager to shut down
the map and planner nodes cleanly.

Run after building and sourcing only this innovation workspace:

```bash
ros2 launch semantic_planning_experiments planner_ab.launch.py \
  output_path:=/tmp/semantic_planning_ab.json
```

Validate the dynamic topic input path:

```bash
ros2 launch semantic_planning_experiments planner_ab.launch.py \
  mask_source:=topic \
  domain_id:=74 \
  output_path:=/tmp/semantic_planning_ab_topic.json
```

Validate the generator-to-planner software chain:

```bash
ros2 launch semantic_planning_experiments generator_planner_ab.launch.py \
  output_path:=/tmp/semantic_generator_planning_ab.json
```

This launch uses ROS domain 75 and still does not start a controller, BT
navigator, localization, physical camera, detector, or robot-base node.

Override `task_urgency` (`0..10`) and `avoidance_level` (`0..100`) to verify
the soft-cost policy. The runner applies both values through the costmap
parameter service before each condition and stores them in the report:

```bash
ros2 launch semantic_planning_experiments generator_planner_ab.launch.py \
  domain_id:=78 task_urgency:=10 avoidance_level:=0.0 \
  output_path:=/tmp/semantic_sweep_urgent.json
```

Verify that a temporary risk decays and the route recovers while the semantic
layer remains enabled:

```bash
ros2 launch semantic_planning_experiments generator_planner_ab.launch.py \
  domain_id:=82 \
  detection_active_duration_sec:=30.0 \
  observation_hold_sec:=0.5 \
  observation_decay_sec:=1.0 \
  recovery_timeout_seconds:=15.0 \
  output_path:=/tmp/semantic_decay_ab.json
```

With a positive `recovery_timeout_seconds`, the runner waits for an empty
external mask and records a third `recovered_after_mask_clear` path plus its
delta from the baseline.

Use `domain_id:=<unused ID>` if 73 is already in use by another local test.
Hybrid-A* builds its lookup table during startup, so the action client allows
up to 60 seconds for lifecycle activation on Jetson-class hardware.

The default route is `(6.0, 0.0) -> (20.0, 0.0)` on
`large_loop_final`. It crosses the example mask in the geometric baseline.
Override `start_x`, `start_y`, `start_yaw`, `goal_x`, `goal_y`, and `goal_yaw`
to test other routes.

The JSON report includes:

- code revision (with `-dirty` when applicable) and map/mask/planner-config
  SHA-256 digests;
- explicit start, goal, planner ID, mask source, topic mode, topic QoS, task
  urgency, and avoidance level;
- generated mask cell count, map-coordinate centroid and occupied bounds;
- complete baseline and semantic paths;
- planner time, path length, semantic crossing length and ratio;
- minimum path-to-semantic-boundary clearance;
- semantic-minus-baseline metric deltas.

Reports default to `/tmp` and must not be committed as claimed research
results until the selected map, mask, and experiment region have been
validated.
