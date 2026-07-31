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

The semantic condition accepts `cost_mode:=fuzzy`, `fixed`, or `lethal`.
`fuzzy` is the proposed policy-aware soft-cost method, `fixed` removes policy
inference while retaining risk-scaled soft costs, and `lethal` is the hard
obstacle comparison. Every report records the selected mode; the repeated
report summarizer rejects mixed modes.

`mask_source:=file` is the default. `mask_source:=topic` converts the same
trinary mask into a `nav_msgs/OccupancyGrid`, waits for the semantic layer
subscriber, and publishes it with reliable, transient-local QoS. This validates
the live topic transport without starting a camera or detector.

`generator_planner_ab.launch.py` goes one step further: a deterministic
synthetic CameraInfo, registered 16UC1 depth image, and object Detection2D pass
through the real `semantic_mask_generator`. The runner consumes that external
topic and records the generated mask geometry before planning.

Synthetic scenario arguments cover depth, seeded depth noise, invalid-depth
fraction, random seed, confidence, exact class label, detection count and
spacing, image position, detection-frame stride, timestamp offset, and
semantic-radius scale. The runner records all values in
`mask_producer_runtime_parameters`, so the comparison fingerprint rejects
trials with different injections.

Select one of the configured warehouse risk labels for a controller-free
class-profile check:

```bash
ros2 launch semantic_planning_experiments generator_planner_ab.launch.py \
  domain_id:=159 synthetic_detection_class_id:=fragile_box \
  output_path:=/tmp/semantic_fragile_box_ab.json
```

This only synthesizes a detector result. It does not prove that an
off-the-shelf detector recognizes `forklift`, `pallet`, or `fragile_box`;
those labels require matching custom detector output.

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

Summarize two or more repeated reports:

```bash
ros2 run semantic_planning_experiments semantic_planning_summary \
  --output /tmp/semantic_repeatability_summary.json \
  /tmp/semantic_repeatability_01.json \
  /tmp/semantic_repeatability_02.json \
  /tmp/semantic_repeatability_03.json
```

The summarizer rejects dirty revisions and mismatched code, configuration,
route, source, or policy inputs. It reports metric mean, population standard
deviation, range, and exact path SHA-256 repeatability for each condition.

Validate and expand the paper pilot manifest without executing any launch:

```bash
ros2 run semantic_planning_experiments semantic_experiment_plan \
  src/semantic_planning_experiments/config/paper_pilot_manifest.yaml \
  --output /tmp/semantic_paper_pilot_plan.json
```

The plan records research questions, factors, repetitions, numeric acceptance
criteria, isolated domains, output paths, and command argument arrays. Only the
two controller-free experiment launch files are allowed. The planner does not
execute generated commands.

Generate the method-ablation plan:

```bash
ros2 run semantic_planning_experiments semantic_experiment_plan \
  src/semantic_planning_experiments/config/paper_ablation_manifest.yaml \
  --output /tmp/semantic_paper_ablation_plan.json
```

It expands fixed soft cost, lethal obstacle, and proposed fuzzy cost into nine
controller-free trials. Each trial also records the embedded disabled semantic
baseline.

Two additional plans cover scenario diversity and bounded perception
perturbations:

```bash
ros2 run semantic_planning_experiments semantic_experiment_plan \
  src/semantic_planning_experiments/config/paper_scenario_matrix_manifest.yaml \
  --output /tmp/semantic_paper_scenario_matrix_plan.json

ros2 run semantic_planning_experiments semantic_experiment_plan \
  src/semantic_planning_experiments/config/paper_robustness_manifest.yaml \
  --output /tmp/semantic_paper_robustness_plan.json
```

Each expands six scenarios into 18 planned trials without executing them.

Generate the warehouse object-class profile plan:

```bash
ros2 run semantic_planning_experiments semantic_experiment_plan \
  src/semantic_planning_experiments/config/paper_object_class_manifest.yaml \
  --output /tmp/semantic_paper_object_class_plan.json
```

It expands `person`, `forklift`, `pallet`, and `fragile_box` into 12 planned
trials. These synthetic labels test the downstream risk profiles only; they do
not measure detector precision or recall.

After the planned reports exist, audit every numeric criterion:

```bash
ros2 run semantic_planning_experiments semantic_experiment_evaluate \
  /tmp/semantic_paper_pilot_plan.json \
  --output /tmp/semantic_paper_pilot_evaluation.json
```

The evaluator exits successfully only when all trials pass. Failed, missing,
invalid, dirty-revision, and unsafe-scope reports remain visible in the JSON
audit and cause a nonzero exit.

Reports default to `/tmp` and must not be committed as claimed research
results until the selected map, mask, and experiment region have been
validated.
