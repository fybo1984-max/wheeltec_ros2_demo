# Semantic costmap plugin (ROS 2 Humble port)

This package is the static-mask proof of concept for semantic-aware WHEELTEC
navigation. It ports the core `concres/semantic_costmap_plugin` layer to the
ROS 2 Humble/Nav2 API used by this workspace.

The layer reads a normal Nav2 map YAML and treats occupied mask pixels as
traversable semantic zones. It applies a Sugeno fuzzy cost controlled by:

- `task_urgency` (`0..10`): higher values make crossing a zone cheaper;
- `avoidance_level` (`0..100`): higher values make crossing a zone costlier.

The mask cost is a soft cost (`0..252`), not a lethal obstacle. The existing
Nav2 planner and controller therefore remain unchanged.

## WHEELTEC integration

The current `senior_akm` Nav2 configuration contains the layer but leaves it
disabled. Use the innovation launch with:

```bash
ros2 launch largemodel largemodel_control.launch.py \
  use_nav:=true \
  semantic_costmap_enabled:=true \
  semantic_task_urgency:=0 \
  semantic_avoidance_level:=70.0
```

This launch starts robot navigation and can move the vehicle. Follow the
workspace safety procedure before running it. Build and unit tests do not start
hardware or publish velocity commands.

The bundled mask is deliberately named `large_loop_semantic_mask_example`; its
zone is an experiment template, not a validated industrial safety boundary.

## Upstream

- Repository: <https://github.com/concres/semantic_costmap_plugin>
- Imported revision: `1a9c13b76df244555ad967f92345ca96700ba8e2`
- Upstream license: Apache-2.0
- Local changes: ROS 2 Humble compatibility, strict map-load and parameter
  validation, unknown/obstacle-preserving costmap merge, WHEELTEC
  configuration, and reduced behavior-focused tests.

See `SEMANTIC_NAVIGATION.md` at the workspace root for the staged integration
plan and the assessment of `e-cagan/semantic-slam-workspace`.
