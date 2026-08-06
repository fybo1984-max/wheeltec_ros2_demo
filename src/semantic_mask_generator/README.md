# Dynamic semantic mask generator

This ROS 2 Humble package converts registered RGB-D object detections and
configured ArUco marker poses into a map-aligned `nav_msgs/OccupancyGrid` risk
mask.

It deliberately does not start a camera, YOLO, Nav2, or robot hardware. The
node is disabled by default and will also reject detections until
`depth_is_registered:=true` explicitly confirms that depth pixels are aligned
with the color image used by the detector.

The default warehouse profile accepts exact detector labels for `person`,
`other_vehicle`, `pallet_load`, `temporary_cargo`, and `fragile_goods`. Their
initial soft-risk value/radius pairs are `100/1.2 m`, `95/1.8 m`, `65/0.8 m`,
`60/0.8 m`, and `85/1.0 m`.
These are experiment parameters rather than trained recognition claims and
must be calibrated for the deployment site. Repeated observations are
spatially merged, held briefly, and then linearly decay so stale positions
disappear. Persistent fragile-goods and loading zones remain in the static
semantic mask.

Only `person` is a standard COCO label commonly available in off-the-shelf
YOLO models. The other labels require a detector trained for those exact
warehouse classes or an upstream label adapter. This repository does not
contain or download model weights.

`marker_enabled:=true` adds a second input on
`/aruco_marker_publisher/markers`. The configured `marker_ids` and
`marker_labels` map marker poses to existing risk profiles; the default mapping
is marker `101` to `fragile_goods`. Marker poses are transformed directly into
the map frame, so they do not require a synthetic bounding box or a second
depth lookup. Unknown IDs and low-confidence markers are ignored. The marker
input is disabled by default.

For a marker-only software check, enable the node and marker input while
leaving registered RGB-D confirmation false:

```bash
ros2 launch semantic_mask_generator dynamic_semantic_mask.launch.py \
  enabled:=true marker_enabled:=true
```

This command expects an existing map, TF tree, and marker publisher; it does
not start any of them.

For the physical fragile-goods experiment, the Astra-specific launch connects
an already running color stream to `aruco_ros` and enables marker input in the
mask generator:

```bash
ros2 launch semantic_mask_generator fragile_marker_input.launch.py \
  enabled:=true marker_size_m:=0.15
```

It expects an ArUco original-dictionary marker with ID `101`, an existing
`/map` topic, and a valid transform from `map` to
`camera_color_optical_frame`. The `marker_size_m` value is the measured outer
black-square side length; it must match the printed marker. This launch does
not start the camera, localization, Nav2, a controller, or robot hardware.

The optional loading-zone input uses a map-frame polygon from
`loading_zone_vertices_m` and a `std_msgs/Bool` state on
`/semantic/loading_zone_active`. When enabled, `true` overlays the configured
soft risk value and `false` removes the zone on the next mask publication. The
default polygon is a placeholder and must be replaced with measured experiment
coordinates before enabling this input.

`risk_value_scale` and `risk_radius_scale` apply validated positive
multipliers to every configured class value and radius. They are intended for
controlled paper scenarios; both operational defaults are `1.0`. A scaled
value must remain in `[1, 100]`.

The output values are semantic risk intensities in `[0, 100]`. Configure
`semantic_costmap_plugin::MaskLayer` with `mask_source: topic` and
`mask_topic: /semantic_mask` to map these intensities into Nav2 soft costs.

The controller-free integration test in `semantic_planning_experiments`
publishes deterministic synthetic RGB-D and Detection2D messages through this
node, then compares Nav2 paths with the resulting topic mask disabled and
enabled. The source supports a selected warehouse class, seeded fixed depth
noise, invalid-depth pixels, multiple detections, detection-frame loss,
timestamp offset, confidence variation, and risk-radius scaling. It does not
require a camera, detector, or robot base.

See the workspace-level `SEMANTIC_NAVIGATION.md` before using live sensors or
starting navigation.
