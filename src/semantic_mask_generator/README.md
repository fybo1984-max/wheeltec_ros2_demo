# Dynamic semantic mask generator

This ROS 2 Humble package converts registered RGB-D object detections into a
map-aligned `nav_msgs/OccupancyGrid` risk mask.

It deliberately does not start a camera, YOLO, Nav2, or robot hardware. The
node is disabled by default and will also reject detections until
`depth_is_registered:=true` explicitly confirms that depth pixels are aligned
with the color image used by the detector.

The default risk profile handles `person` detections with a 1.2 m circular
soft-risk region. Repeated observations are spatially merged, held briefly, and
then linearly decay so stale positions disappear. Persistent fragile-goods and
loading zones remain in the static semantic mask.

`risk_radius_scale` applies a validated positive multiplier to every configured
class radius. It is intended for controlled paper scenarios; the operational
default is `1.0`.

The output values are semantic risk intensities in `[0, 100]`. Configure
`semantic_costmap_plugin::MaskLayer` with `mask_source: topic` and
`mask_topic: /semantic_mask` to map these intensities into Nav2 soft costs.

The controller-free integration test in `semantic_planning_experiments`
publishes deterministic synthetic RGB-D and Detection2D messages through this
node, then compares Nav2 paths with the resulting topic mask disabled and
enabled. The source supports seeded fixed depth noise, invalid-depth pixels,
multiple people, detection-frame loss, timestamp offset, confidence variation,
and risk-radius scaling. It does not require a camera, detector, or robot base.

See the workspace-level `SEMANTIC_NAVIGATION.md` before using live sensors or
starting navigation.
