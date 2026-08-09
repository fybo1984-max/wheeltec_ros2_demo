# Warehouse person detector

This package converts Ultralytics detections to
`vision_msgs/msg/Detection2DArray` for `semantic_mask_generator`. The default
contract subscribes to Astra color images on `/camera/color/image_raw`, keeps
only the standard `person` class at confidence 0.55 or higher, and publishes
`/detections`. Image timestamps and `camera_color_optical_frame` metadata are
preserved for registered-depth projection. The formal defaults explicitly use
an Ultralytics input size of 640 and PyTorch FP32 inference (`half:=false`).

The launch file does not start the camera, Nav2, or the robot:

```bash
ros2 launch ultralytics_ros2 yolo.launch.py \
  model:=/absolute/path/to/yolo11n.pt \
  device:=0
```

Model weights are intentionally ignored by Git. Record the exact model path
and SHA-256 in each experiment unit's metadata. Before launching, confirm that
the active ROS Python environment can import the Jetson-compatible
`ultralytics` package:

```bash
python3 -c "import ultralytics; print(ultralytics.__version__)"
```

After the approved camera and this detector are running, use
`semantic_live_input_readiness` before starting Rosbag collection.
