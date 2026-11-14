# mola_input_rosbag2
Offline RawDataSource from ROS 2 bags

Provided MOLA modules:
* `Rosbag2Dataset`, type RawDataSourceBase.

CLI tools:
* `mola-sm-attach-from-rosbag2`: Adds observations from a bag (e.g. camera images, for visual place recognition)
  to the keyframes of an existing `.simplemap`. Each observation goes to the
  keyframe closest in time (`--max-age`, default 0.5 s), and each keyframe keeps
  the closest one per topic, so none is stored twice. Images are written as
  jpg/png files to `<output>_Images/` (the external files of the input map are
  hard-linked there too); observations already in the map are not added again.
  
  ```bash
  mola-sm-attach-from-rosbag2 -i map.simplemap -o map_cam.simplemap \
    --rosbag2 dataset_dir --topic /camera/image_raw:camera_left
  ```
  
  `--topic` is repeatable (`TOPIC` or `TOPIC:LABEL`), one label per topic: each
  keyframe gets one observation per label. Use `--fixed-sensor-pose
  "x y z yaw pitch roll"` for bags without `/tf`, and `--image-format png` or
  `inline` to change how images are stored.

  The ROS 1 version is `mola-sm-attach-from-rosbag1` (package `mola_input_rosbag1`).
  ROS 2 bags may also need `--tf-topic` / `--tf-static-topic` (e.g. namespaced robots).

## RGB-D cameras
A `sensors` entry of `type: CObservation3DRangeScan` combines the topics of an
RGB-D camera into `mrpt::obs::CObservation3DRangeScan` observations:

```yaml
sensors:
  - topic: /camera/camera/depth/image_rect_raw   # 16UC1 (mm) or 32FC1 (m)
    type: CObservation3DRangeScan
    sensorLabel: rgbd
    intensity_topic: /camera/camera/color/image_raw   # optional
    #camera_info_topic: ...            # default: sibling "camera_info" topic
    #intensity_camera_info_topic: ...  # default: sibling "camera_info" topic
    #max_time_offset: 0.005            # [s] to pair depth and intensity frames
    #max_range: 10.0                   # [m]
    #fixed_sensor_pose: "x y z yaw_deg pitch_deg roll_deg"  # depth camera, +X forward
```

Depth and intensity keep their own resolution and intrinsics
(`cameraParams`, `cameraParamsIntensity`), and `relativePoseIntensityWRTDepth`
comes from `/tf`. Depth already registered to the intensity camera (e.g. an
`aligned_depth_to_color` topic) shares its optical frame, so both cameras then
coincide.

## Build and install
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola).

## Docs and examples
See this package page [in the documentation](https://docs.mola-slam.org/latest/modules.html).

## License
This package is released under the GNU GPL v3 license. Other options available upon request.
