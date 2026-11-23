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
    #max_time_offset: 0.010            # [s] to pair depth and intensity frames
    #is_optional: false                # skip the sensor if a topic is missing
    #max_range: 10.0                   # [m]
    #intensity_16bit_scale: 1.0        # mono16 intensity (e.g. infrared) into 8 bits
    #intensity_channel: visible        # or "ir"
    #fixed_sensor_pose: "x y z yaw_deg pitch_deg roll_deg"  # depth camera, +X forward
```

Depth and intensity topics may also be `sensor_msgs/CompressedImage`, as recorded by
`image_transport`: `compressedDepth` (PNG or RVL, 16UC1 or 32FC1) for the depth and
JPEG/PNG for the intensity. For example, a ZED camera, whose depth (32FC1) is
registered to the left color image and may have a lower resolution:

```yaml
  - topic: /zed/zed_node/depth/depth_registered/compressedDepth
    type: CObservation3DRangeScan
    sensorLabel: zed
    intensity_topic: /zed/zed_node/rgb/image_rect_color/compressed
    max_range: 20.0
```

Compressed topics and the `intensity_*` and `max_queue_length` options need
`mrpt_libros_bridge` >= 4.2.0, which builds the observations
(`mrpt::ros2bridge::RgbdAssembler`, shared with `rosbag2rawlog`, `BridgeROS2` and
`mola_input_rosbag1`). Points take their color from an intensity image of another
resolution than the depth image, and through its lens distortion, with MRPT
versions that define `MRPT_HAS_3DSCAN_INTENSITY_PROJECTION`.

Depth and intensity keep their own resolution and intrinsics
(`cameraParams`, `cameraParamsIntensity`), and `relativePoseIntensityWRTDepth`
comes from `/tf`. Depth already registered to the intensity camera (e.g. an
`aligned_depth_to_color` topic) shares its optical frame, so both cameras then
coincide. Depth and intensity frames whose stamps differ by more than
`max_time_offset` are dropped, with a periodic warning: raise it for cameras
whose streams are not hardware-synchronized. With
`use_bag_recv_time_as_timestamp`, an observation takes the receive time of the
image that completed its pair.

## Build and install
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola).

## Docs and examples
See this package page [in the documentation](https://docs.mola-slam.org/latest/modules.html).

## License
This package is released under the GNU GPL v3 license. Other options available upon request.
