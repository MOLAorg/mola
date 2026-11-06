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
  
  `--topic` is repeatable (`TOPIC` or `TOPIC:LABEL`). Use `--fixed-sensor-pose
  "x y z yaw pitch roll"` for bags without `/tf`, and `--image-format png` or
  `inline` to change how images are stored.

  The ROS 1 version is `mola-sm-attach-from-rosbag1` (package `mola_input_rosbag1`).
  ROS 2 bags may also need `--tf-topic` / `--tf-static-topic` (e.g. namespaced robots).

## Build and install
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola).

## Docs and examples
See this package page [in the documentation](https://docs.mola-slam.org/latest/modules.html).

## License
This package is released under the GNU GPL v3 license. Other options available upon request.
