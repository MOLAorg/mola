# mola_traj_tools
CLI tools to manipulate trajectory files as a complement to [evo](https://github.com/MichaelGrupp/evo).

<!-- toc -->

- [Build and install](#build-and-install)
- [Documentation](#documentation)
- [License](#license)

<!-- tocstop -->

## Build and install
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola) for compilation instructions.

To install from the ROS repositories:

    sudo apt install ros-${ROS_DISTRO}-mola-traj-tools

## Documentation

### traj_ypr2tum

This tool can be used to convert a TXT file with a trajectory in this format:

```
# t [unix timestamp, double]  x y z [meters]  yaw pitch roll [radians]
t x y z yaw pitch roll
```

into the [TUM format](https://github.com/MichaelGrupp/evo/wiki/Formats#tum---tum-rgb-d-dataset-trajectory-format).

Usage:

```bash
  traj_ypr2tum INPUT.ypr OUTPUT.tum
```

### traj_tf

This tool takes an **input** trajectory file in the [TUM format](https://github.com/MichaelGrupp/evo/wiki/Formats#tum---tum-rgb-d-dataset-trajectory-format),
a SE(3) **transformation**, and applies it to the input, writing the modified trajectory to an **output** file.

Usage:

```bash
  traj_tf INPUT.tum OUTPUT.tum "[x y z yaw_deg pitch_deg roll_deg]"
```


## License
This package is released under the BSD-3-clause license.
