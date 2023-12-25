# mola_input_kitti_dataset
CLI tool to evaluate the KITTI odometry benchmark metrics to arbitrary trajectory files,
in both, `kitti` and `tum` formats. Better used together with [evo](https://github.com/MichaelGrupp/evo).

<!-- toc -->

- [Build and install](#build-and-install)
- [Documentation](#documentation)
- [Examples of usage](#examples-of-usage)
  * [Evaluate the KITTI metrics on a solution by your SLAM method](#evaluate-the-kitti-metrics-on-a-solution-by-your-slam-method)
  * [Evaluate the KITTI metrics on another dataset](#evaluate-the-kitti-metrics-on-another-dataset)
  * [Transform a KITTI solution file in TUM format to KITTI](#transform-a-kitti-solution-file-in-tum-format-to-kitti)
- [License](#license)

<!-- tocstop -->

## Build and install
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola) for compilation instructions.

To install from the ROS repositories:

    sudo apt install ros-${ROS_DISTRO}-mola-metrics-eval

This program is largely based on the public Kitti dataset evaluation C++ code.
Rewritten to use Eigen instead of GNU GPL'd code, and to add additional features.
Original source code notice:

    ###########################################################################
    #   THE KITTI VISION BENCHMARK SUITE: VISUAL ODOMETRY / SLAM BENCHMARK    #
    #              Andreas Geiger    Philip Lenz    Raquel Urtasun            #
    #                    Karlsruhe Institute of Technology                    #
    #                Toyota Technological Institute at Chicago                #
    #                             www.cvlibs.net                              #
    ###########################################################################

## Documentation

Basically, this CLI program can be used for evaluating the [KITTI metrics](https://www.cvlibs.net/datasets/kitti/) in two ways:
- To **estimated trajectories of the KITTI odometry sequences**, stored in the [TUM format](https://github.com/MichaelGrupp/evo/wiki/Formats#tum---tum-rgb-d-dataset-trajectory-format) (unlike the original KITTI dev kit, which uses the [kitti format](https://github.com/MichaelGrupp/evo/wiki/Formats#kitti---kitti-dataset-pose-format)). In this case, the program reads the **ground truth sequences** and the **calibration files** from a user's local copy of the KITTI dataset. Calibration data is used to first transform the user's input trajectory to the `cam0` frame, in which the KITTI ground truth paths are given.
- To **any other pair of trajectory files** for other datasets, i.e. a ground truth and an estimated trajectory file.

```bash
USAGE: 

   kitti-metrics-eval  [--no-figures] [--gt-tum-path <trajectory_gt.txt>]
                       [-s <01>] ...  [--save-as-kitti <result.kitti>] -r
                       <result.txt> [-k <>] [--] [--version] [-h]


Where: 

   --no-figures
     Skip generating the error figures

   --gt-tum-path <trajectory_gt.txt>
     If provided, the --sequence flag will be ignored and this particular
     file in TUM format will be read and used as ground truth to compare
     against the resulting odometry path.

   -s <01>,  --sequence <01>  (accepted multiple times)
     The sequence number of the path(s) file(s) to evaluate, used to find
     out GT and calibration files for the Kitti dataset.

   --save-as-kitti <result.kitti>
     If given, will transform the input path from the LIDAR frame to the
     cam0 frame and save the path to a TXT file in the format expected by
     KITTI dev kit.

   -r <result.txt>,  --result-tum-path <result.txt>
     (required)  File to evaluate, in TUM format

   -k <>,  --kitti-basedir <>
     Path to the kitti datasets. Overrides to the default, which is reading
     the env var `KITTI_BASE_DIR`.

   --,  --ignore_rest
     Ignores the rest of the labeled arguments following this flag.

   --version
     Displays version information and exits.

   -h,  --help
     Displays usage information and exits.
```

## Examples of usage

### Evaluate the KITTI metrics on a solution by your SLAM method

Evaluate on all KITTI test sequences 00-10.

```bash
kitti-metrics-eval \
    -r results/estim_%02i.txt \
    -s 00 -s 01 -s 02 -s 03 -s 04 -s 05 -s 06 -s 07 -s 08 -s 09 -s 10 \
    --kitti-basedir /path/to/your/local/kitti_dataset
```

Expected tree layout under the `kitti_dataset` directory:

```
/path/to/your/local/kitti_dataset
├── poses
└── sequences
```

### Evaluate the KITTI metrics on another dataset

Evaluate on another arbitrary dataset, with ground truth trajectory
given in the TUM format:

```bash
kitti-metrics-eval \
    -r results/my_solution.txt \
    --gt-tum-path /path/to/ground_truth.txt
```

### Transform a KITTI solution file in TUM format to KITTI

```bash
kitti-metrics-eval \
    -r results/estim_%02i.txt \
    -s 00 \
    --save-as-kitti results/00.txt
```

## License
This package is released under the BSD-3-clause license.
