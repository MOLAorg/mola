###########################################################################
#   THE KITTI VISION BENCHMARK SUITE: VISUAL ODOMETRY / SLAM BENCHMARK    #
#              Andreas Geiger    Philip Lenz    Raquel Urtasun            #
#                    Karlsruhe Institute of Technology                    #
#                Toyota Technological Institute at Chicago                #
#                             www.cvlibs.net                              #
###########################################################################

This file describes the KITTI visual odometry / SLAM benchmark package.
Accurate ground truth (<10cm) is provided by a GPS/IMU system with RTK
float/integer corrections enabled. In order to enable a fair comparison of
all methods, only ground truth for the sequences 00-10 is made publicly
available. The remaining sequences (11-21) serve as evaluation sequences.

NOTE: WHEN SUBMITTING RESULTS, PLEASE STORE THEM IN THE SAME DATA FORMAT IN
WHICH THE GROUND TRUTH DATA IS PROVIDED (SEE 'POSES' BELOW), USING THE
FILE NAMES 11.txt TO 21.txt. CREATE A ZIP ARCHIVE OF THEM AND STORE YOUR
RESULTS IN ITS ROOT FOLDER.

File description:
=================

Folder 'sequences':

Each folder within the folder 'sequences' contains a single sequence, where
the left and right images are stored in the sub-folders image_0 and
image_1, respectively. The images are provided as greyscale PNG images and
can be loaded with MATLAB or libpng++. All images have been undistorted and
rectified. Sequences 0-10 can be used for training, while results must be
provided for the test sequences 11-21.

Additionally we provide the velodyne point clouds for point-cloud-based
methods. To save space, all scans have been stored as Nx4 float matrix into
a binary file using the following code:

  stream = fopen (dst_file.c_str(),"wb");
  fwrite(data,sizeof(float),4*num,stream);
  fclose(stream);

Here, data contains 4*num values, where the first 3 values correspond to
x,y and z, and the last value is the reflectance information. All scans
are stored row-aligned, meaning that the first 4 values correspond to the
first measurement. Since each scan might potentially have a different
number of points, this must be determined from the file size when reading
the file, where 1e6 is a good enough upper bound on the number of values:

  // allocate 4 MB buffer (only ~130*4*4 KB are needed)
  int32_t num = 1000000;
  float *data = (float*)malloc(num*sizeof(float));

  // pointers
  float *px = data+0;
  float *py = data+1;
  float *pz = data+2;
  float *pr = data+3;

  // load point cloud
  FILE *stream;
  stream = fopen (currFilenameBinary.c_str(),"rb");
  num = fread(data,sizeof(float),num,stream)/4;
  for (int32_t i=0; i<num; i++) {
    point_cloud.points.push_back(tPoint(*px,*py,*pz,*pr));
    px+=4; py+=4; pz+=4; pr+=4;
  }
  fclose(stream);

x,y and y are stored in metric (m) Velodyne coordinates.

IMPORTANT NOTE: Note that the velodyne scanner takes depth measurements
continuously while rotating around its vertical axis (in contrast to the cameras,
which are triggered at a certain point in time). This effect has been
eliminated from this postprocessed data by compensating for the egomotion!!
Note that this is in contrast to the raw data.

The relationship between the camera triggers and the velodyne is the following:
We trigger the cameras when the velodyne is looking exactly forward (into the
direction of the cameras). After compensation, the point cloud data should
correspond to the camera data for all static elements of the scene. Dynamic
ones are still slightly distorted, of course. If you want the raw velodyne
scans, please have a look at the section 'mapping to raw data' below.

The base directory of each folder additionally contains:

calib.txt: Calibration data for the cameras: P0/P1 are the 3x4 projection
matrices after rectification. Here P0 denotes the left and P1 denotes the
right camera. Tr transforms a point from velodyne coordinates into the
left rectified camera coordinate system. In order to map a point X from the
velodyne scanner to a point x in the i'th image plane, you thus have to
transform it like:

  x = Pi * Tr * X

times.txt: Timestamps for each of the synchronized image pairs in seconds,
in case your method reasons about the dynamics of the vehicle.

Folder 'poses':

The folder 'poses' contains the ground truth poses (trajectory) for the
first 11 sequences. This information can be used for training/tuning your
method. Each file xx.txt contains a N x 12 table, where N is the number of
frames of this sequence. Row i represents the i'th pose of the left camera
coordinate system (i.e., z pointing forwards) via a 3x4 transformation
matrix. The matrices are stored in row aligned order (the first entries
correspond to the first row), and take a point in the i'th coordinate
system and project it into the first (=0th) coordinate system. Hence, the
translational part (3x1 vector of column 4) corresponds to the pose of the
left camera coordinate system in the i'th frame with respect to the first
(=0th) frame. Your submission results must be provided using the same data
format.

Mapping to Raw Data
===================

Note that this section is additional to the benchmark, and not required for
solving the object detection task.

In order to allow the usage of the laser point clouds, gps data, the right
camera image and the grayscale images for the TRAINING data as well, we
provide the mapping of the training set to the raw data of the KITTI dataset.
The following table lists the name, start and end frame of each sequence that
has been used to extract the visual odometry / SLAM training set:

Nr.     Sequence name     Start   End
---------------------------------------
00: 2011_10_03_drive_0027 000000 004540
01: 2011_10_03_drive_0042 000000 001100
02: 2011_10_03_drive_0034 000000 004660
03: 2011_09_26_drive_0067 000000 000800
04: 2011_09_30_drive_0016 000000 000270
05: 2011_09_30_drive_0018 000000 002760
06: 2011_09_30_drive_0020 000000 001100
07: 2011_09_30_drive_0027 000000 001100
08: 2011_09_30_drive_0028 001100 005170
09: 2011_09_30_drive_0033 000000 001590
10: 2011_09_30_drive_0034 000000 001200

The raw sequences can be downloaded from
http://www.cvlibs.net/datasets/kitti/raw_data.php
in the respective category (mostly: Residential).

Evaluation Code:
================

For transparency we have included the KITTI evaluation code in the
subfolder 'cpp' of this development kit. It can be compiled via:

g++ -O3 -DNDEBUG -o evaluate_odometry evaluate_odometry.cpp matrix.cpp

