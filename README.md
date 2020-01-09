# Hypertun-SR

## Requirements
 - OpenCV 2.4.13
 - boost (tested with 1.58.0.1)

The pipeline also uses the exFAST library, which is included in src/sparsestereo and the Triangle library, which is included in libs/triangulation.

### Datasets
Two KITTI datasets are used which should be contained in folders as:
 - Hypertun-SR/data/data_stereo_flow 		(http://www.cvlibs.net/datasets/kitti/eval_stereo_flow.php?benchmark=stereo)
 - Hypertun-SR/data/data_odometry_color		(http://www.cvlibs.net/datasets/kitti/eval_odometry.php, only the folder 00)

These two datasets can be toggled by changing the variable `int DATASET` in the main.cpp.

## Compiling and running
The code was already build on Ubuntu 16.04 using cmake. To run the pipeline use the executable located at src/main.out.

To build the code again, navigate into the src folder and use:
`cmake .`
and then
`make`