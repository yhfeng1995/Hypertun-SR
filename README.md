# Hypertun-SR

## Reference

This repo is an implementation of paper: [High-Performance and Tunable Stereo Reconstruction](https://arxiv.org/pdf/1511.00758.pdf)

The author of this paper has not provided source code. Johann Diep provides an unofficial implementment: [Original repo](https://gitlab.com/jdiep/high-performance-and-tunable-stereo-reconstruction). This repo is mostly based on Johann Diep's work and performs several slight changes.

Thanks Sudeep Pillai, Johann Diep and everyone who have make effect to this algorithm!

## Requirements

 - **OpenCV 2.4.13/3.3.0** (Original repo using 2.4.13 but I haven't install this version, exFast library cannot be called successfully due to the change of `cv::FeatureDetector` interface in OpenCV3, so I use default FAST feature detector in OpenCV as replacement. For Feature-based stereo SLAM system, you can also use the match provided in Frame construction. )

 - boost (tested with 1.58.0.1)

The pipeline also uses the exFAST library, which is included in `ThirdParty/sparsestereo` and the Triangle library, which is included in `ThirdParty/triangulation`.

## Datasets
Two KITTI datasets are used which should be contained in folders as:

 - Hypertun-SR/data/data_stereo_flow 		(http://www.cvlibs.net/datasets/kitti/eval_stereo_flow.php?benchmark=stereo)

 - Hypertun-SR/data/data_odometry_color		(http://www.cvlibs.net/datasets/kitti/eval_odometry.php, only the folder 00)

These two datasets can be toggled by changing the variable `int DATASET` in the main.cpp.

## Compiling and running

The code was already build on Ubuntu 16.04 using cmake. To run the pipeline use the executable located at `test/bin/main.out`.

To build the code again, navigate into the project source folder and use:
`cmake .`
and then
`make`

## Efficiency Improvements

The efficiency of the original repo has great room for improvement. We implement several changes to accelerate the pipeline. 

The original repo running at 16Hz with 2 iterations (in [Report](./paper/3DVision_Report_Group3.pdf)), we have improved it up to around 30Hz, which is close to the data provided by original paper (34Hz).

However, since we focus more on the efficiency, the accuracy is still lower than the original paper present, same as original repo.

- Extracting high gradient pixels and preserving the coordinates simultaneously.

- Using [Scanline Algorithm](http://www.sunshine2k.de/coding/java/TriangleRasterization/TriangleRasterization.html#algo2) to allocate triangulates' indexs to all pixels.

## TODO

We haven't make improvements to the accuracy of this repo, so there still remain a great gap between the results provided by the paper and by the implementation.

Currently this repo's results is as follow: 

![](./paper/result.png)
