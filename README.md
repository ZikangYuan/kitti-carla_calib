# kitti-carla_calib

**This sourced code is for eliminating the motion distortion of LiDAR point cloud of [KITTI-CARLA dataset](https://npm3d.fr/kitti-carla).**

## Guide

1. Installation dependency

> GCC >= 5.4.0
>
> Cmake >= 3.0.2
> 
> [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) >= 3.2.8
>
> [PCL](https://pointclouds.org/downloads/) == 1.7 for Ubuntu 16.04, and == 1.8 for Ubuntu 18.04
>
> [OpenCV](https://opencv.org/releases/) == 2.4.9 for Ubuntu 16.04

2. Clone and build this repository
```
git clone https://github.com/ZikangYuan/kitti-carla_calib.git
cd kitti-calra_calib
mkdir include
mkdir lib
mkdir build
cd build
cmake ..
make
```
