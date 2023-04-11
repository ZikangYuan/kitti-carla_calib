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

3. Adjust directory structure

Please make sure that the directory format of each sequence of KITTI-CARLA is as follow:

├── README.md 介绍软件及文档入口
├── bin 编译好的二进制文件,执行./build.sh自动生成，该目录也用于程序打包
├── build.sh 自动编译的脚本
├── doc 项目的文档
├── pack 打包后的程序放在此处
├── pack.sh 自动打包的脚本，生成类似xxxx.20170713_14:45:35.tar.gz的文件，放在pack文件下
└── src 项目的源代码
    ├── api           接入管理
    ├── node          节点管理
    ├── consensus     共识机制
    ├── core          核心功能
        ├── account     账户
        ├── asset       资产
        ├── blockchain  区块链
        ├── transaction 交易
        ├── validator   验证
        ├── witness     证明
        └── state       状态

Seuqnce  
└──────generated  
       ├────────full_poses_lidar.txt  
       ├────────full_ts_camera.txt        
       └────────frames       
                 ├────frame_0000.ply                 
                 ├────frame_0001.ply                
                 ├────......                 
                 └────frame_4999.ply  
                 
Please make sure that the directpry format of ouput result of each sequence is as follow:

Output  
├─────groundtruth.txt  
└─────correct

where "correct" is the directpry for saving calibrated LiDAR sweeps, and "groundtruth.txt" is the groundtruth pose which corresponds one to one with the calibrated LiDAR sweeps.
