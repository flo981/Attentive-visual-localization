# Attentive visual localization using feature and saliency fusion

Python based framework to extract ORB Feature, ORB Cloud from ORB SLAM 2 and saliency information from VOCUS 2. These are used to guide a stereo usb camera towards information-rich areas to improve ORB SLAM 2 localization and avoid tracking loss.

## Installation

python2.7
ros kinetic X

### ORB-SLAM 2:
Includes modifictaions in order extract features, current map points, pose and publish them to ros. Reference: https://github.com/raulmur/ORB_SLAM2

```
cd ORB_SLAM2
chmod +x build.sh
./build.sh

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS
chmod +x build_ros.sh
./build_ros.sh
```

### VOCUS2:
Saliency extraction (pixel precice). Inlcudes modifictaions for saliency map publishing
https://github.com/GeeeG/VOCUS2_ROS

```
cd packages_catkin
catkin build -j 1 --verbose

```
### Stereo Camera Launch
```
cd packages_catkin
catkin build -j 1

```
### Rosserial

```
http://wiki.ros.org/rosserial_python
```

## Run Attentive localization:
### 1. Stereo Camera
```
cd packages_catkin
source devel/setup.bash
rosrun image_transport_tutorial my_publisher
rosrun image_transport_tutorial my_subscriber

```
### 2. ORB SLAM 2
```
rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC3.yaml true
```

### 3.0 Attentive Localization

System settings: /attentive_localization/config.yaml

| ROS TOPIC | Description | Default |
| --- | --- | --- |
| `ORB_FEATURES` | ORB-SLAM 2 Features | "/orb_features_pixel" |
| `ORB_CLOUD` | ORB-SLAM 2 Point-Cloud | "/orb_cloud" |
| `ORB_POSE` | ORB-SLAM 2 Pose | "/stereo_pose" |
| `SALIENCY_PIXEL` | VOCUS 2 Saliency Map | "/saliency/image" |
| `SALIENCY_NORM` | VOCUS 2 Normalized Saliency Map | "/saliency/image_norm" |


| Attentive Localization | Description | Domain | Default |
| --- | --- | --- | --- |
| `X_BINS` | Number of bins in x-direction | ORB | 16
| `Y_BINS` | Number of bins in x-direction | ORB | 12
| `X_DIM` | Camera Resoluation (weight) | ORB, SAL, FUSION | 640
| `Y_DIM` | Camera Resoluation (height) | ORB, SAL, FUSION |480 
| `SCALE` | Density grid size: Downscaling from camera resolution (analogous to X_BINS, Y_BINS at ORB) | SAL, FUSION | 20
| `ADAP_QLEN` | Length of adaptive stability queue | ORB, SAL, Fusion | 10
| `ADAP_SPEED` | Decrease velocity of ADAP_QLEN (must be bigger than ADAP_QLEN) | ORB, SAL, Fusion | 12
| `PLOT_WEIGHT_MAT` | Plot weight matrix of current camera frame  | ORB, SAL, FUSION | False
| `ADAP_CAM_SPEED` | Depth-based camera actuation speed | ORB, SAL, FUSION | False
| `FIX_CAM_SPEED` | Constant camera actuation speed [1,4]| ORB, SAL, FUSION | 3
| `MAX_YAW` | Maximal allowed actuation angle in yaw-direction | ALL | 70
| `MAX_PITCH` | Maximal allowed actuation angle in pitch-direction | ALL" | 30
| `THRESHOLD` | Feature and Saliency Threshold/Weightning [0,1] | FUSION | 0.8
| `WEIGHT_MASK` | Center-sourround extraction of Opposite Feature Estimation | FUSION | 2

Select one of the approaches below:


### 3.1 ORB Feature-based
Attentive localization based on orb features (2d pixel based):
```
cd attentive_localization
python attentive_control_orb_2D.py
```
### 3.2 Saliency-based
Attentive localization based on saliency information (2d pixel based):

```
roslaunch visual_saliency saliency_vocus2.launch
cd attentive_localization
python attentive_control_saliency_2D.py
```
### 3.3 Feature and Saliency Fusion
Attentive localization based on or features AND saliency information (2d pixel based):

```
roslaunch visual_saliency saliency_vocus2.launch
cd attentive_localization
python attentive_control_orbfusion_2D.py
```

### 3.4 Pointcloud-based
Attentive localization based orb point cloud (3d world coordinates bases):
```
cd attentive_localization
python attentive_control_cloud_3D.py
```



## Table of Contents

- [Installation](#installation)
- [Features](#features)
- [Contributing](#contributing)
- [FAQ](#faq)
- [License](#license)
- 


# ROS Setup:

    export ROS_HOSTNAME=192.168.1.x
    export ROS_MASTER_URI=http://192.168.1.x:11311
    export ROS_IP=192.168.1.x
