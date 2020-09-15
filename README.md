# PL-VINS: Real-Time Monocular Visual-Inertial SLAM with Point and Line

This system can run higher accuracy than VINS-Mono at the same work frequency on a low-power CPU Intel Core i7-10710U @1.10 GHz. 

Thanks Jialong Wang (湖南大学大机器人与视觉感知国家工程实验室研三) for helping me code this system, he is good SLAM boy and looking for a related work. If you can provides a nice Job Opportunity, please contact Jialong Wang <slamdragon@qq.com > 

**DemoShow**: [youtube](https://youtu.be/IV5QEfI_MFc) or [bilibili](https://www.bilibili.com/video/BV1464y1F7hk/)

## 1. Prerequisites
1.1 **Ubuntu** and **ROS**

Ubuntu 18.04. ROS Kinetic, please google it.

1.2. **Dependency**

Eigen 3.3.4 + OpenCV 3.2+ Cere-solver: [Ceres Installation](http://ceres-solver.org/installation.html), remember to **sudo make install**.

## 2. Build PL-VINS on ROS
Creat your workspace:
Clone the repository and catkin_make:
```
	mkdir -p ~/catkin_plvins/src    # note that you will create a new workspace named catkin_plvins
	cd ~/catkin_plvins/
	git clone https://github.com/cnqiangfu/PL-VINS.git
```
**Notice**: before the catkin_make, (1) you need to go through /home/<your name>/catkin_plvins/src/PL-VINS/feature_tracker/CMakeLists.txt, see the sign # Important in the CMakeLists.txt, and modify two absolute paths for correctly finding the modified LSD algorithm. (2) and opencv 3.2*

```	
	catkin_make
	source /devel/setup.bash
```


## 3.Run on EuRoC dataset

Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).

run in the ~/catkin_plvins/
```
	roslaunch plvins_show_linepoint.launch
	rosbag play YOUR_PATH_TO_DATASET/MH_05_difficult.bag
```
or 
```
roslaunch plvins_estimator euroc_fix_extrinsic.launch 
```
*This launch runs without loop*



## 3 Related Papers

- **PL-VINS: Real-Time Monocular Visual-Inertial SLAM with Point and Line**.

```
tbd
```

This paper is developed based on VINS-Mono [1], PL-VIO [2], and our previsous work [3].

[1]Vins-mono: A robust and versatile monocular visual-inertial state estimator

[2]Pl-vio: Tightly-coupled monocular visual-inertial odometry using point and line features

[3]A robust RGB-D SLAM system with points and lines for low texture indoor environments

*If you think aforementioned works is useful for research, please cite them.*

## 5. Acknowledgements

Thanks Dr. Qin Tong, Prof. Shen (VINS-Mono), Yijia He (PL-VIO), etc very much.

## 6. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

