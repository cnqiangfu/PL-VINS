# PL-VINS: Real-Time Monocular Visual-Inertial SLAM with Point and Line

This system can run higher accuracy than VINS-Mono at the same work frequency on a low-power CPU Intel Core i7-10710U @1.10 GHz. 

Thank Jialong Wang (湖南大学大机器人与视觉感知国家工程实验室研三) for helping me code this system, he makes huge contribution in this work. 

**Note that**: Mr Wang is really a SLAM enthusiast and looking for a related job. If you can provides a nice **Job Opportunity**, please contact Jialong Wang <slamdragon@qq.com > .  

**DemoShow**: [youtube](https://youtu.be/IV5QEfI_MFc) or [bilibili](https://www.bilibili.com/video/BV1464y1F7hk/)

This respository is an initial version, it will be improved further in the furture.

## 1. Prerequisites
1.1 **Ubuntu** and **ROS**

Ubuntu 18.04. ROS Kinetic, please google it.

1.2. **Dependency**

Eigen 3.3.4 + OpenCV 3.2+ Cere-solver: [Ceres Installation](http://ceres-solver.org/installation.html), remember to **sudo make install**.

## 2. Build PL-VINS on ROS
Clone the repository and catkin_make (# note that you will create a new workspace named *catkin_plvins*):
```
	mkdir -p ~/catkin_plvins/src    
	cd ~/catkin_plvins/
	catkin_make
	source devel/setup.bash
	echo $ROS_PACKAGE_PATH             # test you have created it successfully
	git clone https://github.com/cnqiangfu/PL-VINS.git
```
**Notice**: before the second catkin_make, you need to go through /PL-VINS/feature_tracker/CMakeLists.txt, see the sign **# Important** in the CMakeLists.txt, and modify two absolute paths for correctly finding (1) the modified LSD algorithm, and you also need make sure find (2) OpenCV 3.2

```	
	catkin_make
	source /devel/setup.bash
```

## 3.Run on EuRoC dataset

Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). 

We suggust you select difficult sequences to test.

run in the ~/catkin_plvins/
```
	roslaunch plvins_show_linepoint.launch
	rosbag play YOUR_PATH_TO_DATASET/MH_05_difficult.bag
```
or 
```
roslaunch plvins_estimator euroc_fix_extrinsic.launch        #This launch runs without loop
```

Now you run PL-VINS in the ROS RViZ. 

**Note that**: if you want obtain the trajectory and compare it to your method. Please modify the ouput paths: /PL-VINS/vins_estimator/src/visualization.cpp (trajectory without loop) and /PL-VINS/pose_graph/src/pose_graph.cpp (trajectory with loop)


## 4 Related Papers

- **PL-VINS: Real-Time Monocular Visual-Inertial SLAM with Point and Line**.

```
tbd
```

This paper is developed based on VINS-Mono [1], PL-VIO [2], and our previous work [3].
```
[1] Vins-mono: A robust and versatile monocular visual-inertial state estimator

[2] Pl-vio: Tightly-coupled monocular visual-inertial odometry using point and line features

[3] A robust RGB-D SLAM system with points and lines for low texture indoor environments
```

*If you think aforementioned works are useful for research, please cite them.*

## 5. Acknowledgements

Thank Dr. Qin Tong, Prof. Shen (VINS-Mono), Yijia He (PL-VIO), etc very much.

## 6. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

