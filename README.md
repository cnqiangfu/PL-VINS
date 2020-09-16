# PL-VINS: Real-Time Monocular Visual-Inertial SLAM with Point and Line

PL-VINS can yield higher accuracy than VINS-Mono (IROS best Paper, TRO Honorable Mention Best Paper) at the same work frequency on a low-power CPU Intel Core i7-10710U @1.10 GHz. It is an interesting thing we find that better CPU maybe yield better result whether VINS-Mono or PL-VINS.（Maybe the reason of ROS mechanism）

Thank Jialong Wang (湖南大学机器人视觉感知与控制技术国家工程实验室研三) for helping me code this system, he makes huge contribution in this work. **By the way**: Mr Wang is really a SLAM enthusiast and looking for a related job. If you can provide a nice **Job Opportunity**, please contact Jialong Wang <slamdragon@qq.com > . 


![image](https://github.com/cnqiangfu/PL-VINS/blob/master/support_files/plvins-vinsmono.png)


This respository is an initial version, it will be improved further in the coming months.


**DemoShow**: [youtube](https://youtu.be/IV5QEfI_MFc) or [bilibili](https://www.bilibili.com/video/BV1464y1F7hk/)

[![PL-VINS](https://img.youtube.com/vi/IV5QEfI_MFc/0.jpg)](https://youtu.be/IV5QEfI_MFc)

## 1. Prerequisites
1.1 **Ubuntu** and **ROS**

Ubuntu 18.04. ROS Melodic, please google it.

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
**Notice**: before the second catkin_make, you need to go through /PL-VINS/feature_tracker/CMakeLists.txt, see the sign **# Important** in the CMakeLists.txt, and modify two absolute paths to correctly find the modified LSD algorithm, and you also need to make sure OpenCV 3.2 there.

```	
	catkin_make
	source /devel/setup.bash
```

## 3. Run on EuRoC dataset

Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). We suggust you select difficult sequences to test.

run in the ~/catkin_plvins/
```
	roslaunch plvins_show_linepoint.launch
	rosbag play YOUR_PATH_TO_DATASET/MH_05_difficult.bag
```
or 
```
roslaunch plvins_estimator euroc_fix_extrinsic.launch        #This launch runs without loop
```

Now you should be able to run PL-VINS in the ROS RViZ. 

**Note that**: if you want obtain motion trajectory and compare it to your method. Please modify the ouput paths: /PL-VINS/vins_estimator/src/visualization.cpp (trajectory without loop) and /PL-VINS/pose_graph/src/pose_graph.cpp (trajectory with loop)


## 4. Related Papers

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

*If you think aforementioned works are useful for your research, please cite them.*

## 5. Acknowledgements

Thank Dr. Qin Tong, Dr. Peiliang Li, and Prof. Shen (VINS-Mono); Dr. Yijia He, Ji Zhao, Yue Guo, Wenhao He, and Kui Yuan(PL-VIO) very much.

## 6. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.1

