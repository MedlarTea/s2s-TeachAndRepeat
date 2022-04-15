# Scan-to-Scan-based Teach and Repeat

This work is not done yet.

Introduction
---
It's a scan-to-scan-based 2d LiDAR teach and repeat. In teaching step, it will store the "keyscan" after certain distance. In repeating step, it will repeat the teaching path by ICP (scan-to-scan) pose estimation and wheel odometry information.

In teach step:
- Input: odometry, 2d scan, and joystick
- Output: keyscans

In repeat step: 
- Input: odometry, 2d scan, joystick and scans
- Output: cmd_vel

Hardware
---

Our sony joystick:

<img src="figs/sony-joystick.jpeg" width = "600" height = "480"  alt="sony-joystick" style="zoom:50%;">

Software
---
Some cpp packages
- Eigen
- OpenCV
- PCL

Here, we use ETH pointmatcher to accomplish ICP process for its high speed.
1. Follow [pointmatcher](https://github.com/MedlarTea/libpointmatcher) to install `pointmatcher`
2. Install `libpointmatcher_ros` to Connect ROS and PCL:
```bash
git clone https://github.com/MedlarTea/libpointmatcher
# then copy "libpointmatcher_ros" to your catkin_ws/src
```

Compile
---


Teach and Repeat
---


Observations
---

- Wheel odometry information is much precise than ICP pose estimation


Acknowledge
---

This project depends a lot on [libpointmatcher](https://github.com/MedlarTea/libpointmatcher) and [ethzasl_icp_mapping](https://github.com/ethz-asl/ethzasl_icp_mapping)

