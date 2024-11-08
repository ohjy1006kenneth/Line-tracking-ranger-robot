
# Line Tracking Ranger Robot

Summer Project for CATS Lab (UW-Madison) https://catslab.engr.wisc.edu/

Developed closed-loop control system for a modular autonomous robot platform, enabling the robot to detect lanes and navigate within them autonomously.

![A screenshot of a computer Description
automatically generated](./media/ranger.png)






## Achievements

Designed real-time lane and corner detection algorithm using OpenCV to accurately detect and follow lanes
* Utilized Hough transformation to detect left and right side of a lane (Green & Blue Line)
* Created algorithm to generate driving line (Yellow Line)
* Created algorithm to identify corner types (sharp turn, smooth turn)
![A screenshot of a computer Description
automatically generated](./media/image_processing.png)

Generated control signals and communicated with the robot using ROS2 platform.
* Designed and implemented a PID control system to maintain precise control over the robot’s movements
* Applied different diving modes and speeds for different corner types (Ackermann mode for smooth turns and Spin mode for sharp turns)
* Communicated with the robot using CAN communication protocol


