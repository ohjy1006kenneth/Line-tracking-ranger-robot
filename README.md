
# Line Tracking Ranger Robot

Summer Project for CATS Lab (UW-Madison) https://catslab.engr.wisc.edu/

Developed closed-loop control system for a modular autonomous robot platform, enabling it to detect lanes and navigate within them autonomously.

![A screenshot of a computer Description
automatically generated](./media/ranger.png)






## Achievements

Designed Real-Time Lane and Corner Detection Algorithm using OpenCV:
* Applied Hough transformation to identify left and right lane boundaries (green & blue line)
* Developed a custom algorithm to generate driving line (yellow line) and recognize corner types (sharp turn, smooth turn)
* Increased accuracy by pre-processing images with reverse-perspective and Gaussian blur

![A screenshot of a computer Description
automatically generated](./media/image_processing.png)

Generated Control Signals Using ROS2 Platform:
* Designed and implemented a PID control system to maintain precise control over the robot’s movements
* Programmed adaptive driving modes based on corner types, utilizing Ackermann steering for smooth turns and spin mode for sharp turns
* Integrated CAN communication protocol for reliable communication with the robot


