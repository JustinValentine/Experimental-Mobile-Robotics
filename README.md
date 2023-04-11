# 🤖 Experimental Mobile Robotics 🦆
### Coursework for [CMPUT 412](https://apps.ualberta.ca/catalogue/course/cmput/412)
This project focuses on the exploration and implementation of various mobile robotics concepts using the Duckiebot platform and Robot Operating System (ROS). The Duckiebot is a cost-effective, compact mobile robot designed for educational and research applications. ROS is a flexible and widely-used framework for developing robotic systems in both academia and industry.

For more information, please visit my [course webpage](https://sites.google.com/view/justinvalentine/home).

## 🧩 Implemented Concepts
The following mobile robotics concepts were implemented in this project:

- **PID Controllers:** The Duckiebot utilizes PID controllers for lane-following and tailing other Duckiebots.

- **Machine Learning-based Digit Classification:** A machine learning model enables the Duckiebot to classify handwritten digits captured by an onboard camera.

- **AprilTag Localization:** The Duckiebot employs AprilTags, or visual markers, to perform localization, accurately determining its position and orientation within the environment.

- **Computer Vision:** The Duckiebot implements computer vision techniques such as histogram filtering and morphological operations to detect lane markings and other road markers.

- **Sensor Fusion:** The Duckiebot fuses odometry data and AprilTag Localization to achieve enhanced localization accuracy.

- **Differential Drive Kinematics:** The Duckiebot relies on differential drive kinematics to compute its movements based on the velocities of its wheels.

## ⚙️ Dependencies
To run this project, the following dependencies are required:

- ROS Noetic
- Duckietown Shell (for Duckiebot platform setup)
- ROS packages: duckietown, duckietown_msgs, duckietown_visualization, tensorflow, numpy, OpenCV