**Vision-Based Servo Control using ROS2**



This project demonstrates a real-time vision-based closed-loop control system implemented using ROS2 Humble, OpenCV, and TurtleSim. A live camera feed is processed to detect and track a target object, and the resulting visual error is used to control a simulated robot in real time.



The system showcases a complete perception - control - actuation pipeline using standard ROS2 communication patterns and mixed-language nodes.



**System Architecture**



* **Camera Node (Python)**

&nbsp;  Captures live video frames and publishes them as ROS2 image messages.



* **Vision Tracker Node (Python + OpenCV)**

&nbsp;  Performs object detection and tracking, computes the target centroid, and publishes a  normalized horizontal error signal.



* **Controller Node (C++)**

&nbsp;  Implements a proportional (P) controller that converts visual error into velocity commands.



* **Simulation (TurtleSim)**

&nbsp;  Visualizes the robot motion driven purely by vision-based feedback.



**Key Concepts Demonstrated**



* ROS2 publishers and subscribers
* Multi-node system design
* OpenCV integration with ROS2 via cv\_bridge
* Vision-based feedback control
* Mixed Python and C++ ROS2 nodes
* Unified system launch using ROS2 launch files



**Tech Stack**



* ROS2 Humble
* Python (rclpy, OpenCV)
* C++ (rclcpp)
* TurtleSim
* Ubuntu



**Running the Project**



Build the workspace:

**colcon build**

**source install/setup.bash**



Launch the full system:



**ros2 launch vision\_servoing full\_system.launch.py**





**Demo**



A video demonstration is included showing:

* Live camera feed with bounding box and centroid
* Real-time closed-loop control of the simulated robot



**Notes**



This project is intended to highlight system-level robotics understanding rather than application-specific tuning. It is fully simulation-based and does not require physical hardware.





