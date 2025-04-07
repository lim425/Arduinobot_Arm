
# ROS 2 Manipulator Project

## About the Project
This repository contains my work from the **[Robotics and ROS 2 - Learn by Doing! Manipulators](https://www.udemy.com/course/robotics-and-ros-2-learn-by-doing-manipulators/)** Udemy course by Antonio Brandi. The project focuses on building and controlling a robotic manipulator using ROS 2 (Robot Operating System 2). It includes simulations in Gazebo, forward and inverse kinematics, and motion planning with MoveIt, all developed through hands-on exercises from the course.

The goal was to learn ROS 2 fundamentals and apply them to manipulator robotics, culminating in a functional robotic arm system that can be simulated and controlled using Amazon Alexa voice control.



## Acknowledgements

Special thanks to Antonio Brandi for the excellent "Robotics and ROS 2 - Learn by Doing! Manipulators" course.

 - [Original course github repo](https://github.com/AntoBrandi/Robotics-and-ROS-2-Learn-by-Doing-Manipulators?tab=readme-ov-file#about-the-course)
  - [Udemy course](https://www.udemy.com/course/robotics-and-ros-2-learn-by-doing-manipulators/)



## Usage

Step1: Start the Gazebo simulation with the manipulator:

```bash
ros2 launch arduinobot_bringup simulated_robot.launch.py
```

Step2: Start the ngrok web server

```bash
./ngrok http 5000
```

Step3: Copy the ngrok link and paste it in the section Endpoint of the Alexa Developer account.  - [Alexa Developer Console](https://developer.amazon.com/alexa/console/ask)

![Image Alt](https://github.com/lim425/Arduinobot_Arm/blob/05403b78b3e611e87eb0b201000a54f353aa18c4/images/Screenshot%20from%202025-04-07%2014-33-36.png).
![Image Alt](https://github.com/lim425/Arduinobot_Arm/blob/05403b78b3e611e87eb0b201000a54f353aa18c4/images/Screenshot%20from%202025-04-07%2014-34-25.png).

Step4: Control the manipulator using voice command
![Image Alt](https://github.com/lim425/Arduinobot_Arm/blob/05403b78b3e611e87eb0b201000a54f353aa18c4/images/Screenshot%20from%202025-04-07%2014-45-22.png).


