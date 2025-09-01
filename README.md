# <ins> Snoopy: WRO Future Engineer 2025 </ins>
---
## Table of Content
1. [Introduce the team](#1introduce-the-team)
2. [What is WRO?](#2what-is-wro)
   - [Why do we choose Future Engineer](#why-we-choose-future-engineer)
3. [Robot](#3robot)
4. [Video](#4-video)
5. [Hardware](#5-hardware)
   - [Mobility Management](#mobility-management)
     - [3D design](#3d-design)
     - [Motor](#motor)
     - [Differential](#differential)
     - [Steering](#steering)
     - [Chassis](#chassis)
     - [Assembly]
   - [Power and Sense manaagement](#power-and-sense-management)
     - [Component](#component)
     - [Sensor](#sensor)
     - [Power Supply](#power-supply)
     - [Microcontroller](#microcontroller)
     - [Wiring and Scheme](#wiring-and-scheme)
6. [Software](#6software)
   - [ROS2](#ros2)
   - [OpenCV](#open-cv)
   - [Code for components](#code-for-components)
     - [Camera](#camera)
     - [Lidar](#lidar)
     - [Motor](#motor)
     - [Servo](#servo)
     - [IMU](#imu)
7. [Obtacle Management](#7obstacle-management)
   - [Strategy](#strategy)
     - [Opene Challenge](#open-challenge)
     - [Obstacle Challenge](#obstacle-challenge)
8. [Conclusion](#8conclusion)
---
# 1.Introduce the team
---
<p>
<img src="https://github.com/user-attachments/assets/99e08ddf-4632-4423-bc2b-aeaab1a01221" width=500 height=1000>
</p>
<p><ins><i>Snoopy at WRO INTERNATIONAL 2024</i></ins></p>

Snoopy is a team of passionate young engineers. We started back in grade 11(2024), when three high school students and our robotics teacher teamed up after being inspired by our seniors winning the Startup Idea Award at the WRO International Round in Panama. Last year, we joined the World Robotics International Round in Türkiye, where we learned a lot and made plenty of new friends. [Instagram](https://www.instagram.com/snoopy.cambodia?utm_source=ig_web_button_share_sheet&igsh=ZDNlZDc0MzIxNw==)

---
### Snoopy's Team:
<p>
1. Reaksa Chhoeun<br>
<img src="https://github.com/user-attachments/assets/bacfc1fc-540f-4118-9942-3dc456ac5d30" width=250 height=500><br>
- Team's Advisor<br>
- "Don't give up"<br>
</p>

<p>
2. Seyha Hoy<br>
<img src="https://github.com/user-attachments/assets/b9e06bc5-40fd-4185-bb35-531299ff98bb" width=250 height=500><br>
- Team's Coach<br>
- "Stay Curious"<br>
</p>

<p>
3. Leangheng Vongchhayyuth<br>
<img src="https://github.com/user-attachments/assets/589ef2f6-37d7-4a6b-bf84-9f11694ce73f" width=250 height=500><br>
-Team's Leader<br>
-Team's coder<br>
-16 years old<br>
-<b>School:</b> AUPP High School-Foxcroft Academy<br>
-"God did"<br>
</p>

<p>
4. Bunchhoeun Rattanakboth<br>
<img src="https://github.com/user-attachments/assets/261895cf-c033-498d-92bf-aceaecbe7529" width=250 height=500><br>
-Team's mechanics<br>
-16 years old<br>
-<b>School:</b> AUPP High School-Foxcroft Academy<br>
-"What Should I say?"<br>
</p>

<p>
5. Sreng Kimroathpiseth<br>
<img src="https://github.com/user-attachments/assets/0f51f6fe-61f0-462b-9b5a-e1d460cdedc4" width=250 height=500><br>
-Team's wiring<br>
-17 years old<br>
-<b>School:</b> AUPP High School-Foxcroft Academy<br>
-"Bro idk too"<br>
</p>

<img src="https://github.com/user-attachments/assets/fe8ad09f-b546-4c13-abdc-b8fbc1afb1ee" width = 500 height = 1000><br>
<p><ins><i>Team Picture with coach</i></ins></p><br>
<img src="https://github.com/user-attachments/assets/eee48061-4476-43d7-8ea4-15de23d855b7" width = 500 height =1000><br>
<p><ins><i>Team Picture</i></ins></p><br>
   
---
# 2.What is WRO?
---
WRO (World Robotics Olympiad) is a global robotics competition for young people. It aims to foster creativity, design, and problem-solving skills through challenging and educational robotics activities. WRO involves teams designing and programming robots to complete tasks or solve problems in various categories. [link](https://wro-association.org/)

## Why we choose Future Engineer?

Future Engineer is one of the interesting categories out of all four, because we love to do challenging and complicated things. We chose this category because we all have an interest in cars and want to be involved in that type of engineering, which is our goal to become engineers in the future.

---
# 3.Robot
---

---
# 4. Video
---
- [Youtube](https://youtube.com/@snoopywrocambodia?si=DruGofx_E3J2LwbV)
  - [Open Challenge](https://youtu.be/84BnV2V4fig?si=563qHp9vC6NSgZDj)
  - Obstacle Challenge

---
# 5. Hardware
---
Our robot is built to meet all the game’s requirements. It runs on two motors: one for steering and one for driving. For sensors, we use LiDAR to detect walls, a camera to recognize traffic light colors in the obstacle challenge, and an IMU to track the car’s position.

## Mobility management
### 3D design
   <P>Some parts of our robot were 3D design on our own, and some parts we found online and modified them.</P><br>

| Owned 3D design       | Online 3D design  |
|-----------------      |------------------ |
| Robot's Layer         | Differential Grear|
| Camera mount          | Differential Case |
| Battery Checker Holder|                   |
| Battery Holder        |                   |
|Right rear wheel holder|                   |



#### Robot's Layer
<img src="https://github.com/user-attachments/assets/aaca7009-b4dd-4e82-aebd-a0c9af01f0f8" width=375 height=750><br>
The robot’s chassis is built in three layers: bottom, middle, and top.<br>
  - <b> Chassis: </b> This layer houses the motors with encoders, the steering servo, and the differential. A cutout at the rear of the chassis accommodates the differential, ensuring smooth operation. At the front, the servo and servo rod control the steering of the front wheels.
  - <b> Middle Layer: </b> This layer supports most of the sensors and electronic components, including the camera, LiDAR, motor driver, camera mount, ESP32, battery, and battery checker.
  - <b> Top Layer: </b> The top layer is reserved for the main computing and power components, such as the Raspberry Pi 5, our custom-designed PCB, and the buck converter.

#### Camera Mount:
  <img src="https://github.com/user-attachments/assets/510ba08f-254e-48b5-b4e0-090ad0a76fa3" width=375 height=750><br>
 The camera mount is positioned high to allow the camera to easily detect traffic lights during the obstacle challenge. Its base is precisely aligned with the LiDAR mounting screw holes, as the LiDAR is mounted directly beneath the camera. 
 
#### Battery Checker Holder:
<img width="375" height="750" alt="bat checker" src="https://github.com/user-attachments/assets/f13d800a-2806-4c40-9baf-d081b0c5cf81"><br>
This holder is designed to securely house the battery checker at the back of the robot. The holder’s hole is 10 mm larger than the battery checker to ensure a snug fit.

#### Battery Holder
<img width="375" height="750" alt="bat holder" src="https://github.com/user-attachments/assets/d3af527b-e0ad-4e5e-b33b-040028335573"><br>
The battery holder is designed to secure the battery in place. Similarly, its hole is 10 mm larger than the battery to prevent it from falling out while the robot is in motion.

#### Right Rear Wheel Holder
<img src="https://github.com/user-attachments/assets/2246a5c2-5962-460f-b24b-a8b475e72799" width=375 height=750><br>
This component stabilizes the drive shaft extending from the differential to the right rear wheel.

#### Differential Gear
  <img width="375" height="375" alt="image" src="https://github.com/user-attachments/assets/2f343618-e08c-4044-a46f-e895a8b7485a" ><br>
   We sourced the design from GrabCAD and modified the central hole of the gear to fit the motor shaft precisely.[link](https://grabcad.com/library/lego-technic-gears-1)
   
#### Differential Case
  <img width="375" height="375" alt="image" src="https://github.com/user-attachments/assets/fb99b4ba-bc86-428c-8ea5-d4b03283b81f"><br>
  We adapted the design from [Team StormsNGR](https://github.com/MoCsabi/WRO2024-FE-StormsNGR/blob/main/models/Differential%20Gear%20House.stl), modifying certain parts to accommodate a different motor and to provide a secure mount for the motor.

### Motor
<img width="375" height="375" alt="image" src="https://github.com/user-attachments/assets/45456f2a-1c85-4800-aebb-f3f30f401d74" /><br>
We selected the JGB37-520B DC motor with encoder to track the distance traveled by the robot. This allows us to determine the motor’s speed, control when the robot needs to turn, and monitor wheel rotations. As a result, the robot can stop precisely at its starting point and execute accurate left and right turns.

### Differential
<img width="292" height="173" alt="image" src="https://github.com/user-attachments/assets/df867e55-7b48-4ea0-a3f0-5153c15578fb" /><br>
The robot uses a LEGO differential, which allows the left and right wheels to rotate at different speeds when turning, enabling smooth cornering. We also chose this differential because it easily connects to the LEGO wheels used on the robot, simplifying assembly and ensuring reliable performance.

### Steering
<img width="250" height="500" alt="Front Wheel" src="https://github.com/user-attachments/assets/bffc9a41-fa0f-4189-9bd0-e1eee96b88b9" /><br>
The robot uses a servo motor and a servo rod for steering. The servo is controlled to rotate at specific angles using a PID controller. The servo rod is attached to an arm connected to the servo shaft, which pushes or pulls the linkage as the servo rotates, causing the wheels to pivot in the desired direction.

### Chassis
<img src=https://github.com/user-attachments/assets/d28ae8a6-d40f-4ef8-92fb-1cab2617d32a width=500 height=1000><br>

- <b>Front section</b>: Designed to allow smoother turning and wheel rotation without interference from other parts of the robot. This section also accommodates the servo motor.
- <b>Middle section</b>: Serves as the mounting area for the motor. The motor driver and LiDAR are mounted upside down on the second layer of the car, connected above this section.
- <b>Rear section</b>: Houses the differential and rear wheels, with a cutout that allows the differential to rotate freely. This section also connects the rear wheels to the differential.

## Power and Sense Managament
### Component
At the core of our robot, we have a Raspberry Pi 5 with 8GB of RAM. This Raspberry Pi 5 processes all the data sent to the robot by the other components, such as the lidar, camera, motor, and IMU. Here are the listing and purpose:

<p><b>IMU (Inertial Measurement Unit)</b></p><br>
<p>A device that measures the orientation and acceleration of a moving object. In our self-driving robot, the IMU provides crucial information about the robot's orientation, angular velocity, and linear acceleration. This data is essential for vehicle state estimation, sensor fusion, and control systems.</p><br>

<p><b> LiDAR (Light Detection and Ranging)</b></p><br>
<p>It emits laser beams to measure distances to objects in its surroundings. It provides a 3D point cloud representation of the environment, enabling the robot to perceive obstacles, terrain, and other relevant features. Key applications of LiDAR in your self-driving robot include obstacle detection and avoidance, mapping and localization, and terrain analysis.</p><br>

<p><b> Raspberry Pi 5 8GB RAM</b></p><br>
<p>The Raspberry Pi 5 serves as the brain of our self-driving robot. It processes sensor data, executes control algorithms, and communicates with other components. Key functionalities of the Raspberry Pi 5 in your robot include sensor data processing, algorithm execution, communication, and machine learning.</p><br>

<p><b> Motor Driver</b></p><br>
<p>A motor driver is an electronic circuit that controls the speed and direction of electric motors. It acts as an interface between the Raspberry Pi 5 and the motors, allowing the robot to move forward and backward. Key features of a motor driver include power amplification, direction control, and speed control.</p><br>

<p><b>Motor</b></p><br>
<p>Motors are the actuators that convert electrical energy into mechanical energy, enabling the robot to move. In our self-driving robot, motors are typically used to drive the wheels. Key characteristics of motors for self-driving robots include torque, speed, and efficiency.</p><br>

<p><b> Servo Motor </b></p><br>
<p>A servo motor is a rotary actuator with a built-in position sensor. It allows precise control of the angle of rotation, making it ideal for steering mechanisms in self-driving robots. Key features of a servo motor include precision and holding torque.</p><br>

<p><b>Raspberry Pi Camera Module v2</b></p>
<p>The Raspberry Pi Camera Module v2 is an 8MP camera with a fixed-focus lens. It captures still images up to 3280×2464 and records video at 1080p 30fps, 720p 60fps, or 480p 90fps. It connects to the Pi with a CSI ribbon cable and is commonly used in robotics, vision projects, and security cameras.</p><br>

<p><b>ESP32</b></p><br>
<p>The ESP32 is a powerful microcontroller with built-in Wi-Fi and Bluetooth. It has many pins for sensors and devices, making it popular for IoT, robotics, and smart gadgets. It’s low-cost, fast, and easy to program with Arduino or MicroPython. We use this to receive data from the Lidar and send the data to the Raspberry Pi</p><br>

<p><b>RGB</b></p><br>
<p>RGB stands for Red, Green, and Blue, the three primary colors of light. By mixing these colors in different intensities, almost any other color can be created. We use this to tell us when the robot is ready.</p>

### Sensor
Our robots use multiple sensors for different purposes, like the Lidar we use to detect the wall since it uses lasers to measure the distance between the wall and the robot, so it’ll make it easy to identify how far or how close the robot is when we test the robots in both open and obstacle challenges. The other sensor that we use is the Camera, which we use to detect the color of the traffic light during the obstacle challenge, so it can send a signal to the robot that either turn left or right. We also use IMU to track the position of the robot, we use it to count how many laps the robot has traveled, and to make the robot move in a fixed position.

### Power Supply
We chose an 11.1V LiPo battery since it can provide us with enough power to run all of our components, such as the Raspberry Pi, servo, motor, and sensors. Since the servo, IMU, and motor encoder have a maximum voltage of 5V, we use a buck converter to step down the 11.1V from the battery to 5V, which is then supplied to the components that require only 5V. Additionally, we use a boost converter to step up the 11.1V to 12V to power the motor.

### Microcontroller
The robot Raspberry Pi serves as the main controller, managing all operations. We selected the Raspberry Pi for its processing speed, ease of integration with other components, and the ability to upload or modify code via SSH, eliminating the need to connect a USB cable like with Arduino or ESP32.

### Wiring and Scheme
<p>For wiring, we chose to design our own PCB board that has all the pins that indicate each GPIO pin of the Raspberry Pi 5 and a JST pin for power supply, RGB, Motor encoder, Motor Driver, Motor, IMU, and push button. The screw terminal connectors for the power supply.</p><br>
<img width="500" height="1000" alt="pi" src="https://github.com/user-attachments/assets/1b490486-4d8a-48fc-b444-5107b55dea68" /><br>
<img width="580" height="566" alt="image" src="https://github.com/user-attachments/assets/dad754b6-7eaa-4d3d-ad41-a044ed9f8af4" /><br>
<img width="1000" height="500" alt="image" src="https://github.com/user-attachments/assets/8aae5d95-c62f-4fe5-8ae7-960d1ce65ea2" /><br>
<p>This schematic illustrates a robot control system designed around a Raspberry Pi 5, featuring an IMU for orientation, encoders for wheel feedback, DC motors for driving, and a servo for steering or actuation.</p>

---
# 6.Software
---
	
 We decided to use ROS2 as the main framework for our robots, while OpenCV for the camera. ROS2 provides the communication framework that connects different parts of the system, while OpenCV enables the car to process camera images and recognize objects. Together with sensors like LiDAR and cameras, they allow the robot to detect obstacles, identify lanes, and make driving decisions.

## ROS2

<img src="https://github.com/user-attachments/assets/89468c08-233b-4d48-a26b-9ab78021b3c2" width=500 height=1000><br>

ROS2, the Robot Operating System 2, acts as the central nervous system of the self-driving car. It provides a flexible framework for organizing and coordinating various software components, including nodes, topics, and services. Nodes perform specific tasks, such as sensor data processing, control algorithms, or communication with other nodes. Topics are channels for publishing and subscribing to messages, enabling data exchange between nodes. Services provide a request-response mechanism for nodes to interact and request services from each other.

## Open CV

OpenCV, the Open Source Computer Vision Library, empowers the car to "see" the world around it. It enables a wide range of computer vision techniques, including image processing, feature detection, object detection and tracking, and optical flow.

To integrate these components, a ROS2-based system is meticulously crafted. LiDAR data is captured and processed to generate a point cloud representation of the environment, while camera images are captured and pre-processed using OpenCV to enhance clarity and contrast. LiDAR-based perception uses point cloud data to detect obstacles and create a 3D map of the surroundings, while camera-based perception employs OpenCV to identify lanes, traffic signs, and other relevant visual cues.


## Code for Component
### Camera
### Lidar
### Motor
### Servo
### IMU
---
# 7.Obstacle Management

There are two challenges: the Open Challenge and the Obstacle Challenge. The open challenge requires the robot to drive three full laps around the game field and stop at its starting point. While the Obstacle challenge requires the robot to drive 3 full laps too but there are traffic lights and a parking lot. GREEN = right, RED = left. 

## Strategy 
### Open Challenge
<img width="638" height="530" alt="image" src="https://github.com/user-attachments/assets/dc2c9a11-ced6-4ca3-af5e-93527d3ef58c" /><br>
Our robot begins its run when the push button is pressed, activating its wall-tracking system. Using LiDAR and IMU data, the robot simultaneously monitors both the left and right walls to stay centered. When the left or right wall shows a gap (detected as a hole), the robot recognizes it as a turn. It then follows the wall, making a precise 90° turn, which is counted as one completed turn. If no hole is detected, the robot continues moving forward while keeping track of both walls. The ESP32 ensures fast and reliable data transfer from the LiDAR to the Raspberry Pi 5, which fuses data from the IMU and Pi Camera Module v2 for orientation, stability, and lane recognition. After completing 12 turns, the robot executes a controlled stop. This strategy minimizes wall contact, ensures stable turns, and achieves accurate lap completion.

### Obstacle Challenge
In the Obstacle Challenge, our approach is extended to include traffic sign recognition and parking. The camera identifies red and green pillars, which dictate whether the vehicle should stay on the right or left side of the lane. The LiDAR, with data again transmitted through the ESP32 module, confirms obstacle distances and ensures safe path planning, while the IMU stabilizes navigation during detours. The Raspberry Pi 5 executes a state-machine control system that interprets sign information, chooses the correct lane, and adjusts the steering smoothly to avoid moving signs, since avoiding contact yields higher points. After completing three laps, the camera and LiDAR are used together to detect the magenta parking lot boundaries. The vehicle then executes a precise parallel parking maneuver, carefully controlling the steering servo and motors to ensure it is fully inside the parking space without touching the barriers. The RGB LEDs signal the vehicle’s status throughout (ready, running, or parking), helping the team and judges monitor progress.

---
# 8. Conclusion
Through countless hours of design, coding, testing, and teamwork, Team Snoopy Cambodia has developed a robot that balances innovation with reliability. By combining ROS2, OpenCV, LiDAR, IMU, and a Raspberry Pi 5, we created a self-driving system capable of navigating open tracks, handling obstacles, and performing precise self-parking.<br>
This journey has been more than just building a robot it has been about learning, problem-solving, and pushing our creativity to new levels. Each challenge taught us how to improve our engineering, strengthen our teamwork, and bring our vision closer to reality.<br>
As we move forward to WRO 2025, we are proud of what we’ve achieved and excited to keep innovating, improving, and representing Cambodia on the international stage.







   
