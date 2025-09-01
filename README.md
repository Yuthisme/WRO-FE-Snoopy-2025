# <ins> Snoopy: WRO Future Engineer 2025 </ins>
---
## Table of Content
1. [Introduce the team](#1introduce-the-team)
2. [What is WRO?](#2what-is-wro?)
   - [Why do we choose Future Engineer](#why-do-we-choose-Future-Engineer)
3. [Robot](#3robot)
4. [Video](#4video)
5. [Hardware](#5hardware)
   - Mobility Management
     - 3D design
     - Motor
     - Differential
     - Steering
     - Chassis
     - Assembly
   - Power and Sense manaagement
     - Component
     - Sensor
     - Power Supply
     - Microcontroller
     - Wiring and Scheme
6. [Software](#Software)
   - ROS2
   - OpenCV
   - Code for components
     - Camera
     - Lidar
     - Motor
     - Servo
     - IMU
7. [Obtacle Management](#Obstacle-management)
   - Strategy
     - Opene Challenge
     - Obstacle Challenge
8. [Conclusion](#Conclusion)
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
   <h3><b>3D design</b></h3><br>
   <P>Some parts of our robot were 3D design on our own, and some parts we found online and modified them.</P><br>

| Owned 3D design       | Online 3D design  |
|-----------------      |------------------ |
| Robot's Layer         | Differential Grear|
| Camera mount          | Differential Case |
| Battery Checker Holder|                   |
| Battery Holder        |                   |
|Right rear wheel holder|                   |



- <b> Robot's Layer : </b>The robot’s chassis is built in three layers: bottom, middle, and top.<br>
<img src="https://github.com/user-attachments/assets/aaca7009-b4dd-4e82-aebd-a0c9af01f0f8" width=375 height=750><br>
  - <b> Chassis: </b> This layer houses the motors with encoders, the steering servo, and the differential. A cutout at the rear of the chassis accommodates the differential, ensuring smooth operation. At the front, the servo and servo rod control the steering of the front wheels.
  - <b> Middle Layer: </b> This layer supports most of the sensors and electronic components, including the camera, LiDAR, motor driver, camera mount, ESP32, battery, and battery checker.
  - <b> Top Layer: </b> The top layer is reserved for the main computing and power components, such as the Raspberry Pi 5, our custom-designed PCB, and the buck converter.

- <b>Camera Mount:</b> <br>
  <img src="https://github.com/user-attachments/assets/510ba08f-254e-48b5-b4e0-090ad0a76fa3" width=375 height=750><br>
 The camera mount is positioned high to allow the camera to easily detect traffic lights during the obstacle challenge. Its base is precisely aligned with the LiDAR mounting screw holes, as the LiDAR is mounted directly beneath the camera. 
- <b> Battery Checker Holder:</b><br>
<img width="375" height="750" alt="bat checker" src="https://github.com/user-attachments/assets/f13d800a-2806-4c40-9baf-d081b0c5cf81"><br>
This holder is designed to securely house the battery checker at the back of the robot. The holder’s hole is 10 mm larger than the battery checker to ensure a snug fit.
- <b> Battery Holder:</b> <br>
<img width="375" height="750" alt="bat holder" src="https://github.com/user-attachments/assets/d3af527b-e0ad-4e5e-b33b-040028335573"><br>
The battery holder is designed to secure the battery in place. Similarly, its hole is 10 mm larger than the battery to prevent it from falling out while the robot is in motion.
- <b> Right Rear Wheel Holder: </b><br>
<img src="https://github.com/user-attachments/assets/2246a5c2-5962-460f-b24b-a8b475e72799" width=375 height=750><br>
This component stabilizes the drive shaft extending from the differential to the right rear wheel.
- <b> Differential Gear: </b><br>
  <img width="375" height="375" alt="image" src="https://github.com/user-attachments/assets/2f343618-e08c-4044-a46f-e895a8b7485a" ><br>
   We sourced the design from GrabCAD and modified the central hole of the gear to fit the motor shaft precisely.[link](https://grabcad.com/library/lego-technic-gears-1)
- <b> Differential Case:</b><br>
  <img width="375" height="375" alt="image" src="https://github.com/user-attachments/assets/fb99b4ba-bc86-428c-8ea5-d4b03283b81f"><br>
  We adapted the design from [Team StormsNGR](https://github.com/MoCsabi/WRO2024-FE-StormsNGR/blob/main/models/Differential%20Gear%20House.stl), modifying certain parts to accommodate a different motor and to provide a secure mount for the motor.

<h3><b>Motor</b></h3>
<img width="375" height="375" alt="image" src="https://github.com/user-attachments/assets/45456f2a-1c85-4800-aebb-f3f30f401d74" /><br>
We selected the JGB37-520B DC motor with encoder to track the distance traveled by the robot. This allows us to determine the motor’s speed, control when the robot needs to turn, and monitor wheel rotations. As a result, the robot can stop precisely at its starting point and execute accurate left and right turns.

<h3><b>Differential</b></h3>
<img width="292" height="173" alt="image" src="https://github.com/user-attachments/assets/df867e55-7b48-4ea0-a3f0-5153c15578fb" /><br>
The robot uses a LEGO differential, which allows the left and right wheels to rotate at different speeds when turning, enabling smooth cornering. We also chose this differential because it easily connects to the LEGO wheels used on the robot, simplifying assembly and ensuring reliable performance.

<h3><b>Steering</b></h3>
<img width="250" height="500" alt="Front Wheel" src="https://github.com/user-attachments/assets/bffc9a41-fa0f-4189-9bd0-e1eee96b88b9" /><br>
The robot uses a servo motor and a servo rod for steering. The servo is controlled to rotate at specific angles using a PID controller. The servo rod is attached to an arm connected to the servo shaft, which pushes or pulls the linkage as the servo rotates, causing the wheels to pivot in the desired direction.

<h3><b>Chassis</b></h3>
<img src=https://github.com/user-attachments/assets/d28ae8a6-d40f-4ef8-92fb-1cab2617d32a width=500 height=1000><br>

- <b>Front section</b>: Designed to allow smoother turning and wheel rotation without interference from other parts of the robot. This section also accommodates the servo motor.
- <b>Middle section</b>: Serves as the mounting area for the motor. The motor driver and LiDAR are mounted upside down on the second layer of the car, connected above this section.
- <b>Rear section</b>: Houses the differential and rear wheels, with a cutout that allows the differential to rotate freely. This section also connects the rear wheels to the differential.







   
