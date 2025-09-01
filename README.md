# <ins> Snoopy: WRO Future Engineer 2025 </ins>
---
## Table of Content
1. [Introduce the team](#1.introduce-the-team)
2. [What is WRO?](#2.what-is-wro?)
   - [Why do we choose Future Engineer](#a.why-do-we-choose-future-engineer)
3. [Robots](#3.robots)
4. [Video](#4.video)
5. [Hardware](#5.hardware)
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
6. [Software](#software)
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
<p><ins><i>Team Picture</ins></p><br>
   
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
   <p>i.<b>3D design</b></p><br>
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

- <b>Camera Mount: <br>
  <img src="https://github.com/user-attachments/assets/510ba08f-254e-48b5-b4e0-090ad0a76fa3" width=375 height=750><br>
  </b> The camera mount is positioned high to allow the camera to easily detect traffic lights during the obstacle challenge. Its base is precisely aligned with the LiDAR mounting screw holes, as the LiDAR is mounted directly beneath the camera. 


   
