import rclpy
import math
import numpy as np
import time
from gpiozero import Button

from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import Button  # Import Button from gpiozero
from fe_interfaces.msg import Angle, MotorControl, Holes
from my_robot.PID import PID
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Point


class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        self.publisher_button = self.create_publisher(String, 'button', 10)
        self.angle_subscriber = self.create_subscription(Angle, 'angles', self.angle_callback, 10)
        self.publisher_ = self.create_publisher(MotorControl, 'motor_control', 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.object_subscriber = self.create_subscription(Point, '/object_detection', self.object_callback, 10)
        self.holes_sub = self.create_subscription(Holes, "hole", self.hole_callback, 10)


        # self.timer = self.create_timer(1.0, self.publish_message)

        # Set up GPIO pin 4 as an input button
        self.button = Button(4)
        
        self.start = False
        self.turn = False
        self.initial = True
        self.turn_counter = 0
        self.hole = Holes()

        # Add a listener to the button press
        
        self.get_logger().info("Listening for button presses on GPIO pin 13.")
        # self.button.when_pressed = self.on_button_press()
        
        # mode for open or obstacle challenge
        self.mode = True

        self.target_heading_rev = np.array([math.pi/2, math.pi, (3*math.pi)/2, (2*math.pi)])
        self.target_heading = np.array([(3*math.pi)/2, math.pi, math.pi/2, 0.0])

        self.prev_angle = 0.0
        self.current_angle = 0.0

        self.move_pid = PID(0, 11,0.1)
        self.move_object_pid = PID(0, 0.02,0.02)
        self.turnOutput = PID(0, 2, 0, 0.08)
        self.turn_pid = PID(0, 10,0.01)

        self.imu = Imu()
        self.angle = Angle()
        self.corrected_yaw = 0.0

        self.turnSettled = self.turnOutput.is_settled()

        self.is_turning = False  # Track if the robot is currently turning
        self.turn_target_yaw = 0.0  # Store target yaw for the turn
        self.turning = 0


        # for obstacle_challenge
        self.object = Point()

    def hole_callback(self, msg):
        self.hole = msg

    def object_callback(self, msg):
        self.object = msg

    def imu_callback(self, msg: Imu):

        """1. get the IMU roll angle.
        2. get the traget angle from the taget heading arrays(clockwise or counter_clockwise)
        3. check if the robot reaches the target heading 
        """
        self.corrected_yaw = msg.orientation.x
        # Normalize corrected yaw to be within [-2PI, 2PI]
        # if self.imu.orientation.x > math.pi:
        #     self.corrected_yaw -= 2 * math.pi
        # elif self.imu.orientation.x < -math.pi:
        #     self.corrected_yaw += 2 * math.pi

        
        self.prev_angle_angle = self.corrected_yaw
        

    def normalize_yaw(self, yaw):
        # Normalize yaw to be within -pi to pi
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi
        return yaw
        

    def angle_callback(self, msg: Angle):
        """
        1. the autonomous code: mode true -> open challeng; mode -> false -> obstacle
        2. starts when the button is pressed
        """
        # self.get_logger().info(f"button: {self.button.when_released}")
        self.button.when_released = self.on_button_press
        if self.start:
            self.angle = msg
            turnOutput_PID = self.turnOutput.calculate_pid_output((math.pi/2) - (self.corrected_yaw))

            if self.mode:

                if self.turn_counter == 12:
                    self.stop()
                    self.get_logger().info(f"Turning completed")
                else:
                    if msg.angles[3] > 0.01 and self.is_turning == False:
                        self.get_logger().info(f"Turn: {self.turn}")
                        # self.get_logger().info(f"Moving forward{msg.angles[3]}")
                        self.move_forward(msg)
                    
                        if self.hole.rotation != 0:
                            if self.hole.rotation == 1: #Counter Clockwise
                                self.turn = False
                            else: #Clockwise
                                self.turn = True

                        self.get_logger().info(f"Turn: {self.turn}")
                        
            else:
                self.get_logger().info(f"Obstacle challenge")
                self.avoid_object(msg)

    def avoid_object(self, msg_angle: Angle):
        """
        1. if obstacle run the code
        2. if the object is furthur than 0.006 place the object red: on the right of the frame; green: on the left of the frame
        3. 1 = red, 0 = green, -1 = no objects
        4. if there is no object and teh disatance is less than 0.006 move to the center of the game field
        """
        msg = MotorControl()
        msg.forward = True
        msg.speed = 0.28
        # msg.speed = 0.3
        # msg.speed = 0.28
        msg.angle = 20.0
        msg.stop = False

        if self.mode:
            error = msg_angle.angles[2] - msg_angle.angles[0]
            pid_output = self.move_pid.calculate_pid_output(error)
            angle = self.clamp(12.0 - pid_output, 3.0, 20.0)
            msg.angle = angle
        else:
            if self.object.y > 0.006:
                if self.object.x == 1.0:
                    error = self.object.z + 300.0
                elif self.object.x == 0.0:
                    error = self.object.z - 300.0
                self.get_logger().info(f"Error: {self.object.z}")
                pid_output = self.move_object_pid.calculate_pid_output(error)
                self.get_logger().info(f"PID: {pid_output}")
                angle = self.clamp(12.0 + pid_output, 0.0, 20.0)
                msg.angle = angle
            elif self.object.x == -1.0:
                error = msg_angle.angles[2] - msg_angle.angles[0]
                pid_output = self.move_pid.calculate_pid_output(error)
                angle = self.clamp(12.0 - pid_output, 0.0, 20.0)
                msg.angle = angle
        # self.get_logger().info(f"Forward: {self.turn_counter}")
        # self.get_logger().info(f"Angle: {msg.angle}")
        # self.get_logger().info(f"PID: {pid_output}")
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Moving forward")


    
    def move_forward(self, msg_angle: Angle):
        
        """
        1. find the difference between the left and right lidar
        2. take the error and input into PI
        3. center the robot
        """
        msg = MotorControl()
        msg.forward = True
        msg.speed = 1.0
        # msg.speed = 0.3
        # msg.speed = 0.28
        msg.angle = 22.0
        self.is_turning == False
        msg.stop = False

        if self.mode == True:
            self.get_logger().info(f"Turn: {self.turn}")
            
            if self.turn: #Counter Clockwise
                error = self.target_heading[self.turn_counter%4] - self.corrected_yaw
                self.turn_target_yaw = self.target_heading[self.turn_counter%4]
                # error = math.pi/2 - (self.corrected_yaw - self.prev_angle)
            else: #clockwise
                error = self.target_heading_rev[self.turn_counter%4] - self.corrected_yaw
                self.turn_target_yaw = self.target_heading_rev[self.turn_counter%4]

            # self.get_logger().info(f"Turn: {self.turn}")
            # self.get_logger().info(f"target: {self.target_heading}")
            # self.get_logger().info(f"yaw: {self.corrected_yaw}")
            # self.get_logger().info(f"Error Turning: {abs(error)}")
            # self.get_logger().info(f"Counter: {self.turn_counter}")
        
            #check if the robot reaches the traget heading
            if abs(error) < 0.1:  # Threshold for completing the turn
                self.turn_counter += 1
                self.is_turning = False  # End turning state
                self.get_logger().info("Turn completed. Moving forward.")

        if self.hole.rotation == 2:
            error = msg_angle.angles[2] - 0.50 #track the RIGHT wall
            self.get_logger().info(f"tracking RIGHT {msg_angle.angles[2]}")
            # self.turning = self.target_heading_rev[self.turn_counter%4] - self.corrected_yaw
            
        elif self.hole.rotation == 1:
            error = 0.50 - msg_angle.angles[0]  #track the LEFT wall
            self.get_logger().info(f"tracking LEFT {msg_angle.angles[0]}")
            
            # self.turning = self.target_heading[self.turn_counter%4] - self.corrected_yaw
        else:
            error = msg_angle.angles[2] - msg_angle.angles[0] #track both side because no hole known yet.
            self.get_logger().info(f"tracking BOTH")

        # if abs(self.turning) < 0.05:
        #     self.turn_counter += 1

        pid_output = self.move_pid.calculate_pid_output(error)
        angle = self.clamp(20 + pid_output, 10.0, 27.0)
        msg.angle = angle
        self.get_logger().info(f"Forward: {self.turn_counter}")
        self.get_logger().info(f"ERROR: {error}")
        self.get_logger().info(f"Angle: {msg.angle}")
        # self.get_logger().info(f"PID: {pid_output}")
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Moving forward")
    
    def start_turn(self):
        """
        1. start the turning the state
        2. true: right; false: left
        """
        self.get_logger().info(f"Turning in progress")
        # Set target yaw for a 90-degree turn, adjusting based on current yaw
        if self.turn:
            self.turn_target_yaw = ((math.pi*3)/2)
        else:
            self.turn_target_yaw = (math.pi/2)
        # self.turn_target_yaw = self.normalize_yaw(self.turn_target_yaw)
        
        # Enter turning state
        self.is_turning = True
        self.get_logger().info(f"Turn: {self.turn}")
        if self.turn:
            self.get_logger().info("Turning right")
            self.turn_right()
        else:
            self.get_logger().info("Turning left")
            self.turn_left()

    def turn_left(self):
        msg = MotorControl()
        msg.forward = True
        msg.speed = 0.0
        msg.stop = False
        # msg.angle = 20.0
        self.is_turning == True
        self.publisher_.publish(msg)
        turnOutput_PID = self.turnOutput.calculate_pid_output(self.corrected_yaw - self.target_heading_rev[self.turn_counter%4])
        error = (-1 * self.target_heading_rev[self.turn_counter%4]) + self.corrected_yaw
        pid_output = self.turn_pid.calculate_pid_output(error)
        self.get_logger().info(f"PID Turn: {pid_output}")
        angle = self.clamp(5 + pid_output, 3.0, 12.0)
        self.get_logger().info(f"Angle: {angle}")
        msg.angle = angle
        self.publisher_.publish(msg)
    
    def turn_right(self):
        msg = MotorControl()
        msg.forward = True
        msg.speed = 0.0
        msg.stop = False
        msg.angle = 20.0
        self.publisher_.publish(msg)
        turnOutput_PID = self.turnOutput.calculate_pid_output(self.target_heading[self.turn_counter%4] - (self.corrected_yaw))
        error = self.target_heading[self.turn_counter%4] - self.corrected_yaw
        pid_output = self.turn_pid.calculate_pid_output(error)
        angle = self.clamp(20.0 + pid_output, 0.0, 24.0)
        msg.angle = angle
        self.publisher_.publish(msg)
        

    def stop(self):
        msg = MotorControl()
        msg.stop = True
        msg.forward = False
        self.publisher_.publish(msg)
        self.get_logger().info(f"Stopping")

    def clamp(self,input_value, min_value, max_value):
        if input_value > max_value:
            return max_value
        if input_value < min_value:
            return min_value
        return input_value

    def publish_message(self):
        msg = String()
        msg.data = 'Button not pressed'
        self.publisher_button.publish(msg)
        # self.get_logger().info(f'Published: {msg.data}')

    def on_button_press(self):
        msg = String()
        msg.data = 'Button pressed!'
        self.start = True
        self.publisher_button.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
