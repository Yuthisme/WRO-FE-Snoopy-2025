#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from fe_interfaces.msg import Angle, Holes

# Tunables
FORWARD_MIN = 0.20 #STARTING DISTANCE OF "B"
FORWARD_MAX = 0.50 #ENDING DISTNACE OF "B"
THRESHOLD = 0.15 #THRESHOLD FOR CORRECTING LIDAR ERROR


class AngleSubscriber(Node):
    """Subscribes to Angle and /scan; extracts left/right wedges from LaserScan."""

    def __init__(self):
        super().__init__('angle_subscriber')
        self.create_subscription(Angle, 'angles', self.angle_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_process, 10)
        self.publisher = self.create_publisher(Holes, 'hole', 10)
    #DECLARABLE VARIABLES
        self.left_min_angle = 0
        self.left_max_angle = 0
        self.right_min_angle = 0
        self.right_max_angle = 0   

        self.scan = None
        self._msg_count = 0
        self.turning = False
        self.holes = Holes()

        self.right = 0
        self.left = 0

        self.angles_right = np.array([])
        self.angles_left = np.array([])

        self.hole_right = False
        self.hole_left = False
        self.turn = 0


    def angle_callback(self, msg: Angle):
        # if self.scan is None:
        #     return
        # self._msg_count += 1

        # The dynamic "a"
        self.left  = float(msg.angles[0])
        self.right = float(msg.angles[2])

        left_count = 0

        # Calculation using of the triangles for the angles
        left_min_hyp  = math.hypot(self.left,  FORWARD_MIN)
        left_max_hyp  = math.hypot(self.left,  FORWARD_MAX)
        right_min_hyp = math.hypot(self.right, FORWARD_MIN)
        right_max_hyp = math.hypot(self.right, FORWARD_MAX)

        # self.get_logger().info(f"Left: {left_min_hyp}, Right: {left_max_hyp}")

        self.right_min_angle  = math.degrees(math.acos(self.left / left_min_hyp))   + 180.0
        self.right_max_angle  = math.degrees(math.acos(self.left / left_max_hyp))   + 180.0
        self.left_min_angle = 360.0 - math.degrees(math.acos(self.right / right_min_hyp))
        self.left_max_angle = 360.0 - math.degrees(math.acos(self.right / right_max_hyp))




    def scan_process(self, msg):
        index_right, self.angles_right, ranges_right, index_left, self.angles_left, ranges_left = self.extract_sector(msg) #GET DATA FROM FUNCTION 

        # self.get_logger().info(f"a left: {self.left}")
        # self.get_logger().info(f"range left size: {ranges_left.size}")
        # self.get_logger().info(f"max range left: {max(ranges_left)}")
        # self.get_logger().info(f"a right: {self.right}")

        if self.angles_right.size > 0 and self.angles_left.size > 0 and self.right > 0 and self.left > 0:
            exp_right, exp_left = self.expected_hypo() # GET CALCULATED HYPOTENUE FROM FUNCTION

            # self.get_logger().info(f"left: {self.left_min_angle} right: {self.left_max_angle}")

        
            # self.get_logger().info(f"angles {self.angles.size}")
            # self.get_logger().info(f"hypo: {data.size}")
            # self.get_logger().info(f"ranges right: {ranges_right}")
            # self.get_logger().info(f"ranges left: {ranges_left}")

            count_right = 0
            count_left = 0
            
            for i in range(math.ceil(exp_right.size/2), exp_right.size):
                if ranges_right[i]  > exp_right[i] + THRESHOLD:
                    count_right += 1
            for i in range(math.ceil(exp_left.size/2), exp_left.size):
                if ranges_left[i] > exp_left[i] + THRESHOLD:
                    count_left += 1

            self.get_logger().info(f"count_right: {count_right}, exp: {round(exp_right.size/2)}")
            self.get_logger().info(f"count_left: {count_left}, exp: {round(exp_left.size/2)}")

            if count_right >= round(exp_right.size/2):
                self.hole_right = True
            elif count_right < round(exp_right.size/2):
                self.hole_right = False
            if count_left >= round(exp_left.size/2):
                self.hole_left = True
            elif count_left < round(exp_left.size/2):
                self.hole_left = False

        else:
            self.hole_left = False
            self.hole_right = False

        if self.turn == 0:
            if self.hole_left is not None and self.hole_right is not None:
                if self.hole_left == True:
                    self.turn = 1
                elif self.hole_right == True:
                    self.turn = 2
                else:
                    self.turn = 0
            else:
                self.turn = 0

        self.holes.rotation = self.turn
        self.holes.hole[0] = self.hole_left
        self.holes.hole[1] = self.hole_right
        
        self.publisher.publish(self.holes)
            
                
                    

            


        # if count >= math.ceil(data.size/2):
        #     self.get_logger().info(f"HOLES! {count}")


    def expected_hypo(self):

        ang_right = np.radians(self.angles_right)  # convert degrees to radians RIGHT SIDE
        ang_left = np.radians(self.angles_left) # covnert degrees to radians LEFT SIDE

        self.get_logger().info(f"angle left: {ang_left}")
        # self.get_logger().info(f"exp right: {ang_right}")

        a_right = self.right # DYNAMIC A RIGHT SIDE
        a_left = self.left # DYNAMIC A LEFT SIDE

        if ang_right.size == 0 or a_right == 0: #CHECK IF DATA IS VALID RIGHT SIDE
            self.get_logger().warn("NO DATA!")
            return np.array([]), np.array([])
        
        if ang_left.size == 0 or a_left == 0: #CHECK IF DATA IS VALID LEFT SIDE
            self.get_logger().warn("NO DATA!")
            return np.array([]), np.array([])

        target_hyp_right = a_right / np.cos(ang_right) # vectorize the npyarray RIGHT SIDE
        target_hyp_left = a_left / np.cos(ang_left) # vectorize the npyarray LEFT SIDE

        
        # target_hyp_right = np.round(target_hyp_right, 5)
        # target_hyp_left = np.round(target_hyp_left, 5)

        # self.get_logger().info(f"Computed hypotenuses: {abs(target_hyp_right)}") 
        self.get_logger().info(f"Computed hypotenuses: {abs(target_hyp_left)}")
        return abs(target_hyp_right), target_hyp_left
    
    def deg_to_rad(num): 
        return num * (math.pi/180) #CONVERTS DEGREE TO RADIAN

        
    def extract_sector(self, scan):

        start_deg_right = self.right_min_angle 
        end_deg_right = self.right_max_angle
        
        start_deg_left = self.left_max_angle
        end_deg_left = self.left_min_angle

        self.get_logger().info(f"left min: {start_deg_left}")
        self.get_logger().info(f"left max: {end_deg_left}")
        """
        Returns (indices, angles_deg, ranges) for the requested sector.
        Filters out invalid ranges (NaN/inf/<=0).
        """
        mask_right = self.sector_mask_deg(scan, start_deg_right, end_deg_right, include_end=True)
        mask_left = self.sector_mask_deg(scan, start_deg_left, end_deg_left, include_end=True)

        idx_right = np.nonzero(mask_right)[0]
        idx_left = np.nonzero(mask_left)[0]
        
        if idx_right.size == 0 or idx_left.size == 0:
            return idx_right, np.array([]), np.array([]), idx_left, np.array([]), np.array([])
        
        
        
        ranges_right = np.asarray(scan.ranges)[idx_right]
        angles_deg_right = np.degrees(scan.angle_min + idx_right * scan.angle_increment)

        ranges_left = np.asarray(scan.ranges)[idx_left]
        angles_deg_left = np.degrees(scan.angle_min + idx_left * scan.angle_increment)

        # keep only valid distances
        valid_right = np.isfinite(ranges_right) & (ranges_right > 0.0)
        valid_left = np.isfinite(ranges_left) & (ranges_left > 0.0)

        self.get_logger().info(f"idx left: {idx_left.size}")
        self.get_logger().info(f"idx right: {idx_right.size}")

        return idx_right[valid_right], angles_deg_right[valid_right], ranges_right[valid_right], idx_left[valid_left], angles_deg_left[valid_left], ranges_left[valid_left]
    
    def sector_mask_deg(self, scan: LaserScan, start_deg: float, end_deg: float, include_end: bool = True):
        """
        Boolean mask selecting scan rays whose bearing (in deg, normalized to [0,360))
        lies within [start_deg, end_deg]. Handles wrap-around (e.g., 350..10).
        """
        N = len(scan.ranges)
        angles_rad = scan.angle_min + np.arange(N) * scan.angle_increment
        angles_deg = np.degrees(angles_rad)
        angles360 = self._norm_deg_0_360(angles_deg)

        a = self._norm_deg_0_360(start_deg)
        b = self._norm_deg_0_360(end_deg)

        if a <= b:
            mask = (angles360 >= a) & ((angles360 <= b) if include_end else (angles360 < b))
        else:
            # wrap-around range like 350..10
            mask = (angles360 >= a) | ((angles360 <= b) if include_end else (angles360 < b))
        return mask
    
    def _norm_deg_0_360(self, deg):
        return (deg % 360.0 + 360.0) % 360.0

    def scan_callback(self, msg):

        self.get_logger().info(f"increment: {msg.angle_increment}")
        # for i, angle_deg in enumerate(self.angles_array):
        #     target_angle = math.radians(angle_deg)  # Convert degrees to radians for indexing
            
        #     # Check if the target angle is within the laser scan range
        #     if not (msg.angle_min <= target_angle <= msg.angle_max):
        #         self.get_logger().info(f"Target angle {angle_deg}° is out of range.")
        #         continue

        #     # Calculate the index corresponding to the target angle
        #     index = int((target_angle - msg.angle_min) / msg.angle_increment)

        #     # Ensure the index is within the range array
        #     if 0 <= index < len(msg.ranges):
        #         range_at_angle = msg.ranges[index]
                
        #         # Check if the range value is within the sensor's valid range
        #         if msg.range_min <= range_at_angle <= msg.range_max:
        #             self.get_logger().info(f"Range at {angle_deg}°: {range_at_angle} meters")
        #             self.angle_msg.angles[i] = range_at_angle  # Assign to the Angle message
        #         else:
        #             self.get_logger().info(f"Range at {angle_deg}° is out of sensor's range.")
        #             self.angle_msg.angles[i] = float('nan')  # Set to NaN if out of range
        #     else:
        #         self.get_logger().info(f"Index for angle {angle_deg}° is out of range.")

        # for i, angle_deg in enumerate(self.target_angles_deg_mult):
        #     target_angle = math.radians(angle_deg)  # Convert degrees to radians for indexing
            
        #     # Check if the target angle is within the laser scan range
        #     if not (msg.angle_min <= target_angle <= msg.angle_max):
        #         self.get_logger().info(f"Target angle {angle_deg}° is out of range.")
        #         continue

        #     # Calculate the index corresponding to the target angle
        #     index = int((target_angle - msg.angle_min) / msg.angle_increment)

        #     # Ensure the index is within the range array
        #     if 0 <= index < len(msg.ranges):
        #         range_at_angle = msg.ranges[index]
                
        #         # Check if the range value is within the sensor's valid range
        #         if msg.range_min <= range_at_angle <= msg.range_max:
        #             self.get_logger().info(f"Range at {angle_deg}°: {range_at_angle} meters")
        #             self.angle_msg.angle_mult[i] = range_at_angle  # Assign to the Angle message
        #         else:
        #             self.get_logger().info(f"Range at {angle_deg}° is out of sensor's range.")
        #             self.angle_msg.angle_mult[i] = float('nan')  # Set to NaN if out of range
        #     else:
        #         self.get_logger().info(f"Index for angle {angle_deg}° is out of range.")

            
    

def main(args=None):
    rclpy.init(args=args)
    node = AngleSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
