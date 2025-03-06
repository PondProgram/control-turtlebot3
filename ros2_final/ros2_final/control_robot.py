import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        self.control_sub = self.create_subscription(
            String,
            'robot_control',
            self.control,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            'control_scan',
            self.control_callback,
            qos_profile
        )
        
        self.max_linear_speed = 0.20
        self.min_linear_speed = 0.05
        self.max_angular_speed = 1.0
        self.min_angular_speed = 0.10
        
        self.max_detection_distance = 0.30  
        self.min_detection_distance = 0.10  
        self.service_response = None

        self.front_angles = (-15, 15)
        self.left_angles = (75, 105)
        self.right_angles = (-105, -75)
        self.back_angles = (165, -165)

    def control(self,msg):
        self.service_response = msg.data

    def get_region_distance(self, ranges, angle_range):
        start_idx = int((angle_range[0] + 180) * len(ranges) / 360)
        end_idx = int((angle_range[1] + 180) * len(ranges) / 360)
        
        if end_idx < start_idx:
            region_ranges = ranges[start_idx:] + ranges[:end_idx]
        else:
            region_ranges = ranges[start_idx:end_idx]
            
        valid_ranges = [r for r in region_ranges if r > 0.01 and not np.isinf(r)]
        return min(valid_ranges) if valid_ranges else float('inf')

    def calculate_speed(self, distance):
        if distance >= self.max_detection_distance:
            return self.min_linear_speed, self.min_angular_speed
        
        elif distance <= self.min_detection_distance:
            return self.max_linear_speed, self.max_angular_speed
        
        else:
            ratio = 1.0 - ((distance - self.min_detection_distance) / 
                         (self.max_detection_distance - self.min_detection_distance))
            
            linear_speed = self.min_linear_speed + ratio * (self.max_linear_speed - self.min_linear_speed)
            angular_speed = self.min_angular_speed + ratio * (self.max_angular_speed - self.min_angular_speed)
            
            return linear_speed, angular_speed

    def control_callback(self, scan_msg):
        front_dist = self.get_region_distance(scan_msg.ranges, self.front_angles)
        left_dist = self.get_region_distance(scan_msg.ranges, self.left_angles)
        right_dist = self.get_region_distance(scan_msg.ranges, self.right_angles)
        back_dist = self.get_region_distance(scan_msg.ranges, self.back_angles)
        
        cmd = Twist()
   
        if front_dist < self.max_detection_distance:
            if self.service_response != 'Obstacle detected Front!':
                linear_speed, _ = self.calculate_speed(front_dist)
                cmd.linear.x = linear_speed
            
        elif left_dist < self.max_detection_distance:
            _, angular_speed = self.calculate_speed(left_dist)
            cmd.angular.z = angular_speed
            
        elif right_dist < self.max_detection_distance:
            _, angular_speed = self.calculate_speed(right_dist)
            cmd.angular.z = -angular_speed
            
        elif back_dist < self.max_detection_distance:
            if self.service_response != 'Obstacle detected Back!':
                linear_speed, _ = self.calculate_speed(back_dist)
                cmd.linear.x = -linear_speed
    
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()