import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from srv_msg.msg import WarningObstacle
from srv_msg.srv import StopRobot
from geometry_msgs.msg import Twist
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class LidarHistogram(Node):
    def __init__(self):
        super().__init__('lidar_histogram')

        self.client = self.create_client(StopRobot, 'stop_robot')
        try:
            if not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Service not available')
                raise RuntimeError('Stop robot service not available')
        except Exception as e:
            self.get_logger().error(f'Service setup error: {str(e)}')

        self.lidar_max_range = 0.20
 
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.control_pub = self.create_publisher(String, 'robot_control', 10)
        self.show_msg = self.create_publisher(WarningObstacle, 'show_msg', 10)
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile
        )
        
        self.front_angles = (325, 35)    
        self.left_angles = (36, 144)    
        self.right_angles = (216, 324)   
        self.back_angles = (145, 215)    

        
        self.last_obstacle_location = None

    def get_region_distance(self, ranges, angle_range):
        num_readings = len(ranges)
        start_idx = int((angle_range[0] % 360) * num_readings / 360)
        end_idx = int((angle_range[1] % 360) * num_readings / 360)

        if start_idx > end_idx:  
            region_ranges = ranges[start_idx:] + ranges[:end_idx]
            region_indices = list(range(start_idx, num_readings)) + list(range(0, end_idx))
        else:
            region_ranges = ranges[start_idx:end_idx]
            region_indices = list(range(start_idx, end_idx))

        valid_data = [(r, idx) for r, idx in zip(region_ranges, region_indices) if r > 0.01 and not np.isinf(r)]

        if valid_data:
            min_range, min_idx = min(valid_data, key=lambda x: x[0])  
            min_angle = (min_idx / num_readings) * 360  
            return min_range, min_angle
        else:
            return float('inf'), None  


    def lidar_callback(self, scan_msg):
        status_msg = String()
        value = WarningObstacle()
        status_msg.data = "Continue"
        
        front_dist, front_angle = self.get_region_distance(scan_msg.ranges, self.front_angles)
        left_dist, left_angle = self.get_region_distance(scan_msg.ranges, self.left_angles)
        right_dist, right_angle = self.get_region_distance(scan_msg.ranges, self.right_angles)
        back_dist, back_angle = self.get_region_distance(scan_msg.ranges, self.back_angles)
        
        if front_dist < self.lidar_max_range:
            self.last_obstacle_location = 'Obstacle detected Front!'
            value.angle = front_angle
            value.distance = front_dist
            self.show_msg.publish(value)
            self.stop_robot()
            return  
        
        if back_dist < self.lidar_max_range:
            self.last_obstacle_location = 'Obstacle detected Back!'
            value.angle = back_angle
            value.distance = back_dist
            self.show_msg.publish(value)
            self.stop_robot()
            return  

        if left_dist < self.lidar_max_range:
            status_msg.data = 'Obstacle detected Left!'
            value.angle = left_angle
            value.distance = left_dist
            self.show_msg.publish(value)
            self.control_pub.publish(status_msg)
        
        if right_dist < self.lidar_max_range:
            status_msg.data = 'Obstacle detected Right!'
            value.angle = right_angle
            value.distance = right_dist
            self.show_msg.publish(value)
            self.control_pub.publish(status_msg)

        if all(d >= self.lidar_max_range for d in [front_dist, back_dist, left_dist, right_dist]):
            self.control_pub.publish(status_msg)

    def stop_robot(self):
        request = StopRobot.Request()
        request.stop = True
        future = self.client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            status_msg = String()
            
            if response.message == "Robot Stop":
                status_msg.data = self.last_obstacle_location
                self.control_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main():
    rclpy.init()
    try:
        node = LidarHistogram()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()