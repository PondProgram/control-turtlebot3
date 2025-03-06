import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from srv_msg.msg import WarningObstacle  

class RobotControlSubscriber(Node):
    def __init__(self):
        super().__init__('robot_control_subscriber')

        self.robot_control_sub = self.create_subscription(
            String, 
            'robot_control', 
            self.robot_control_callback, 
            10
        )
        
        self.warning_obstacle_sub = self.create_subscription(
            WarningObstacle, 
            'show_msg', 
            self.warning_obstacle_callback, 
            10
        )

        self.data = "No Data"
        self.angle = 0.0
        self.distance = 0.0
        
    def robot_control_callback(self, msg):
        self.data = msg.data

    def warning_obstacle_callback(self, msg):
        self.angle = msg.angle
        self.distance = msg.distance
        self.show_msg()  

    def show_msg(self):
        self.get_logger().info(f'{self.data} \nAngle: {self.angle} degrees \nDistance: {self.distance} meters\n')

def main():
    rclpy.init()
    try:
        node = RobotControlSubscriber()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
