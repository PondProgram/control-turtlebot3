import rclpy
from rclpy.node import Node
from srv_msg.srv import StopRobot  
from geometry_msgs.msg import Twist

class StopRobotServer(Node):
    def __init__(self):
        super().__init__('stop_robot')
        self.srv = self.create_service(StopRobot, 'stop_robot', self.callback)
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

    def callback(self, request, response):
        if request.stop:
            self.publish_velocity()  
            response.message = "Robot Stop"
        return response
    
    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.0   
        msg.angular.z = 0.0  
        self.cmd_vel.publish(msg)  

def main():
    rclpy.init()
    node = StopRobotServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
