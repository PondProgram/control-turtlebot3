import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class TestScan(Node):
    def __init__(self):
        super().__init__('test_scan')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile
        )


    def scan_callback(self, msg):
        self.min_angle = msg.angle_min
        self.max_angle = msg.angle_max
        self.increment = msg.angle_increment
        self.range = msg.ranges

        self.range = [float('nan') if r > 0.5 else r for r in self.range]

        print(f"angle min: {self.min_angle}")
        print(f"angle max: {self.max_angle}")
        print(f"angle increment: {self.increment}")
        print("\n")
        print(f"ranges: {self.range}")
        print("\n")
        
def main(args=None):
    rclpy.init(args=args)
    controller = TestScan()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
