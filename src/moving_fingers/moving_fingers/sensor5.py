import rclpy
import rclpy.node as node_mod
from rclpy.node import Node

from std_msgs.msg import Float64
import numpy as np


class Sensor5(Node):
    """ A moving finger """
    publisher_: node_mod.Publisher
    period: int
    timer: node_mod.Timer
    x: float
    y: float

    def __init__(self) -> None:
        """ Initializer """
        super().__init__("sensor_5")
        self.publisher_ = self.create_publisher(Float64, 'sensor5', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0
        self.y = 0

    def increment(self) -> None:
        """ Increment self.i by 1 """
        self.x += 1
        self.y = np.sin(np.degrees(self.x))

    def timer_callback(self):
        """ The callback function for this package """
        self.increment()
        msg = Float64()
        msg.data = self.y
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: \"{msg.data}\"')

def main(args=None):
    """ The main executable body of the module. 
    
    Will initialize the rclpy module and run the program 
    """
    rclpy.init(args=args)

    sensor5 = Sensor5()
    rclpy.spin(sensor5)

    # Once complete
    sensor5.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
