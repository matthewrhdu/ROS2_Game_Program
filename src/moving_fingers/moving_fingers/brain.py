import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import rclpy.node as node_mod
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64
from redirection.action import Redirection


class Brain(Node):
    """ A subscriber class to read finder movement details"""
    subscription: node_mod.Subscription
    _action_client1: ActionClient

    def __init__(self, num: int):
        """ The initializer """
        super().__init__('brain')
        self.sensor = num
        self.subscription = self.create_subscription(Float64, f"sensor{num}", self.listener_callback, 10)
        self._action_clients = ActionClient(self, Redirection, f"actuator{num}")
        self.send_to = None

    def listener_callback(self, msg: Float64):
        """ The callback function for the subscriber"""
        if 0.9 <= msg.data <= 1:
            self.get_logger().info(f'position: \"{msg.data}\"')
            self.send_to = self._action_clients[self.sensor - 1]

    def send_goal(self, order: float):
        self.get_logger().info(f'Sending Data')
        goal_msg = Redirection.Goal()
        goal_msg.order = order

        self.send_to.wait_for_server()

        if self.send_to is not None:
            self._send_goal_future = self.send_to.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            
            self.send_to = None

def main(args = None):
    """ The main executable for the listener """
    rclpy.init(args=args)
    brains = [Brain(n) for n in range(1, 5 + 1)]

    executor = MultiThreadedExecutor(5)

    for i in range(5):
        executor.add_node(brains[i])

    executor.spin()

    for s in range(5):
        brains[s].destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()