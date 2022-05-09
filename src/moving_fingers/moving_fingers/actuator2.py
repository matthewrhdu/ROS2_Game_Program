from cupshelpers import activateNewPrinter
import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

from redirection.action import Redirection

class Actuator2(Node):
    _action_server: ActionServer

    def __init__(self):
        super().__init__('actuator_2')
        self._action_server = ActionServer(self, Redirection, 'actuator2', self.execute_callback)

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f'Rotating 90 degrees')
        goal_handle.succeed()
        
        result = Redirection.Result()
        result.exit_status = 0
        return result


def main(args=None):
    rclpy.init(args=args)

    actuator = Actuator2()

    rclpy.spin(actuator)


if __name__ == '__main__':
    main()