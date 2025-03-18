import rclpy

from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from drone_go_to_interfaces.action import GoToDrone
from geographic_msgs.msg import GeoPoint

class DroneActionServer(Node):
    def __init__(self):
        super().__init__("drone_action_server") 
        self._server = ActionServer(
            self,
            GoToDrone,
            'drone_go_to',
            self._execute_callback,
        )
        self.logger = self.get_logger()
    def _execute_callback(self, goal_handle: ServerGoalHandle):
        self.logger.info("Executing callback")
        feedback_msg = GoToDrone.Feedback()
        print(type(goal_handle))
        print(request = goal_handle.request)
        pass

def main(args=None):
    rclpy.init(args=args)
    action_client = DroneActionServer()
    rclpy.spin(action_client)

if __name__ == "__main__":
    main()
