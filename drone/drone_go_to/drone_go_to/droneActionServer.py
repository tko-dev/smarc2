import rclpy

from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from drone_go_to_interfaces.action import GoToDrone
from geographic_msgs.msg import GeoPoint
from geodesy import utm

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
        self.logger.info(f"{goal_handle.request}")
        utm_val = utm.fromMsg(goal_handle.request.geopoint)
        self.logger.info(f"{utm_val}")
        feedback_msg = GoToDrone.Feedback()
        for i in range(5):
            import time
            feedback_msg.distance_remaining = float(i)
            goal_handle.publish_feedback(feedback_msg)
            self.logger.info("Sending feedback")

            time.sleep(1)
        result_msg = GoToDrone.Result()
        goal_handle.succeed()
        return result_msg

def main(args=None):
    rclpy.init(args=args)
    action_client = DroneActionServer()
    rclpy.spin(action_client)

if __name__ == "__main__":
    main()
