import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from drone_go_to_interfaces.action import GoToDrone
from geographic_msgs.msg import GeoPoint

class DroneActionClient(Node):
    def __init__(self):
        super().__init__('drone_action_client')
        self._client = ActionClient(
            self,
            GoToDrone,
            'drone_go_to',
        )
        self.logger = self.get_logger()
        self.logger.info("Initializing drone action client node")
    def send_goal(self):
        goal_msg = GoToDrone.Goal()
        goal_msg.geopoint = GeoPoint()
        self.logger.info(f"Sending empty geopoint {GeoPoint()}")

        server_ready = self._client.wait_for_server(timeout_sec=1)
        self._send_goal_future = self._client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        if server_ready:
            pass
        else:
            self.logger.error("Action server does not exist")
    def _feedback_callback(self, feedback_msg: GoToDrone.Feedback):
        pass


def main(args=None):
    rclpy.init(args=args)
    action_client = DroneActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == "__main__":
    main()
