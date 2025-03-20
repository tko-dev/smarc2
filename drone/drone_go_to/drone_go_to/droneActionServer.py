from typing import Optional

import rclpy

from rclpy.action import ActionServer
from rclpy.time import Duration, Time
from rclpy.action.server import ServerGoalHandle
from tf2_ros import TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Pose
from geodesy import utm
from tf2_ros import TransformListener, Buffer, TransformException

from drone_go_to_interfaces.action import GoToDrone
from drone_msgs.msg import Links as DroneLinks
from drone_msgs.msg import Topics as DroneTopics

class DroneActionServer(Node):
    def __init__(self):
        super().__init__("drone_action_server") 
        self._server = ActionServer(
            self,
            GoToDrone,
            'drone_go_to',
            self._execute_callback,
        )
        # TODO: Get this as parameter
        self.robot_name = "Quadrotor"
        self.target_topic = f"{self.robot_name}/{DroneTopics.UNITY_TARGET}"
        self.target_frame = f"{self.robot_name}/{DroneLinks.BASE_LINK}_gt"
        self.logger = self.get_logger()
        self.logger.info(f"Publishing outputs to unity at {self.target_topic}")

        # INFO:
        # publisher topic takes a pose that is the drone's reference frame, so need to get proper transform
        # robot.name/BASELINK
        self._publisher = self.create_publisher(Pose, self.target_topic, 5)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread = True)

    def transform_goal(self, utm_val: utm.UTMPoint) -> Optional[Pose]:
        # TODO: Discuss with Ozer
        # FIX: If latest is appropriate
        try:
            t = self._tf_buffer.lookup_transform(self.target_frame, 'utm', Time(seconds=0), timeout=Duration(seconds=5))
        except TransformException as e:
            self.logger.error({e})
            return None
        goal = Pose()
        goal.position.x = utm_val.easting
        goal.position.y = utm_val.northing
        goal.position.z = utm_val.altitude
        return do_transform_pose(goal, t)

        pass
    def _execute_callback(self, goal_handle: ServerGoalHandle):
        self.logger.info("Executing callback")
        self.logger.info(f"{goal_handle.request}")
        utm_val: utm.UTMPoint = utm.fromMsg(goal_handle.request.geopoint)
        goal_base_link = self.transform_goal(utm_val)
        if goal_base_link is None:
            goal_handle.abort()
        else:
            self.logger.info(f"Publishing to {self.target_topic}, with position {goal_base_link}")
            self._publisher.publish(goal_base_link)
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
