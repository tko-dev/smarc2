from typing import Optional

import rclpy

import numpy as np

from rclpy.action import ActionServer
from rclpy.time import Duration, Time
from rclpy.action.server import ServerGoalHandle
from tf2_geometry_msgs import do_transform_pose
from rclpy.node import Node
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
            "drone_go_to",
            self._execute_callback,
        )
        # TODO: Get this as parameter in a real version
        self.robot_name = "Quadrotor"
        self.target_topic = f"{self.robot_name}/{DroneTopics.UNITY_TARGET}"
        # TODO: Discuss with Ozer
        # FIX: Probably should not use ground truth but rather use estimate position
        self.target_frame = f"{self.robot_name}/{DroneLinks.BASE_LINK}_gt"
        self.logger = self.get_logger()
        self.logger.info(f"Publishing outputs to unity at {self.target_topic}")

        # NOTE: Publisher to target topic that allows for teleportation of the target
        self._publisher = self.create_publisher(Pose, self.target_topic, 5)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

    def transform_goal(self, utm_val: utm.UTMPoint) -> Optional[Pose]:
        # TODO: Discuss with Ozer
        # Is pulling the most recent time stamp appropriate here:
        # Rationale: yes because the goal request is not stamped and we want the latest transform
        try:
            t = self._tf_buffer.lookup_transform(
                self.target_frame, "utm", Time(seconds=0), timeout=Duration(seconds=2)
            )
        except TransformException as e:
            self.logger.error({e})
            # TODO: Discuss with Ozer
            # I dislike making this an optional return wonder if there is a better way to handle transform error
            # basically it puts this on the caller to check if transform worked
            return None
        goal = Pose()
        # based on ReadMe in repository
        goal.position.x = utm_val.easting
        goal.position.y = utm_val.northing
        goal.position.z = utm_val.altitude
        return do_transform_pose(goal, t)

    def compute_distance(self, utm_val: utm.UTMPoint):
        position = self.transform_goal(utm_val)
        if position is not None:
            delta = np.sqrt(
                (position.position.x) ** 2
                + (position.position.y) ** 2
                + (position.position.z) ** 2
            )
            return delta
        else:
            # C: style error code
            return -1.0

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        self.logger.info("Executing callback")
        self.logger.info(f"{goal_handle.request}")
        utm_val: utm.UTMPoint = utm.fromMsg(goal_handle.request.geopoint)
        self.goal_base_link = self.transform_goal(utm_val)
        if self.goal_base_link is None:
            goal_handle.abort()
        else:
            self.logger.info(
                f"Publishing to {self.target_topic}, with position {self.goal_base_link}"
            )
            self._publisher.publish(self.goal_base_link)
        feedback_msg = GoToDrone.Feedback()

        distance = self.compute_distance(utm_val)
        while distance > 0.1:
            feedback_msg.distance_remaining = distance
            goal_handle.publish_feedback(feedback_msg)
            self.logger.info("Sending feedback")
            self.logger.info(f"Distance to target {distance}")
            distance = self.compute_distance(utm_val)

        result_msg = GoToDrone.Result()
        goal_handle.succeed()
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    action_client = DroneActionServer()
    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
