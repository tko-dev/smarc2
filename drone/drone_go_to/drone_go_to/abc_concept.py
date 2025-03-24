from rclpy.node import Node
import abc
from abc import ABCMeta

from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle


class ActionServerBase(Node, metaclass=ABCMeta):
    def __init__(self, node_name: str, action_name: str, action_type, **kwargs):
        super().__init__(node_name)
        self.action_type = action_type
        self._server = ActionServer(
            self, self.action_type, action_name, self._execution_callback, **kwargs
        )
    @property
    def actionResultMsg(self):
        """Empty result message."""
        return self.action_type.Result()
    @property
    def actionFeedbackMsg(self):
        """Empty feedback message."""
        return self.action_type.Feedback()

    @abc.abstractmethod
    def _execution_callback(self, goal_handle: ServerGoalHandle):
        """Primary execution callback

        Returns:
            A populated `self.actionResultMsg()` or more generically a Ros ActionType.Result()
        """
        pass
