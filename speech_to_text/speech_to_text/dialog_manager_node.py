""" ROS2 Node for Dialog Manger """

import time
import rclpy
from simple_node import Node

from speech_to_text_msgs.msg import StringArray
from speech_to_text_msgs.action import ListenOnce
from std_srvs.srv import Empty


class DialogManagerNode(Node):
    """Dialog Manager Node Class"""

    def __init__(self) -> None:
        super().__init__("dialog_manager_node")

        self.is_new_msg = False
        self.new_msg = None

        # service clients
        self.__start_listening_client = self.create_client(Empty, "start_listening")
        self.__stop_listening_client = self.create_client(Empty, "stop_listening")
        self.__calibrating_client = self.create_client(Empty, "calibrate_listening")

        # pubs and subs
        self.__pub = self.create_publisher(StringArray, "stt_dialog", 10)

        self.__subscription = self.create_subscription(
            StringArray, "stt_parse", self.__stt_callback, 10
        )

        # action server
        self.__action_server = self.create_action_server(
            ListenOnce, "listen_once", execute_callback=self.__execute_server
        )

    def __stt_callback(self, msg: StringArray) -> None:
        """final speech calback

        Args:
            msg (StringArray): list of tags
        """

        self.get_logger().info("Dialog Manager: " + str(msg.strings))
        self.__pub.publish(msg)

        if not self.is_new_msg:
            self.new_msg = msg
            self.is_new_msg = True

    def calibrate_stt(self) -> None:
        """calibrate stt method"""

        req = Empty.Request()
        self.__calibrating_client.wait_for_service()
        self.__calibrating_client.call(req)
        self.get_logger().info("calibrating stt")

    def start_stt(self) -> None:
        """start stt method"""

        req = Empty.Request()
        self.__start_listening_client.wait_for_service()
        self.__start_listening_client.call(req)
        self.get_logger().info("starting stt")

    def stop_stt(self) -> None:
        """stop stt method"""

        req = Empty.Request()
        self.__stop_listening_client.wait_for_service()
        self.__stop_listening_client.call(req)
        self.get_logger().info("stopping stt")

    def __execute_server(self, goal_handle) -> ListenOnce.Result:
        """action server execute callback

        Args:
            goal_handle ([type]): goal_handle

        Returns:
            ListenOnce.Result: action server result (list of tags)
        """

        self.is_new_msg = False
        self.new_msg = StringArray()

        if goal_handle.request.calibrate:
            self.calibrate_stt()

        # starting stt
        self.start_stt()

        # wait for message
        while not self.is_new_msg and not goal_handle.is_cancel_requested:
            self.get_logger().info("Waiting for msg")
            time.sleep(0.5)

        # stoping stt
        self.stop_stt()

        # results
        result = ListenOnce.Result()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()

        else:
            result.stt_strings = self.new_msg.strings
            goal_handle.succeed()

        return result


def main():
    rclpy.init()
    node = DialogManagerNode()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
