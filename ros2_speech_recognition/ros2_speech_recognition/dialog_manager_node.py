""" ROS2 Node for Dialog Manger """

import time
import rclpy

from ros2_speech_recognition_interfaces.msg import StringArray
from ros2_speech_recognition_interfaces.action import ListenOnce
from std_srvs.srv import Empty

from custom_ros2 import (
    Node,
    ActionSingleServer
)


class DialogManagerNode(Node):
    """ Dialog Manager Node Class """

    def __init__(self):
        super().__init__("dialog_manager_node")

        self.is_new_msg = False
        self.new_msg = None

        # service clients
        self.__start_listening_client = self.create_client(
            Empty, "start_listening")
        self.__stop_listening_client = self.create_client(
            Empty, "stop_listening")
        self.__calibrating_client = self.create_client(
            Empty, "calibrate_listening")

        # pubs and subs
        self.__pub = self.create_publisher(StringArray, "stt_dialog", 10)

        self.subscription = self.create_subscription(
            StringArray,
            "stt_parse",
            self.__stt_callback,
            10)

        # action server
        self.__action_server = ActionSingleServer(self,
                                                  ListenOnce,
                                                  "listen_once",
                                                  execute_callback=self.__execute_server
                                                  )

    def destroy_node(self):
        """ destroy node and action server """

        self.__action_server.destroy()
        super().destroy_node()

    def __stt_callback(self, msg: StringArray):
        """ final speech calback

        Args:
            msg (StringArray): list of tags
        """

        self.get_logger().info("Dialog Manager: " + str(msg.strings))
        self.__pub.publish(msg)

        if not self.is_new_msg:
            self.new_msg = msg
            self.is_new_msg = True

    def calibrate_stt(self):
        """ calibrate stt method """

        req = Empty.Request()
        self.__calibrating_client.wait_for_service()
        self.__calibrating_client.call(req)
        self.get_logger().info("calibrating stt")

    def start_stt(self):
        """ start stt method """

        req = Empty.Request()
        self.__start_listening_client.wait_for_service()
        self.__start_listening_client.call(req)
        self.get_logger().info("starting stt")

    def stop_stt(self):
        """ stop stt method """

        req = Empty.Request()
        self.__stop_listening_client.wait_for_service()
        self.__stop_listening_client.call(req)
        self.get_logger().info("stopping stt")

    def __execute_server(self, goal_handle) -> ListenOnce.Result:
        """ action server execute callback

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
        while(not self.is_new_msg and not self.__action_server.is_canceled()):
            self.get_logger().info("Waiting for msg")
            time.sleep(0.5)

        # stoping stt
        self.stop_stt()

        # results
        result = ListenOnce.Result()

        if self.__action_server.is_canceled():
            self.__action_server.wait_for_canceling()
            goal_handle.canceled()

        else:
            result.stt_strings = self.new_msg.strings
            goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    node = DialogManagerNode()

    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
