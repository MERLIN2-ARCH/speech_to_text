'''ROS node dialog manger'''

import time
import threading
import collections
import asyncio

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ros2_speech_recognition_interfaces.msg import StringArray
from ros2_speech_recognition_interfaces.action import ListenOnce
from action_msgs.msg import GoalStatus
from std_srvs.srv import Empty


class DialogManagerNode(Node):
    '''DialogManagerNode class'''

    def __init__(self):
        super().__init__('dialog_manager_node')

        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._current_goal = None

        self.is_new_msg = False
        self.new_msg = None
        self.is_server_canceled = False

        # service clients
        self.__start_listening_client = self.create_client(
            Empty, 'start_listening')
        self.__stop_listening_client = self.create_client(
            Empty, 'stop_listening')
        self.__calibrating_client = self.create_client(
            Empty, 'calibrate_listening')

        # pubs and subs
        self.__pub = self.create_publisher(StringArray, 'stt_dialog', 10)

        self.subscription = self.create_subscription(
            StringArray,
            'stt_parse',
            self.stt_callback,
            10)
        self.subscription  # prevent unused variable warning

        # action server
        self._action_server = ActionServer(self,
                                           ListenOnce,
                                           'listen_once',
                                           execute_callback=self.__execute_server,
                                           cancel_callback=self.__cancel_server,
                                           handle_accepted_callback=self.__accepted_callback,
                                           )

    def destroy(self):
        ''' destroy node method'''

        self._action_server.destroy()
        super().destroy_node()

    def stt_callback(self, msg):
        '''Method to redirect stt data'''

        self.get_logger().info("Dialog Manager: " + str(msg.strings))
        self.__pub.publish(msg)

        if(not self.is_new_msg):
            self.new_msg = msg
            self.is_new_msg = True

    def __accepted_callback(self, goal_handle):
        '''action server accepted callback or defer execution of an already accepted goal'''

        with self._goal_queue_lock:
            if self._current_goal is not None:
                # Put incoming goal in the queue
                self._goal_queue.append(goal_handle)
                self.get_logger().info('Goal put in the queue')
            else:
                # Start goal execution right away
                self._current_goal = goal_handle
                self._current_goal.execute()

    def __cancel_server(self, goal_handle):
        '''action server cancel callback'''

        self.is_server_canceled = True
        self.get_logger().info("cancelling action server")
        return CancelResponse.ACCEPT

    async def calibrate_stt(self):
        '''calibrate stt method'''

        req = Empty.Request()
        self.__calibrating_client.wait_for_service()
        future = self.__calibrating_client.call_async(req)
        self.get_logger().info("calibrating stt")
        try:
            await future
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))

    async def start_stt(self):
        '''start stt method'''

        req = Empty.Request()
        self.__start_listening_client.wait_for_service()
        future = self.__start_listening_client.call_async(req)
        self.get_logger().info("starting stt")
        try:
            await future
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))

    async def stop_stt(self):
        '''stop stt method'''

        req = Empty.Request()
        self.__stop_listening_client.wait_for_service()
        future = self.__stop_listening_client.call_async(req)
        self.get_logger().info("stopping stt")
        try:
            await future
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))

    def __execute_server(self, goal_handle):
        '''action server execute callback'''

        try:
            self.is_new_msg = False
            self.is_server_canceled = False
            self.new_msg = StringArray()

            if(goal_handle.request.calibrate):
                asyncio.run(self.calibrate_stt())

            # starting stt
            asyncio.run(self.start_stt())

            # wait for message
            while(not self.is_new_msg and not self.is_server_canceled):
                self.get_logger().info("Waiting for msg")
                time.sleep(1)

            # stoping stt
            asyncio.run(self.stop_stt())

            # results
            result = ListenOnce.Result()

            if(goal_handle.status != GoalStatus.STATUS_CANCELED and
               goal_handle.status != GoalStatus.STATUS_CANCELING):
                result.stt_strings = self.new_msg.strings
                goal_handle.succeed()
            else:
                goal_handle.canceled()

            return result

        finally:
            with self._goal_queue_lock:
                try:
                    # Start execution of the next goal in the queue.
                    self._current_goal = self._goal_queue.popleft()
                    self.get_logger().info('Next goal pulled from the queue')
                    self._current_goal.execute()
                except IndexError:
                    # No goal in the queue.
                    self._current_goal = None


def main(args=None):
    rclpy.init(args=args)

    node = DialogManagerNode()

    executor = MultiThreadedExecutor()

    rclpy.spin(node, executor=executor)

    rclpy.spin(node)

    node.destroy()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
