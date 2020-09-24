'''ROS node to make stt'''

import os
import time

import speech_recognition as sr

import rclpy
import ament_index_python
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Empty

from ros2_speech_recognition.custom_thread import CustomThread


class STTNode(Node):  # pylint: disable=too-many-instance-attributes
    '''STTNode class'''

    def __init__(self):

        super().__init__('stt_node')

        self.__listen_thread = None

        self.__rec = sr.Recognizer()
        self.__mic = sr.Microphone()
        self.__rec.dynamic_energy_threshold = True  # Update the threshold after listen
        self._energy_threshold = 0

        # loading params
        service_param_name = "service"
        grammar_param_name = "grammar"
        started_param_name = "started"

        self.declare_parameter(service_param_name, "sphinx")
        self.declare_parameter(grammar_param_name, ament_index_python.get_package_share_directory(
            "ros2_speech_recognition") + '/grammars/example.gram')

        self.declare_parameter(started_param_name, True)

        self.service = self.get_parameter(
            service_param_name).get_parameter_value().string_value
        self.grammar = self.get_parameter(
            grammar_param_name).get_parameter_value().string_value
        self.started = self.get_parameter(
            started_param_name).get_parameter_value().bool_value

        if self.started:
            self.calibrate(2)
            self._start()

        # result stt publisher
        self.__pub = self.create_publisher(String, 'stt', 10)

        # service servers
        self.__start_server = self.create_service(
            Empty, 'start_listening', self.__start_srv)
        self.__stop_server = self.create_service(
            Empty, 'stop_listening', self.__stop_srv)
        self.__calibrate_server = self.create_service(
            Empty, 'calibrate_listening', self.__calibrate_srv)

    # CALIBRATE
    def __calibrate_srv(self, req, res):  # pylint: disable=unused-argument
        '''Mthod callback to calibrate'''

        self.calibrate(2)
        return res

    def calibrate(self, segundos):
        '''Mthod to calibrate'''

        rec = sr.Recognizer()
        mic = sr.Microphone()
        self.get_logger().info("A moment of silence, please...")
        with mic as source:
            rec.adjust_for_ambient_noise(source, duration=segundos)
        self._energy_threshold = rec.energy_threshold  # Saving energy threshold
        self.__rec.energy_threshold = self._energy_threshold
        self.get_logger().info("Set minimum energy threshold to " + str(self._energy_threshold))

    # LISTEN
    def listen(self):
        '''Method to listen'''

        while self.started:
            self.get_logger().info("Threshold " + str(self.__rec.energy_threshold))
            self.get_logger().info("Say something!")
            with self.__mic as source:
                audio = self.__rec.listen(source)

                # Saving threshold to reuse it after using the start ros service
                self._energy_threshold = self.__rec.energy_threshold
                self.get_logger().info("Got it! Now to recognize it...")

                stt_result = String()

                try:
                    if self.service == 'sphinx':
                        value = self.__rec.recognize_sphinx(
                            audio, grammar=self.grammar)
                    elif self.service == 'google':
                        value = self.__rec.recognize_google(audio)

                    if str is bytes:  # bytes for strings (Python 2)
                        self.get_logger().info(format(value).encode("utf-8"))
                        stt_result.data = format(value).encode("utf-8")
                    else:  # unicode for strings (Python 3+)
                        self.get_logger().info(format(value))
                        stt_result.data = format(value)

                except sr.UnknownValueError:
                    stt_result.data = "UnknownValueError"

                except sr.RequestError as error:
                    stt_result.data = "Couldn't request results from Google STT; " + \
                        str(error)

                finally:
                    self.__pub.publish(stt_result)

    def __listen_thread_function(self):
        '''Method callback of thread'''

        try:
            self.get_logger().info("listen_thread starts listening")
            self.listen()
        finally:
            self.get_logger().info("listen_thread ends")

    # START
    def __start_srv(self, req, res):  # pylint: disable=unused-argument
        '''Method callback to start listen'''

        self.start()
        return res

    def start(self):
        '''Method to start listen'''

        if not self.started:
            self.started = not self.started
            self._start()
        else:
            self.get_logger().info("stt is already running")

    def _start(self):
        '''Method protected to start listen'''

        while(self.__listen_thread != None and self.__listen_thread.is_alive()):
            time.sleep(1)

        self.__rec.energy_threshold = self._energy_threshold
        self.__listen_thread = CustomThread(
            target=self.__listen_thread_function)
        self.__listen_thread.start()
        self.get_logger().info("start_listening, Threshold " +
                               str(self.__rec.energy_threshold))

    # STOP
    def __stop_srv(self, req, res):  # pylint: disable=unused-argument
        '''Method callback to stop listen'''

        self.stop()
        return res

    def stop(self):
        '''Method to stop listen'''

        if self.started:
            self.started = not self.started
            self._stop()
        else:
            self.get_logger().info("stt is already stopped")

    def _stop(self):
        '''Method protected to stop listen'''

        if(self.__listen_thread != None and self.__listen_thread.is_alive()):
            self.__listen_thread.terminate()
        self.get_logger().info("stop_listening, Threshold " +
                               str(self.__rec.energy_threshold))


def main(args=None):
    rclpy.init(args=args)

    node = STTNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
