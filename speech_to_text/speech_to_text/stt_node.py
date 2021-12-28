""" ROS2 Node to make STT """

import time

import speech_recognition as sr

import rclpy
import ament_index_python

from std_msgs.msg import String
from std_srvs.srv import Empty
from speech_to_text.custom_thread import CustomThread

from simple_node import Node


class STTNode(Node):  # pylint: disable=too-many-instance-attributes
    """ STT Node Class """

    def __init__(self):

        super().__init__("stt_node")

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
            "speech_to_text") + "/grammars/example.gram")

        self.declare_parameter(started_param_name, True)

        self.service = self.get_parameter(
            service_param_name).get_parameter_value().string_value
        self.grammar = self.get_parameter(
            grammar_param_name).get_parameter_value().string_value
        self.started = self.get_parameter(
            started_param_name).get_parameter_value().bool_value

        if self.started:
            self.calibrate_stt(2)
            self._start_stt()

        # result stt publisher
        self.__pub = self.create_publisher(String, "stt", 10)

        # service servers
        self.__start_server = self.create_service(
            Empty, "start_listening", self.__start_stt_srv)
        self.__stop_server = self.create_service(
            Empty, "stop_listening", self.__stop_stt_srv)
        self.__calibrate_server = self.create_service(
            Empty, "calibrate_listening", self.__calibrate_stt_srv)

    # CALIBRATE
    def __calibrate_stt_srv(self,
                            req: Empty.Request,
                            res: Empty.Response) -> Empty.Response:  # pylint: disable=unused-argument
        """ calibrate service

        Args:
            req (Empty.Request): empty
            res (Empty.Response): empty

        Returns:
            Empty.Response: empty
        """

        self.calibrate_stt(2)
        return res

    def calibrate_stt(self, seconds: int):
        """ calibrate noise

        Args:
            seconds (int): seconds to check noise
        """

        rec = sr.Recognizer()
        mic = sr.Microphone()
        self.get_logger().info("A moment of silence, please...")
        with mic as source:
            rec.adjust_for_ambient_noise(source, duration=seconds)
        self._energy_threshold = rec.energy_threshold  # Saving energy threshold
        self.__rec.energy_threshold = self._energy_threshold
        self.get_logger().info("Set minimum energy threshold to " + str(self._energy_threshold))

    # LISTEN
    def listen_from_mic(self):
        """ listen from mic """

        while self.started and rclpy.ok():
            self.get_logger().info("Threshold " + str(self.__rec.energy_threshold))
            self.get_logger().info("Say something!")
            with self.__mic as source:
                audio = self.__rec.listen(source)

                # Saving threshold to reuse it after using the start ros service
                self._energy_threshold = self.__rec.energy_threshold
                self.get_logger().info("Got it! Now to recognize it...")

                stt_result = String()

                try:
                    if self.service == "sphinx":
                        value = self.__rec.recognize_sphinx(
                            audio, grammar=self.grammar)
                    elif self.service == "google":
                        value = self.__rec.recognize_google(audio)

                    if value is bytes:  # bytes for strings (Python 2)
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

    def __listen_stt_thread_cb(self):
        """ thread callback to listen """

        try:
            self.get_logger().info("listen_thread starts listening")
            self.listen_from_mic()
        finally:
            self.get_logger().info("listen_thread ends")

    # START
    def __start_stt_srv(self,
                        req: Empty.Request,
                        res: Empty.Response) -> Empty.Response:  # pylint: disable=unused-argument
        """ service to start listen

        Args:
            req(Empty.Request): empty
            res(Empty.Response): empty

        Returns:
            Empty.Response: empty
        """

        self.start_stt()
        return res

    def start_stt(self):
        """ start listen """

        if not self.started:
            self.started = not self.started
            self._start_stt()
        else:
            self.get_logger().info("stt is already running")

    def _start_stt(self):
        """ start listen with a thread"""

        while(self.__listen_thread is not None and self.__listen_thread.is_alive()):
            time.sleep(0.1)

        self.__rec.energy_threshold = self._energy_threshold
        self.__listen_thread = CustomThread(
            target=self.__listen_stt_thread_cb)
        self.__listen_thread.start()
        self.get_logger().info("start listening, Threshold " +
                               str(self.__rec.energy_threshold))

    # STOP
    def __stop_stt_srv(self,
                       req: Empty.Request,
                       res: Empty.Response) -> Empty.Response:  # pylint: disable=unused-argument
        """ service to stop listen

        Args:
            req(Empty.Request): empty
            res(Empty.Response): empty

        Returns:
            Empty.Response: empty
        """

        self.stop_stt()
        return res

    def stop_stt(self):
        """ stop listen """

        if self.started:
            self.started = not self.started
            self._stop_stt()
        else:
            self.get_logger().info("stt is already stopped")

    def _stop_stt(self):
        """ stop listen with a thread """

        if(self.__listen_thread is not None and self.__listen_thread.is_alive()):
            self.__listen_thread.terminate()
        self.get_logger().info("stop listening, Threshold " +
                               str(self.__rec.energy_threshold))


def main(args=None):
    rclpy.init(args=args)

    node = STTNode()

    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
