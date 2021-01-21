""" ROS2 Node to Parse Speech """

from typing import List
import rclpy
import ament_index_python

from std_msgs.msg import String
from ros2_speech_recognition_interfaces.msg import StringArray

from jsgf import parse_grammar_file

from rclpy.node import Node


class ParserNode(Node):  # pylint: disable=too-few-public-methods
    """ Parser Node Class """

    def __init__(self):
        super().__init__("parser_node")

        # loading params
        grammar_param_name = "grammar"

        self.declare_parameter(grammar_param_name, ament_index_python.get_package_share_directory(
            "ros2_speech_recognition") + "/grammars/example.gram")

        self.grammar = self.get_parameter(
            grammar_param_name).get_parameter_value().string_value

        self.jsgf_grammar = parse_grammar_file(self.grammar)

        # pubs and subs
        self.__pub = self.create_publisher(StringArray, "stt_parse", 10)

        self.subscription = self.create_subscription(
            String,
            "stt_nlp",
            self.__parse,
            10)

    def __parse(self, msg: String):
        """ parser callback

        Args:
            msg (String): msg with text to parse
        """

        data = msg.data
        new_msg = StringArray()
        new_msg.strings = self.parse(data)
        self.__pub.publish(new_msg)

    def parse(self, data: str) -> List[str]:
        """ parse text into a list of str (tags)

        Args:
            data (str): text to parse

        Returns:
            List[str]: list of tags

        """

        rule = self.jsgf_grammar.find_matching_rules(data)
        tag_list = []
        if rule:
            for tag in rule[0].matched_tags:
                tag_list.append(tag)

        self.get_logger().info("Parser: " + str(tag_list))
        return tag_list


def main(args=None):
    rclpy.init(args=args)

    node = ParserNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
