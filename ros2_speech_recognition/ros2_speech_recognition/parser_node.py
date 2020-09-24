'''ROS node to parse speech'''

import os

import rclpy
import ament_index_python
from rclpy.node import Node

from std_msgs.msg import String
from ros2_speech_recognition_interfaces.msg import StringArray

from jsgf import parse_grammar_file


class ParserNode(Node):  # pylint: disable=too-few-public-methods
    '''ParserNode class'''

    def __init__(self):
        super().__init__('parser_node')

        # loading params
        grammar_param_name = "grammar"

        self.declare_parameter(grammar_param_name, ament_index_python.get_package_share_directory(
            "ros2_speech_recognition") + '/grammars/example.gram')

        self.grammar = self.get_parameter(
            grammar_param_name).get_parameter_value().string_value

        self.loaded_grammar = parse_grammar_file(self.grammar)

        # pubs and subs
        self.__pub = self.create_publisher(StringArray, 'stt_parse', 10)

        self.subscription = self.create_subscription(
            String,
            'stt_nlp',
            self.__parse,
            10)
        self.subscription  # prevent unused variable warning

    def __parse(self, msg):
        '''Method cb'''

        data = msg.data
        new_msg = StringArray()
        new_msg.strings = self.parse(data)
        self.__pub.publish(new_msg)

    def parse(self, data):
        '''Method to do the parse'''

        rule = self.loaded_grammar.find_matching_rules(data)
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


if __name__ == '__main__':
    main()
