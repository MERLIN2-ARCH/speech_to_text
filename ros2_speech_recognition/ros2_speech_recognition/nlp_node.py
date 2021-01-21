""" ROS2 Node for NLP """

import rclpy

from std_msgs.msg import String

import nltk
import contractions

from custom_ros2 import Node


class NLPNode(Node):  # pylint: disable=too-few-public-methods
    """ NLP Node Class """

    def __init__(self):
        super().__init__("nlp_node")

        # pubs and subs
        self.__pub = self.create_publisher(String, "stt_nlp", 10)

        self.subscription = self.create_subscription(
            String,
            "stt",
            self.__do_nlp,
            10)

    def __do_nlp(self, msg: String):
        """ nlp callback

        Args:
            msg (String): text to do nlp
        """

        data = msg.data
        new_msg = String()
        new_msg.data = self.do_nlp(data)
        self.__pub.publish(new_msg)

    def do_nlp(self, data: str) -> str:
        """ apply nlp to a text

        Args:
            data (str): text to do nlp

        Returns:
            str: text after nlp
        """

        nlp_s = ""
        if data:
            sentences = nltk.sent_tokenize(contractions.fix(data))
            tags = nltk.pos_tag(nltk.word_tokenize(sentences[0]))
            self.get_logger().info(str(tags))

            for tag in tags:
                if(tag[1] == "TO"  # Take the word if is a to  # pylint: disable=too-many-boolean-expressions
                   # or a prepositional or subordinating conjuntion
                   or tag[1] == "IN"
                   or tag[1] == "CD"  # or a cardinal number
                   or tag[1][0] == "N"  # or a noum
                   or tag[1][0] == "J"  # or a adjetive
                   or tag[1][0] == "P"  # or a pronoum
                   or tag[1][0] == "V"  # or a verb
                   or tag[1][0] == "W"):  # or a interrogative pronoun

                    nlp_s += tag[0]+" "
        self.get_logger().info("NLP: " + str(nlp_s))
        return nlp_s


def main(args=None):
    rclpy.init(args=args)

    node = NLPNode()

    node.join_spin()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
