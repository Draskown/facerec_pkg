import sys
import time
import rclpy
import pygame
from gtts import gTTS
from io import BytesIO
from rclpy.node import Node
from std_msgs.msg import String

class GreetUser(Node):
    def __init__(self) -> None:
        """
        Create a GreetUser class, which is a subclass of the Node class.
        """
        super().__init__("greet")

        # Create the subscriber. This subscriber will receive an String
        # from the detected_user_id topic. The queue size is 10 messages.
        _ = self.create_subscription(
            String,
            "recognized_name", 
            self.__callback, 
            10,
        )

        # Initialize pygame and pygame mixer instances
        pygame.init()
        pygame.mixer.init()

    def __callback(self, 
                    msg: String,
                    ) -> None:
        """
        Callback function.
        """ 
        
        # Greet the user
        self.__speak_text("Здравствуйте, " + msg.data)

    def __speak_text(self, text):
        """
        Voices the passed text

        Args:
        - text: the text to voice
        """
        # Create an io object to store the voicing
        io = BytesIO()
        # Create a voicer instance
        tts = gTTS(text, lang="ru")
        # Write generated audio into the io
        tts.write_to_fp(io)
        # Initialize the first input of the io
        io.seek(0)

        # Create a sound object of the io
        sound = pygame.mixer.Sound(io)
        # Voice the text
        sound.play()

        # Wait until the whole text is said
        while pygame.mixer.get_busy():
            time.sleep(1)


def main(args: list=None) -> None:
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    node = GreetUser()

    # Spin the node so the callback function is called.
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()