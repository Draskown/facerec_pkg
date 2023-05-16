import os
import sys
import json
import rclpy
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
            "detected_user_id", 
            self.__callback, 
            10,
        )
        
        self.__last_greeted = None

        # Initialize the path to the data
        path = os.path.dirname(os.path.realpath(__file__))

        # Load data from the json file
        with open(os.path.join(path, "users.json")) as f:
            self.__data = json.load(f)

    def __callback(self, 
                    msg: String,
                    ) -> None:
        """
        Callback function.
        """ 

        # Skip if no face has been detected or
        # If the person has recently been greeted
        # msg is None or msg.data is None or 
        if msg.data == "-2" \
              or msg.data <= "0" or self.__last_greeted == msg.data:
            print("Unknown user")

        # Greet the user
        sys.stdout.write("Hello there, " + self.__data[msg.data]["name"] + "\n")
        self.__last_greeted = msg.data


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