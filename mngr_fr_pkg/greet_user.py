import sys
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
            "recognized_name", 
            self.__callback, 
            10,
        )

    def __callback(self, 
                    msg: String,
                    ) -> None:
        """
        Callback function.
        """ 
        
        # Greet the user
        sys.stdout.write("Hello there, " + msg.data + "\n")


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