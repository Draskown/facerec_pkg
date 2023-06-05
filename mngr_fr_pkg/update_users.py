import os
import rclpy
from rclpy.node import Node
from mngr_fr_pkg.mngr_facerec import FaceRecognizer
 
class UpdateUsers(Node):
    """
    Creates an UpdateUsers class, which is a subclass of the Node class.
    """
    def __init__(self) -> None:
        # Initiate the Node class"s constructor and give it a name
        super().__init__("update_users")

        # Initialize the path to the data
        path = os.path.dirname(os.path.realpath(__file__))

        # Create an instance of the recognizer class
        self.__fr = FaceRecognizer(os.path.join(path, "Images"),
                                   os.path.join(path, "encodings.pkl"),
                                   os.path.join(path, "users.json"),
                                )
        
    def update_users(self) -> None:
       self.__fr.update_users()
  
def main(args: list=None) -> None:
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    node = UpdateUsers()

    # Update the users from the file
    node.update_users()
    node.get_logger().info("Updated users")
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == "__main__":
    main()