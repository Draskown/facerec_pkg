import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from mngr_facerec import FaceRecognizer

class ImgSubscriber(Node):
    def __init__(self) -> None:
        """
        Create a imgSubscriber class, which is a subclass of the Node class.
        """
        super().__init__("img_sub")

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        _ = self.create_subscription(
            Image, 
            "frames", 
            self.__callback, 
            10,
        )

        # Create the publisher. This publisher will send the detected user id
        # to the detected_user_id topic. The queue size is 10 messages.
        self.__publisher = self.create_publisher(
            String,
            "recognized_name",
            10,
        )
        
        # Create a string of the last greeted person to compare later
        self.__last_greeted = None

        # Used to convert between ROS and OpenCV images
        self.__br = CvBridge()

        # Initialize the path to the data
        path = os.path.dirname(os.path.realpath(__file__))

        # Create an instance of the recognizer class
        self.__fr = FaceRecognizer(os.path.join(path, "Images"),
                                   os.path.join(path, "encodings.pkl"),
                                   os.path.join(path, "users.json"),
                                   )

    def __callback(self, 
                   data: Image,
                   ) -> None:
        """
        Callback function.
        """ 
        # Convert ROS Image message to OpenCV image
        current_frame = self.__br.imgmsg_to_cv2(data)
        
        # Recognize the face on the received image
        self.__fr.recognize_faces(current_frame)

        # Get recognized face
        recognized_id = self.__fr.get_user_id()

        # Skip if no face has been detected or
        # If the person has recently been greeted
        if recognized_id == "-2" or self.__last_greeted == recognized_id:
            return
        
        # If the person is unknown
        # Create them
        if recognized_id == "-1":
            self.__fr.create_user(current_frame)
            return

        # Load .json data
        json_data = self.__fr.get_json_data()
        # Greet the user
        self.__last_greeted = recognized_id

        msg = String()
        msg.data = json_data[recognized_id]["name"]

        self.__publisher.publish(msg)


def main(args: list=None) -> None:    
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    node = ImgSubscriber()

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
