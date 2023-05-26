import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImagePublisher(Node):
  """
  Creates an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self) -> None:
    # Initiate the Node class's constructor and give it a name
    super().__init__("cam_pub")
      
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, "frames", 10)

    # Create the timer to publish the message ewvery 0.1 seconds
    _ = self.create_timer(0.1, self.__timer_callback)
         
    # Create a VideoCapture object
    self.cap = cv2.VideoCapture()
         
    # Used to convert between ROS and OpenCV images
    self.__br = CvBridge()

    # Declare parameters for camera address and index
    # and use them in global fields
    self.declare_parameter("cam_src_index", -1)
    self.declare_parameter("cam_src_http", "http://192.168.2.135:8000/")
    self.__cam_src_int = self.get_parameter("cam_src_index")
    self.__cam_src_str = self.get_parameter("cam_src_http")

  def __check_index(self) -> int or str:
    """
    Checks whether for which camera source to use:
    provided by index or by http address
    """
    if self.__cam_src_int.value != -1:
      return self.__cam_src_int.value
    else:
      return self.__cam_src_str.value

  def __timer_callback(self) -> None:
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    self.cap.open(self.__check_index())
    
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()
          
    if ret == True:
      # Publish the image.
      # The "cv2_to_imgmsg" method converts an OpenCV
      # image to a ROS 2 image message
      self.publisher_.publish(self.__br.cv2_to_imgmsg(frame))
  
def main(args: list=None) -> None:
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = ImagePublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)

  image_publisher.cap.release()

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == "__main__":
  main()
