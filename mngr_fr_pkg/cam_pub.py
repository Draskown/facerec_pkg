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
      
    # We will publish a message every 0.1 seconds
    timer_period = 0.1  # seconds
      
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
         
    # Create a VideoCapture object
    self.cap = cv2.VideoCapture()
         
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def timer_callback(self) -> None:
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    self.cap.open("http://192.168.2.137:8000/")
    
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()
          
    if ret == True:
      # Publish the image.
      # The "cv2_to_imgmsg" method converts an OpenCV
      # image to a ROS 2 image message
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
  
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