import rclpy
import os
import cv2
import numpy as np
from rcl_interfaces.msg     import ParameterDescriptor
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_path


class ImagePublisher(Node):
    """ class to publish image for visualization in rviz """
    def __init__(self,name):
        super().__init__(name)

        # create publisher for image
        self.image_pub = self.create_publisher(Image, 'Image', 10)
        
        # declare parameter for image selection
        param_descriptor = ParameterDescriptor(description="sets image file to select")
        self.declare_parameter("img_file", "", param_descriptor)
        self.img = self.get_parameter("img_file").value

        # load pregenerated image
        self.package_path = get_package_share_path('bin_picking_project')
        self.image_path = os.path.join(self.package_path, self.img)
        self.cv_image = cv2.imread(self.image_path) 
        self.bridge = CvBridge()

        # create timer with publish callback
        self.period = 1.0
        self.timer = self.create_timer(self.period, self.publish_image)

        # output info msg that image publisher is online
        self.get_logger().info(f"Image publisher is on...")

    def publish_image(self):
        # publish the image message
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(np.array(self.cv_image), "bgr8"))


def main(args=None):
    # initialize ROS2
    rclpy.init(args=args)

    # create the image_publisher node
    image_publisher = ImagePublisher('image_publisher')

    # spin to keep the node alive
    try:
        rclpy.spin(image_publisher)

    except KeyboardInterrupt:
        print("Quitting the image publisher node...")
        image_publisher.destroy_node()


if __name__ == '__main__':
    main()