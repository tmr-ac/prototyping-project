import rclpy
import imutils
import cv2
import os
from .image_processing.shapedetector    import ShapeDetector
from .image_processing.colorlabeler     import ColorLabeler
from rclpy.node             import Node
from rcl_interfaces.msg     import ParameterDescriptor
from std_msgs.msg           import String
from ament_index_python.packages    import get_package_share_path


class ObjectPublisher(Node):
    """ class to publish object shapes, colors and coordinates from given images """    
    def __init__(self,name):
        super().__init__(name)

        # create object coordinate publisher
        self.object_pub = self.create_publisher(String, "detected_objects", 10)

        # declare parameter for image selection
        param_descriptor = ParameterDescriptor(description="sets image file to select")
        self.declare_parameter("img_file", "", param_descriptor)
        self.img = self.get_parameter("img_file").value

        # create timer with detect_shapes callback
        self.period = 1
        self.timer = self.create_timer(self.period, self.detect_shapes)

        #output info msg that object coordinate publisher is online
        self.get_logger().info(f"Object detection publisher is on...")

    def detect_shapes(self):
        # select image file for object detection
        package_path = get_package_share_path('bin_picking_project')
        image_path = os.path.join(package_path, self.img)
        
        # load image and resize to smaller factor for better approximation
        image = cv2.imread(image_path)
        resized = imutils.resize(image, width = 300)
        ratio = image.shape[0] / float(resized.shape[0])

        # convert image to grayscale, blur slightly and threshold
        blurred = cv2.GaussianBlur(resized,(5, 5), 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)[1]

        # find contours in thresholded image 
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # initialize shape detector, color labeler, object list and definition
        sd = ShapeDetector()
        cl = ColorLabeler()
        object_list = []
        object_definition = ""
        square_counter = 0

        # loop over contours
        for c in cnts:
            # compute center, detect name of contour
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cX = int((M["m01"] / M["m00"]) * ratio)
            cY = int((M["m10"] / M["m00"]) * ratio)

            # detect shape of contour and label of color
            shape = sd.detect(c)
            color = cl.label(lab, c)

            # compute distances from pixels to meters (1 pixel = 0.026458333 cm)
            cX = cX * 0.02646 / 100
            cY = cY * 0.02646 / 100

            # compute object coordinates globally, based on box corner of marker box (x = 0.806, y = 0.48)
            cX = 0.806 - cX 
            cY = 0.48 - cY 

            # define object with properties and append to list
            object_definition = " ".join([shape, color, str(cX), str(cY)])
            object_list.append(object_definition)

        # publish objects to detected_objects topic
        objects = String()
        objects.data = " ".join([str(item) for item in object_list])
        self.object_pub.publish(objects)


def main(args = None):
    # initialize ROS2
    rclpy.init()

    # create the object_publisher node
    object_publisher = ObjectPublisher("object_publisher")

    # spin to keep the node alive
    try:
        rclpy.spin(object_publisher)

    except KeyboardInterrupt:
        print("Quitting the object coordinate publisher node...")
        object_publisher.destroy_node()

if "__name__" == "__main__":
    main()