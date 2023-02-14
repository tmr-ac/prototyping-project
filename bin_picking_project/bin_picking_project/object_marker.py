import rclpy
from rclpy.node import Node
from numpy import mat
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion

class Objects():
    """ class to define objects"""    
    def __init__(self, object_id, shape, color, x, y):
        # define instance attributes of class
        self.object_id = object_id
        self.shape = shape
        self.color = color
        self.x = float(x)
        self.y = float(y)
        self.r = None
        self.g = None
        self.b = None

        # call method to compute rgb values
        self.set_colors()

    def set_colors(self):
        # set rgb values of objects
        if self.color == "yellow":
            self.r = 1.0
            self.g = 1.0
            self.b = 0.0
        elif self.color == "red":
            self.r = 1.0
            self.g = 0.0
            self.b = 0.0
        elif self.color == "pink":
            self.r = 1.0
            self.g = 0.0
            self.b = 1.0
        else:
            self.r = 0.0
            self.g = 1.0
            self.b = 0.0


class ObjectMarkers(Node):
    """ class to publish object markers"""    
    def __init__(self, name):
        super().__init__(name)

        # create publisher for marker array message
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        # create subscriber for marker coordinates
        self.object_sub = self.create_subscription(String, "detected_objects", self.object_callback, 10)

        # create subscriber for inverse kinematics feedback
        self.feedback_sub = self.create_subscription(String, "grasp_feedback", self.feedback_callback, 10)

        # create empty list for objects and MarkerArray() message
        self.objects = []
        self.array = MarkerArray()
        
        # define target locations for objects
        self.target_triangle = mat("0.675; -0.28; 0.417")
        self.target_circle = mat("0.535; -0.42; 0.417")
        self.target_square = mat("0.745; -0.49; 0.417")
        self.target = mat("0.0; 0.0; 0.0")

        # output info msg that object marker publisher is online
        self.get_logger().info(f"Object marker publisher is on...")

    def object_callback(self, msg):
        # split string message from object detection
        self.object_data = msg.data.split()
        self.split_objects = [' '.join(self.object_data[i:i+4]) for i in range(0,len(self.object_data),4)]

        # create objects for marker placement from first message of object_detection (only runs once so markers are not overwritten)
        if len(self.objects) == 0:
            for i, item in enumerate(self.split_objects):
                self.split_msg = item.split(" ")
                self.objects.append(Objects(i, self.split_msg[0], self.split_msg[1], self.split_msg[2], self.split_msg[3]))

        # call method to create markers
        self.create_markers()

    def create_markers(self):
        # define markers from first message of object_detection (only runs once so markers are not overwritten)
        if len(self.array.markers) == 0:
            # iterate over objects list and construct markers
            for i, item in enumerate(self.objects):
                if item.shape == "triangle":
                    new_object = Marker(type = Marker.MESH_RESOURCE, mesh_resource = "package://bin_picking_project/environment/triangle.dae")
                elif item.shape == "square":
                    new_object = Marker(type = Marker().CUBE)
                elif item.shape == "circle":
                    new_object = Marker(type = Marker().CYLINDER)
                new_object.header.frame_id = "world"
                new_object.id = item.object_id
                new_object.action = Marker().ADD
                new_object.color = ColorRGBA(r = item.r, g = item.g, b = item.b, a = 1.0)
                new_object.pose = Pose(position = Point(x = item.x, y = item.y, z = 0.43), orientation = Quaternion(x = 0.0, y = 0.0, z = -0.707, w = 0.707))
                new_object.scale = Vector3(x = 0.07, y = 0.07, z = 0.07)
                self.array.markers.append(new_object)

        # create timer and call marker publisher method
        self.timer = self.create_timer(0, self.publish_marker)

    def feedback_callback(self, msg):
        # adjust target position based on shape of grasped object
        picked_object = int(msg.data[0])
        if "triangle" in msg.data:
            self.target = self.target_triangle
        elif "circle" in msg.data:
            self.target = self.target_circle
        else:
            self.target = self.target_square
            self.target_square = self.target_square + mat("-0.07 ; 0.07 ; 0.0")

        # adjust positions and frame_ids of objects, depending on grasp feedback of inverse kinematics node
        if "grasped" in msg.data:
            self.array.markers[picked_object].header.frame_id = "body6"
            self.array.markers[picked_object].pose.position.x = 0.0
            self.array.markers[picked_object].pose.position.y = 0.0
            self.array.markers[picked_object].pose.position.z = -0.07 / 2
        elif "placed" in msg.data:
            self.array.markers[picked_object].header.frame_id = "world"
            self.array.markers[picked_object].pose.position.x = float(self.target[0])
            self.array.markers[picked_object].pose.position.y = float(self.target[1])
            self.array.markers[picked_object].pose.position.z = float(self.target[2])


    def publish_marker(self):
        # publish the marker array message
        self.marker_pub.publish(self.array)


def main(args=None):
    # initialize ROS2
    rclpy.init(args=args)

    # create the marker_publisher node
    object_markers = ObjectMarkers('object_marker_publisher')

    # spin to keep the node alive
    try:
        rclpy.spin(object_markers)

    except KeyboardInterrupt:
        print("Quitting the object marker publisher node...")
        object_markers.destroy_node()


if __name__ == '__main__':
    main()