import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion


class EnvironmentPublisher(Node):
    """ class to publish environment markers (table and box) in rviz """
    def __init__(self, name):
        super().__init__(name)

        #create publishers for environment markers
        self.table_pub = self.create_publisher(Marker, 'table_marker', 10)
        self.box_pub = self.create_publisher(Marker, 'box_marker', 10)
        self.target_pub = self.create_publisher(Marker, 'target_marker', 10)

        #create table marker and define properties
        self.table_marker = Marker()
        self.table_marker.header.frame_id = "world"
        self.table_marker.header.stamp = self.get_clock().now().to_msg()
        self.table_marker.id = 0
        self.table_marker.type = Marker().MESH_RESOURCE
        self.table_marker.mesh_resource = "package://bin_picking_project/environment/env_table.dae"
        self.table_marker.action = Marker().ADD
        self.table_marker.pose = Pose(position = Point(x = 0.9, y = 0.0, z = 0.75 * 0.5), orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0))
        self.table_marker.scale = Vector3(x = 0.5, y = 0.3, z = 0.5)
        self.table_marker.color = ColorRGBA(r = 0.5, g = 0.35, b = 0.25, a = 1.0)

        #create box marker and define properties
        self.box_marker = Marker()
        self.box_marker.header.frame_id = "world"
        self.box_marker.header.stamp = self.get_clock().now().to_msg()
        self.box_marker.id = 0
        self.box_marker.type = Marker().MESH_RESOURCE
        self.box_marker.mesh_resource = "package://bin_picking_project/environment/env_box.dae"
        self.box_marker.action = Marker().ADD
        self.box_marker.pose = Pose(position = Point(x = 0.9, y = -0.2, z = 0.75 * 0.5), orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0))
        self.box_marker.scale = Vector3(x = 0.5, y = 0.4, z = 0.5)
        self.box_marker.color = ColorRGBA(r = 0.5, g = 0.5, b = 0.5, a = 1.0)

        #create target marker and define properties
        self.target_marker = Marker()
        self.target_marker.header.frame_id = "world"
        self.target_marker.header.stamp = self.get_clock().now().to_msg()
        self.target_marker.id = 0
        self.target_marker.type = Marker().MESH_RESOURCE
        self.target_marker.mesh_resource = "package://bin_picking_project/environment/target_position.dae"
        self.target_marker.action = Marker().ADD
        self.target_marker.pose = Pose(position = Point(x = 0.605, y = -0.35, z = 0.3855), orientation = Quaternion(x = 0.0, y = 0.0, z = -0.707, w = 0.707))
        self.target_marker.scale = Vector3(x = 0.07, y = 0.07, z = 0.07)
        self.target_marker.color = ColorRGBA(r = 0.5, g = 0.5, b = 0.5, a = 1.0)

        #create timer with publish callback
        self.period = 1.0   
        self.timer = self.create_timer(self.period, self.publish_environment)

        #output info msg that environment marker publisher is online
        self.get_logger().info(f"Environment marker publisher is on...")

    def publish_environment(self):
        # publish the table and box
        self.table_pub.publish(self.table_marker)
        self.box_pub.publish(self.box_marker)
        self.target_pub.publish(self.target_marker)


def main(args=None):
    # initialize ROS2
    rclpy.init(args=args)

    # create the env_publisher node
    env_publisher = EnvironmentPublisher('environment_publisher')

    # spin to keep the node alive
    try:
        rclpy.spin(env_publisher)

    except KeyboardInterrupt:
        print("Quitting the environment marker publisher node...")
        env_publisher.destroy_node()


if __name__ == "__main__":
    main()

