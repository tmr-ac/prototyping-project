from ament_index_python.packages       import get_package_share_path

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions                import Node
from launch.substitutions              import Command, LaunchConfiguration
from launch.actions                    import DeclareLaunchArgument, ExecuteProcess
from launch                            import LaunchDescription

import os


# The following function is mandatory
def generate_launch_description():

    # Defining the object, which must be returned
    ld = LaunchDescription()

    package_path = get_package_share_path('bin_picking_project')

    urdf_model_path  = os.path.join(package_path, 'urdf_model/robot_model.urdf')

    rviz_config_path = os.path.join(package_path, 'rviz/config.rviz')

    params_config_path  = os.path.join(package_path, 'config/params1.yaml')

    #sim_time = LaunchConfiguration('use_sim_time', default = 'false')

    model_arg = DeclareLaunchArgument(name          = 'model',
                                      default_value = str(urdf_model_path),
                                      description   = "ABB 1200 6dof robot model")

    rviz_arg = DeclareLaunchArgument(name          = 'rvizconfig',
                                     default_value = str(rviz_config_path),
                                     description   = "Rviz configuration file")

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type = str)

    # Configure the object detection node
    object_detection_node = Node(package    = "bin_picking_project",
                                 executable = "object_detection",
                                 output     = "screen",
                                 parameters = [params_config_path],
                                 namespace  = None)


    # Configure the inverse kinematics node
    inverse_kinematics_node = Node(package    = "bin_picking_project",
                                  executable = "inverse_kinematics",
                                  output     = "screen",
                                  namespace  = None)


    # Configure the environment_marker publisher node
    env_marker_node = Node(package    = "bin_picking_project",
                            executable = "environment_marker",
                            output     = "screen",
                            namespace  = None)


    # Configure the object_marker publisher node
    obj_marker_node = Node(package    = "bin_picking_project",
                            executable = "object_marker",
                            output     = "screen",
                            namespace  = None)
    

    # Configure the image publisher node
    image_node = Node(package    = "bin_picking_project",
                        executable = "image_publisher",
                        output     = "screen",
                        parameters = [params_config_path],
                        namespace  = None)


    # Configure the robot_state_publisher
    robot_state_node = Node(package    = "robot_state_publisher",
                            executable = "robot_state_publisher",
                            name       = "robot_state_publisher",
                            output     = "screen",
                            parameters = [{'robot_description': robot_description}])


    # Configure RViz for visualization
    rviz_node = Node(package    = "rviz2",
                     executable = "rviz2",
                     output     = "screen",
                     namespace  = None,
                     arguments  = ["-d", LaunchConfiguration("rvizconfig")])

    ld.add_action(model_arg)
    ld.add_action(object_detection_node)
    ld.add_action(inverse_kinematics_node)
    ld.add_action(env_marker_node)
    ld.add_action(obj_marker_node)
    ld.add_action(image_node)
    ld.add_action(robot_state_node)
    ld.add_action(rviz_arg)
    ld.add_action(rviz_node)

    return ld