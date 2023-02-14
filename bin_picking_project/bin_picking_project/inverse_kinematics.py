import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from numpy import mat, zeros, eye, cos, sin, block, sign, arccos, arcsin, arctan, transpose
from math import sqrt, atan2, pi
from time import sleep


class InverseKinematics(Node):
    """ inverse kinematics and joint state publisher class """
    def __init__(self, name):
        super().__init__(name)

        # create publisher for JointStates
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # create subscriber for detected_objects 
        self.object_sub = self.create_subscription(String,'detected_objects', self.ik_callback, 10)

        # create publisher for feedback of grasped objects 
        self.feedback_pub = self.create_publisher(String, 'grasp_feedback', 10)

        # define home position of robot
        self.rot_1 = self.rot_2 = self.rot_3 = self.rot_4 = self.rot_5 = self.rot_6 = 0.0

        # create JointState message
        self.joint_state = JointState()
        self.joint_state.name = ["rotation1", "rotation2", "rotation3", "rotation4", "rotation5", "rotation6"]

        # create timer to publish joint states
        self.timer = self.create_timer(0.1, self.publish_joints)

        # define safe points for robot movement (collision prevention)
        self.safepoint1 = [-1.30, 0.4, -0.4, 0.0, 0.0, 0.0]
        self.safepoint2 = [-2.10, 0.6, -0.6, 0.0, 0.0, 0.0]

        # define target locations for objects (on the table next to the box)
        self.targetpoint_triangle = mat("0.675; -0.28; 0.452")
        self.targetpoint_circle = mat("0.535; -0.42; 0.452")
        self.targetpoint_square = mat("0.675; -0.42; 0.452")
        self.targetpoint = mat("0.0; 0.0; 0.0")
        self.target_joints = []
    
        # create state object, define lists for objects, shapes and goal positions and create step counter for robot movement
        self.state_=self.State()
        self.goal_positions = []
        self.objects = []
        self.objects_shape = []
        self.step_counter = 1

        # output info msg that inverse_kinematics is online
        self.get_logger().info(f"Inverse kinematics is on...")

    def ik_callback(self,msg):
        # split string message from object detection
        self.object_data = msg.data.split()
        self.objects = [' '.join(self.object_data[i:i+4]) + " 0.465" for i in range(0,len(self.object_data),4)]

        # create objects for robot movement
        for i in self.objects:
            self.split_objects = i.split()
            self.state_.endeffector_position=mat(f"{self.split_objects[2]}; {self.split_objects[3]}; {self.split_objects[4]}")
            self.goal_positions.append(self.state_.inverse_kinematics())
            self.objects_shape.append(self.split_objects[0])

        # execute movement sequence between the objects
        self.movement_sequence()

    def movement_sequence(self):
        # iterate over the objects goal_positions
        for index, gp in enumerate(self.goal_positions):
            # create feedback message for object markers
            feedback = String()

            # change targetpoint depending on shape
            if self.objects_shape[index] == "triangle":
                self.targetpoint = self.targetpoint_triangle
            elif self.objects_shape[index] == "circle":
                self.targetpoint = self.targetpoint_circle
            else:
                self.targetpoint = self.targetpoint_square
                self.targetpoint_square = self.targetpoint_square + mat("-0.14 ; 0.14 ;0.0")

            # compute inverse kinematics for target location
            self.state_.endeffector_position = mat(f"{self.targetpoint[0]}; {self.targetpoint[1]}; {self.targetpoint[2]}")
            self.target_joints = self.state_.inverse_kinematics()
        
            # execute movements from initial to safepoint 1
            initial = [self.rot_1, self.rot_2, self.rot_3, self.rot_4, self.rot_5, self.rot_6]
            self.move_joints(self.safepoint1, initial)

            # execute movement from safepoint 1 to the object, then publish feedback that object can be grasped
            self.move_joints(gp, self.safepoint1)
            feedback.data = str(index) + " grasped" + self.objects_shape[index]
            self.feedback_pub.publish(feedback)

            # execute movements from object to safepoint 1, then to safepoint 2
            self.move_joints(self.safepoint1, gp)
            self.move_joints(self.safepoint2, self.safepoint1)

            # execute movement from safepoint 2 to target position, then pusblish feedback that object can be released
            self.move_joints(self.target_joints, self.safepoint2)
            feedback.data = str(index) + " placed" + self.objects_shape[index]
            self.feedback_pub.publish(feedback)

            # execute movement from target position to safepoint 2
            self.move_joints(self.safepoint2, self.target_joints)

        # shutdown node after first complete sequence is finished
        rclpy.shutdown()

    def move_joints(self,qgoal,qinitial):
        # move robot from initial to target location in 50 steps
        while self.step_counter<=50:
            self.rot_1 += (qgoal[0] - qinitial[0]) / 50
            self.rot_2 += (qgoal[1] - qinitial[1]) / 50
            self.rot_3 += (qgoal[2] - qinitial[2]) / 50
            self.rot_4 += (qgoal[3] - qinitial[3]) / 50
            self.rot_5 += (qgoal[4] - qinitial[4]) / 50
            self.rot_6 += (qgoal[5] - qinitial[5]) / 50
            self.publish_joints()
            self.step_counter += 1
            sleep(0.05)

        # reset step_counter back to 0 after movement is finished
        self.step_counter = 0 

    def publish_joints(self):
        # publish the joint states
        self.time_now = self.get_clock().now().to_msg()
        self.joint_state.header.stamp = self.time_now
        self.joint_state.position = (self.rot_1, self.rot_2, self.rot_3, self.rot_4, self.rot_5, self.rot_6)
        self.joint_pub.publish(self.joint_state)


    class State():
        """ subclass to compute inverse kinematics """
        def __init__(self):

            # define geometric properties of robot (according to URDF)
            self.P0_w = mat("0.0; 0.0; 0.3991")
            self.rpy0_w = mat("0.0; 0.0; 0.0")
            self.P1_0 = mat("0.0; 0.0; 0.0")
            self.rpy1_0 = mat("0.0; 0.0; 1.5708")
            self.P2_1 = mat("0.0; 0.0; 0.448")
            self.rpy2_1 = mat("0.0;0.0;0.0")
            self.P3_2 = mat("0.29; 0.0; 0.042")
            self.rpy3_2 = mat("0.0; 1.5708; 0.0")
            self.P4_3 = mat("0.0; 0.0; 0.161")
            self.rpy4_3 = mat("0.0; 0.0; 0.0")
            self.P5_4 = mat("0.082; 0.0; 0.0")
            self.rpy5_4 = mat("0.0; -1.5708; 0.0")

            # create placeholder for rotation matrices
            self.Rq1 = self.Rq2 = self.Rq3 = self.Rq4 = self.Rq5 = mat(eye((3)))
            self.R0_w = self.R1_0 = self.R2_1 = self.R3_2 = self.R4_3 = mat(eye((3)))

            # compute rotation matrices for initial state
            self.endeffector_position = mat("0.0;0.0;0.0")
            self.endeffector_rpy = mat("0.0;0.0;0.0")
            self.initial()

        def euler2R(self,roll, pitch, yaw):
            # compute rotation matrix from euler angles and return rotation matrix
            sz = sin(yaw)
            sy = sin(pitch)
            sx = sin(roll)
            cz = cos(yaw)
            cy = cos(pitch)
            cx = cos(roll)
            R = mat([
                [cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
                [-sy,   cy*sx,          cy*cx]
            ])
            return R

        def initial(self):
            # compute initial rotation matrix
            self.R1_0 = self.euler2R(self.rpy1_0[0,0], self.rpy1_0[1,0], self.rpy1_0[2,0])
            self.R2_1 = self.euler2R(self.rpy2_1[0,0], self.rpy2_1[1,0], self.rpy2_1[2,0])
            self.R3_2 = self.euler2R(self.rpy3_2[0,0], self.rpy3_2[1,0], self.rpy3_2[2,0])
            self.R4_3 = self.euler2R(self.rpy4_3[0,0], self.rpy4_3[1,0], self.rpy4_3[2,0])
            self.R5_4 = self.euler2R(self.rpy5_4[0,0], self.rpy5_4[1,0], self.rpy5_4[2,0])
    
        def inverse_kinematics(self):
            # compute rotation matrix from world to frame joint 6    
            R5_w = self.euler2R(self.endeffector_rpy[0,0], self.endeffector_rpy[1,0], self.endeffector_rpy[2,0])
            
            # compute position of joint 5
            xc = self.endeffector_position[0,0] + self.P5_4[0,0]*R5_w[0,2]
            yc = self.endeffector_position[1,0] + self.P5_4[0,0]*R5_w[1,2]
            zc = self.endeffector_position[2,0] + self.P5_4[0,0]*R5_w[2,2]

            # compute relevant distances between joints (see project documentation)
            r = sqrt(xc * xc + yc * yc)
            d1 = self.P0_w[2,0] + self.P1_0[2,0]
            s = zc - d1
            rr = sqrt(r * r + s * s)
            d2 = self.P2_1[2,0]
            d4 = sqrt((self.P3_2[0,0] + self.P4_3[2,0])**2 + (self.P3_2[2,0])**2)

            # compute theta 1 - 4 and joint values for joint 1 - 3
            q1 = atan2(-xc, yc)
            theta1 = atan2(s, r)
            theta2 = arccos((rr * rr + d2 * d2 - d4 * d4) / (2 * d2 * rr))
            theta3 = atan2((self.P3_2[2,0]), self.P3_2[0,0] + self.P4_3[2,0])
            q2 = pi/2 - theta1 - theta2
            theta4 = arcsin((d2 * cos(q2) - s) / d4)
            q3 = - q2 + theta4 + theta3

            # compute rotation matrices Rq1, Rq2, Rq3, R2_w, R5_2 and Rnew
            Rq1 = self.euler2R(0, 0, q1)
            Rq2 = self.euler2R(0, q2, 0)
            Rq3 = self.euler2R(0, q3, 0)
            R2_w = self.R0_w * Rq1 * self.R1_0 * Rq2 * self.R2_1 * Rq3            
            R5_2 = transpose(R2_w) * R5_w
            Rnew = transpose(self.R3_2) * R5_2

            # compute q4, q5 and q6
            q5 = arcsin(Rnew[2,2])
            q6 = arccos(Rnew[2,0] / cos(q5))
            q4 = 0.0
            # q4 = arccos(-Rnew[0,2]/cos(q5)), q4 manually set to 0, as the calculation can cause Nan (not a number) errors (possibly singularity problems within the movement)
            
            goal_position=[q1, q2, q3, q4, q5, q6]
            return goal_position
            

def main(args=None):
    # initialize ROS2
    rclpy.init()

    # create the inverse_kinematics node
    inverse_kinematics = InverseKinematics("inverse_kinematics")

    # spin the node once
    try:
        rclpy.spin(inverse_kinematics)

    except KeyboardInterrupt:
        print("Quitting the inverse kinematics node...")
        inverse_kinematics.destroy_node()


if __name__ == '__main__':
    main()
