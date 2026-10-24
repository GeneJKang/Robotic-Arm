import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from arm_kinematics.ForwardKinematics import fk
from arm_kinematics.InverseKinematics import ik
import numpy as np


class IKNode(Node):

    def __init__(self):
        super().__init__('ik_node') #Registers node under 'ik_node' as the name
        #Create a publisher as self.pub
        self.pub = self.create_publisher(JointState, '/joint_commands', 10)   #Create a publisher that sends JointState messages under /joint_commands as the name with 10 queue depth (messages to buffer)
        
        #Compute target with fk(...) and run ik() on it and store as self.angles
        target = fk(0, 0, 0, 0, 0, 0)
        self.angles = ik(target)    #Store on self so timer_callback can access it
        
        #Create the timer pointing at self.timer_callback
        self.timer = self.create_timer(0.01, self.timer_callback)    #0.01 period in secods so 100 times per second, self.timer_callback is what ROS uses to call each time


        #Create the subscriber
        self.sub = self.create_subscription(PoseStamped, '/target_pose', self.pose_callback, 10)    #mirrors publisher function. what kind of name doesnt matter but it is important to be the same name as publisher to connect

    def publish_angles(self):
         #Make a JointState(), fill in the header.stap, name, position
        msg = JointState()  #Create empty message
        msg.header.stamp = self.get_clock().now().to_msg()  #Get current ROS time and .to_msg() converts to a message
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'] #joint names from URDF. Must match exactly
        msg.position = [float(a) for a in self.angles]  #Must be plain floats and ik() returns tuples
        self.pub.publish(msg)



    def timer_callback(self):
        #publish it with self.pub
        self.publish_angles()

    def pose_callback(self, msg):
        #Pull out positiion p as a numpy array of x y and z
        p = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        #Convert orientation to 3x3 rotation matrix R
        q = msg.pose.orientation
        R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        #Build 4x4 target transformation matrix
        target_matrix = np.identity(4)
        target_matrix[0:3, 0:3] = R #Place R on top left 3x3
        target_matrix[0:3, 3] = p   #Place p in top right of matrix

        result = ik(target_matrix) #Store just in case if its None
        if (result is None):    #if None then reuse previous valid tuple
            self.get_logger().warn('Target out of reach', throttle_duration_sec=1.0)
            return self.publish_angles()
        else:
            self.angles = result #update movement
            self.publish_angles()


def main():
    rclpy.init()
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
