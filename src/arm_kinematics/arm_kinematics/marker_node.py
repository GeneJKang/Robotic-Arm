from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
import rclpy



class MarkerNode(Node):

    def __init__(self):
        super().__init__('marker_node')
        self.server = InteractiveMarkerServer(self, 'target_marker')
        marker = InteractiveMarker()
        marker.header.frame_id = 'base_link'
        marker.name = 'target'
        marker.scale = 0.3
        marker.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        marker.pose.position.x = 0.3
        marker.pose.position.y = 0.2
        marker.pose.position.z = 0.8

        axes = [
            ('x', 0.7071, 0.7071, 0.0, 0.0),
            ('y', 0.7071, 0.0, 0.0, 0.7071),
            ('z', 0.7071, 0.0, 0.7071, 0.0)
        ]

        for name, w, x, y, z in axes:
            orientation = (w, x, y ,z)

            #Create Move control
            move_control = create_control(
                name=f"move_{name}",
                interaction_mode=InteractiveMarkerControl.MOVE_AXIS,
                orientation_tuple=orientation
            )
            marker.controls.append(move_control)

            #Create and append rotate control
            rotate_control = create_control(
                name=f"rotate_{name}",
                interaction_mode=InteractiveMarkerControl.ROTATE_AXIS,
                orientation_tuple=orientation
            )
            marker.controls.append(rotate_control)

        #Insert Marker into server and apply changes
        self.server.insert(marker, feedback_callback=self.feedback_callback)
        self.server.applyChanges()

        self.pub = self.create_publisher(PoseStamped, '/target_pose', 10)

    def feedback_callback(self, feedback):
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = feedback.pose
        self.pub.publish(msg)



def create_control(name, interaction_mode, orientation_tuple):
    #Helper function to build configured InteractiveMarkerControl
    control = InteractiveMarkerControl()
    control.name = name
    control.interaction_mode = interaction_mode
    control.always_visible = True

    w, x, y, z = orientation_tuple
    control.orientation = Quaternion(w=w, x=x, y=y, z=z)
    return control

def main():
    rclpy.init()
    node = MarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()