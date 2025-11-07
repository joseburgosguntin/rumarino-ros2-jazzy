#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import time

from interfaces.msg import Map, MapObject
from vision_msgs.msg import BoundingBox3D
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3


def make_bbox(center_pos, center_ori, size_xyz):
    pose = Pose()
    pose.position = Point(x=center_pos["x"], y=center_pos["y"], z=center_pos["z"])
    pose.orientation = Quaternion(
        w=center_ori["w"], x=center_ori["x"], y=center_ori["y"], z=center_ori["z"]
    )
    size = Vector3(
        x=size_xyz["x"], y=size_xyz["y"], z=size_xyz["z"]
    )
    return BoundingBox3D(center=pose, size=size)


class OneshotMap(Node):
    def __init__(self):
        super().__init__("oneshot_map_node")
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(Map, "/hydrus/map", map_qos)

    def publish_once(self):
        # --- build the map msg ---
        msg = Map()

        # map bounds
        # All values need to be explicitly set as floats, otherwise 
        # an int ends up being parsed as a float in the mission 
        # executor. That is very bad.
        msg.map_bounds = make_bbox(
            center_pos={"x": 0.0, "y": 0.0, "z": 0.0},
            center_ori={"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            size_xyz={"x": 1000.0, "y": 1000.0, "z": 3.0},
        )

        # objects (e.g., one gate)
        gate = MapObject()
        gate.cls = 2
        gate.bbox = make_bbox(
            center_pos={"x": 0.0, "y": 0.0, "z": 3.0},
            center_ori={"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            size_xyz={"x": 0.04, "y": 3 + (2 * 0.04), "z": 4.0},
        )
        msg.objects.append(gate)

        # publish
        self.pub.publish(msg)
        self.get_logger().info("Published map_msg once")


# def main(args=None):
#     rclpy.init(args=args)
#     node = OneshotMap()
#     node.publish_once()
#
#     # give DDS time to flush message
#     time.sleep(1.0)
#
#     node.destroy_node()
#     rclpy.shutdown()
#

def main(args=None):
    rclpy.init(args=args)
    node = OneshotMap()
    node.publish_once()

    # Keep the node alive so subscribers can receive the message
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
