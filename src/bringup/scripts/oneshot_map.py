#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as RclpyNode
from interfaces.msg import Map  # replace with actual msg import
import json

objects = [
  {
    "cls": 0,
    "bbox": {
        "center": { 
               "position": { "x": 2, "y": 2, "z": 0 },
               "orientation": { "w": 1, "x": 0, "y": 0, "z": 0 }
        },
        "size": { "x": 1, "y": 1, "z": 1 }
    }
  }
]
map_msg = {
    "map_bounds": {
        "center": { 
               "position": { "x": 1000 / 2, "y": 1000 / 2, "z": 1000 / 2 },
               "orientation": { "w": 1, "x": 0, "y": 0, "z": 0 },
        },
        "size": { "x": 1000, "y": 1000, "z": 1000 },
    },
    "objects": objects,
    "new_object_indexes": list(range(len(objects)))
}


# minimal publisher node
class OneshotMap(RclpyNode):
    def __init__(self):
        super().__init__("oneshot_map_node")
        self.pub = self.create_publisher(Map, "/hydrus/map", 1)
        msg = Map()
        # fill msg fields from map_msg_data
        # assuming Map supports something like from_json
        # msg.deserialize(json.dumps(map_msg_data))
        self.pub.publish(msg)
        self.get_logger().info("Published map_msg once")
        rclpy.shutdown()  # exit after publishing

def main(args=None):
    rclpy.init(args=args)
    node = OneshotMap()
    rclpy.spin(node)
