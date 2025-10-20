#!/usr/bin/env python3
"""
RViz Visualization Node for HydrusCV MapObject messages.

This node subscribes to MapObject messages and publishes visualization markers
for RViz2, including:
- 3D bounding boxes for detected objects
- Text labels with class names and confidence
- Color-coded markers by object class
"""

import rclpy
from rclpy.node import Node
from interfaces.msg import Map, MapObject
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
import math


class MapVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')
        
        # Declare parameters
        self.declare_parameter('map_topic', 'detections')
        self.declare_parameter('marker_topic', 'object_markers')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('marker_lifetime', 1.0)  # seconds
        
        # Get parameters
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        marker_lifetime = self.get_parameter('marker_lifetime').get_parameter_value().double_value
        
        # Create subscriber and publisher
        self.map_sub = self.create_subscription(
            MapObject,
            map_topic,
            self.map_callback,
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            marker_topic,
            10
        )
        
        # Store marker lifetime
        self.marker_lifetime_sec = int(marker_lifetime)
        self.marker_lifetime_nsec = int((marker_lifetime - self.marker_lifetime_sec) * 1e9)
        
        # Define color map for different object classes
        self.class_colors = {
            'gate': ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7),      # Green
            'buoy': ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.7),      # Orange
            'shark': ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7),     # Red
            'swordfish': ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.7), # Blue
            'person': ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7),    # Yellow
            'bottle': ColorRGBA(r=0.5, g=0.0, b=0.5, a=0.7),    # Purple
            'boat': ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.7),      # Cyan
            'default': ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.7)    # Gray
        }
        
        self.marker_id = 0
        
        self.get_logger().info(f'Map Visualizer started')
        self.get_logger().info(f'  Subscribing to: {map_topic}')
        self.get_logger().info(f'  Publishing markers to: {marker_topic}')
        self.get_logger().info(f'  World frame: {self.world_frame}')
    
    def map_callback(self, msg: MapObject):
        """Process incoming MapObject message and create visualization markers."""
        marker_array = MarkerArray()
        
        # Create bounding box marker
        bbox_marker = self.create_bbox_marker(msg)
        marker_array.markers.append(bbox_marker)
        
        # Create text label marker
        label_marker = self.create_label_marker(msg)
        marker_array.markers.append(label_marker)
        
        # Publish markers
        self.marker_pub.publish(marker_array)
    
    def create_bbox_marker(self, map_obj: MapObject) -> Marker:
        """Create a 3D bounding box marker for the detected object."""
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'bounding_boxes'
        marker.id = self.marker_id
        self.marker_id += 1
        
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set position and orientation from bounding box
        marker.pose = map_obj.bbox.center
        
        # Set size from bounding box
        marker.scale = map_obj.bbox.size
        
        # Set color based on class
        marker.color = self.class_colors.get(map_obj.cls, self.class_colors['default'])
        
        # Set lifetime
        marker.lifetime.sec = self.marker_lifetime_sec
        marker.lifetime.nanosec = self.marker_lifetime_nsec
        
        return marker
    
    def create_label_marker(self, map_obj: MapObject) -> Marker:
        """Create a text label marker showing class name above the object."""
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'labels'
        marker.id = self.marker_id
        self.marker_id += 1
        
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position text above the bounding box
        marker.pose.position.x = map_obj.bbox.center.position.x
        marker.pose.position.y = map_obj.bbox.center.position.y
        marker.pose.position.z = map_obj.bbox.center.position.z + map_obj.bbox.size.z / 2.0 + 0.3
        marker.pose.orientation.w = 1.0
        
        # Set text content
        marker.text = f"{map_obj.cls}"
        
        # Set text size
        marker.scale.z = 0.2  # Text height
        
        # Set text color (white with full opacity)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        # Set lifetime
        marker.lifetime.sec = self.marker_lifetime_sec
        marker.lifetime.nanosec = self.marker_lifetime_nsec
        
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = MapVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
