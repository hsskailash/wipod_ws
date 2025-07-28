#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
import pyroutelib3
import numpy as np
import math
from geographic_msgs.msg import GeoPoint

class OSMWaypointGenerator(Node):
    def __init__(self):
        super().__init__('osm_waypoint_generator')
        
        # Subscriber for end goal
        self.goal_sub = self.create_subscription(
            PoseStamped, '/end_goal', self.goal_callback, 10)
            
        # Publisher for waypoints
        self.waypoints_pub = self.create_publisher(Path, '/osm_waypoints', 10)
        
        # OSM Map loading
        self.graph = self.load_osm_map()
        
        # State variables
        self.current_pose_sub = None
        self.active_goal = None
        self.current_position = None
        self.interpolation_distance = 5.0  # Meters between waypoints

    def load_osm_map(self):
        """Load OSM map from PBF file"""
        map_path = "/home/bunty/Documents/map.pbf"
        self.get_logger().info(f"Loading OSM map from {map_path}")
        with open(map_path, "rb") as f:
            return pyroutelib3.osm.Graph.from_file(
                pyroutelib3.osm.CarProfile(), f)

    def goal_callback(self, msg):
        """Handle new goal and activate pose subscription"""
        if self.active_goal:
            self.get_logger().warn("Already processing a goal, ignoring new one")
            return
            
        # Store new goal
        self.active_goal = (
            msg.pose.position.x,  # Latitude
            msg.pose.position.y   # Longitude
        )
        
        # Create temporary pose subscription
        if not self.current_pose_sub:
            self.current_pose_sub = self.create_subscription(
                PoseStamped,
                '/gnss_pose',
                self.pose_callback,
                10
            )
            self.get_logger().info("Subscribed to current pose")

    def pose_callback(self, msg):
        """Process single pose and calculate route"""
        # Get current position
        self.current_position = (
            msg.pose.position.x,  # Latitude
            msg.pose.position.y   # Longitude
        )
        
        # Calculate route
        if self.active_goal:
            route = self.calculate_route(self.current_position, self.active_goal)
            if route:
                self.publish_waypoints(route)
            
            # Cleanup
            self.destroy_subscription(self.current_pose_sub)
            self.current_pose_sub = None
            self.active_goal = None
            self.get_logger().info("Route processed, unsubscribed from pose")

    def calculate_route(self, start_gps, end_gps):
        """Calculate route using pyroutelib3"""
        try:
            start_node = self.graph.find_nearest_node(start_gps)
            end_node = self.graph.find_nearest_node(end_gps)
            
            route = pyroutelib3.find_route(self.graph, start_node.id, end_node.id)
            return [self.graph.get_node(node).position for node in route] if route else None
            
        except Exception as e:
            self.get_logger().error(f"Routing failed: {str(e)}")
            return None

    def haversine(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS points in meters"""
        R = 6371000  # Earth radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = (math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * (math.sin(delta_lambda/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c

    def interpolate_points(self, points):
        """Add intermediate points between route nodes"""
        interpolated = []
        if not points:
            return interpolated
            
        interpolated.append(points[0])
        
        for i in range(len(points)-1):
            p1 = points[i]
            p2 = points[i+1]
            
            # Calculate segment distance
            distance = self.haversine(p1[0], p1[1], p2[0], p2[1])
            segments = max(1, int(distance / self.interpolation_distance))
            
            # Linear interpolation (for short distances)
            for j in range(1, segments):
                fraction = j / segments
                lat = p1[0] + fraction * (p2[0] - p1[0])
                lon = p1[1] + fraction * (p2[1] - p1[1])
                interpolated.append((lat, lon))
                
            interpolated.append(p2)
            
        return interpolated

    def calculate_yaw(self, p1, p2):
        """Calculate orientation from p1 to p2 in radians"""
        lat1, lon1 = p1
        lat2, lon2 = p2
        
        # Convert to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Calculate bearing
        dlon = lon2 - lon1
        x = math.cos(lat2) * math.sin(dlon)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(x, y)
        
        return bearing

    def quaternion_from_yaw(self, yaw):
        """Convert yaw angle to quaternion"""
        q = Quaternion()
        q.z = math.sin(yaw / 2)
        q.w = math.cos(yaw / 2)
        return q

    def publish_waypoints(self, route_points):
        """Convert route to Path message with orientations"""
        # Add intermediate points
        dense_points = self.interpolate_points(route_points)
        if not dense_points:
            self.get_logger().warn("No waypoints generated")
            return
            
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'gps'
        
        # Create poses with orientations
        for i in range(len(dense_points)):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = dense_points[i][0]
            pose.pose.position.y = dense_points[i][1]
            
            # Calculate orientation based on next point
            if i < len(dense_points) - 1:
                yaw = self.calculate_yaw(dense_points[i], dense_points[i+1])
                pose.pose.orientation = self.quaternion_from_yaw(yaw)
            else:  # For last point, use previous orientation
                pose.pose.orientation = self.quaternion_from_yaw(
                    self.calculate_yaw(dense_points[i-1], dense_points[i]))
                
            path_msg.poses.append(pose)
            
        self.waypoints_pub.publish(path_msg)
        self.get_logger().info(f"Published {len(path_msg.poses)} waypoints")

def main(args=None):
    rclpy.init(args=args)
    node = OSMWaypointGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()

if __name__ == '__main__':
    main()