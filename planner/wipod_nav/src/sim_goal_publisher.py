#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        self.goal_pub = self.create_publisher(PoseStamped, '/end_goal',10)
        self.lat = 12.924051
        self.long = 77.498644
        # Create timer to publish at 1Hz
        self.timer = self.timer_callback(self.lat,self.long)
        
        
        self.get_logger().info("Pose Publisher Node Started!")
      

    def timer_callback(self,lat,long):
     # Create PoseStamped message
         msg = PoseStamped()
         # Set header
         msg.header.stamp = self.get_clock().now().to_msg()
         msg.header.frame_id = 'map'  # Coordinate frame
         # Set position (x, y, z)
         msg.pose.position.x = lat
         msg.pose.position.y = long
         msg.pose.position.z = 0.0
         # Set orientation (quaternion - identity orientation)
         msg.pose.orientation.w = 1.0
         # Publish message
         self.goal_pub.publish(msg)
         
         self.get_logger().info(f"Published pose")

def main(args=None):
    rclpy.init(args=args)
    
    pose_publisher = PosePublisher()
    
    try:
        rclpy.spin(pose_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        pose_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()