#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, QuaternionStamped

from nav_msgs.srv import SetMap
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


from math import radians, floor, sin, cos
from PIL import Image
from io import BytesIO

import numpy as np
import requests
import math
import cv2

from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Vector3,Quaternion,Vector3Stamped, QuaternionStamped
from sensor_msgs.msg import Imu, NavSatFix, MagneticField

from math import radians, floor, sin, cos, atan2, asin
from tf_transformations import quaternion_from_euler

from tf2_ros import StaticTransformBroadcaster
import tf2_ros
import sys
from rclpy.qos import QoSProfile, DurabilityPolicy

def scale(A, B, k):
    Y = A.shape[0]
    X = A.shape[1]
    for y in range(0, k):
        for x in range(0, k):
            A[y:Y:k, x:X:k] = B

def numpy_to_occupancy_grid(arr, ori, res, info=None):
        if not len(arr.shape) == 2:
                raise TypeError('Array must be 2D')
        if not arr.dtype == np.int8:
                raise TypeError('Array must be of int8s')
        # arr = np.clip(arr, -128, 127)
        grid = OccupancyGrid()
        if isinstance(arr, np.ma.MaskedArray):
                # We assume that the masked value are already -1, for speed
                arr = arr.data
        flattened_arr = arr.ravel()
        # print("Flattened array (first 100 elements):", flattened_arr[:1000])

        grid.data = arr.ravel().tolist()
        grid.info = info or MapMetaData()
        grid.info.height = arr.shape[0]
        grid.info.width = arr.shape[1]
        grid.info.origin = ori
        grid.info.resolution = res


        return grid




def compute_dist_constant(zoom_level, latitude):
    latitude_in_rad = latitude * np.pi / 180
    dconst = 40075016.686 / (2**(zoom_level + 8)) * np.cos(latitude_in_rad)
    return dconst

def tile_meter_to_pixel(meter):
    resolution = 0.2908986275853943
    pixel = floor(meter / resolution)
    return pixel

def hddeg2num(lat_deg, lon_deg, zoom):
    lat_rad = radians(lat_deg)
    n = 2.0 ** zoom
    xtile = (lon_deg + 180.0) / 360.0 * n
    ytile = (1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n
    return (xtile, ytile)

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
    return (xtile, ytile)

def getImageCluster(lat_deg, lon_deg, delta_lat, delta_long, zoom):
    smurl = r"https://tile.openstreetmap.org/{0}/{1}/{2}.png"
    xact, yact = hddeg2num(lat_deg, lon_deg, zoom)
    xmin, ymax = deg2num(lat_deg - delta_lat, lon_deg - delta_long, zoom)
    xcur, ycur = deg2num(lat_deg, lon_deg, zoom)
    xmax, ymin = deg2num(lat_deg + delta_lat, lon_deg + delta_long, zoom)

    xmin = xcur - 2
    ymin = ycur - 2
    xmax = xcur + 2
    ymax = ycur + 2

    origin_x, origin_y = int((xact % 1) * 256), int((yact % 1) * 256)
    origin_x = ((xcur - xmin) * 256) + origin_x
    origin_y = 256 * (ymax - ymin + 1) - (((ycur - ymin) * 256) + origin_y)

    Cluster = Image.new('RGB', ((xmax - xmin + 1) * 256, (ymax - ymin + 1) * 256))
    for xtile in range(xmin, xmax + 1):
        for ytile in range(ymin, ymax + 1):
            try:
                headers = {
                    'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/58.0.3029.110 Safari/537.3',
                    'Referer': 'https://tile.openstreetmap.org/'
                }
                imgurl = smurl.format(zoom, xtile, ytile)
                print("Opening: " + imgurl)
                imgstr = requests.get(imgurl, headers=headers)
                tile = Image.open(BytesIO(imgstr.content))
                Cluster.paste(tile, box=((xtile - xmin) * 256, (ytile - ymin) * 256))
            except Exception as e:
                print(e)
                print("Couldn't download image")
                tile = None

    return Cluster, origin_x, origin_y

def createImageMask(img):
    color1 = np.asarray([0, 0, 255])  # LL
    color2 = np.asarray([255, 255, 255])  # UL
    SCALE_CONST = (2048 / img.shape[0])
    mask = cv2.inRange(img, color1, color2)
    mask = cv2.resize(mask.astype(np.uint8), dsize=(2048, 2048), interpolation=cv2.INTER_LINEAR)
    mask = cv2.medianBlur(mask, 11)
    processed_mask = process_mask(mask)
    return processed_mask, SCALE_CONST

def process_mask(masked):
    processed_mask = np.flip(masked, 0)
    processed_mask = 100 * (processed_mask < 200).astype(np.int8)
    return processed_mask.astype(np.int8)

class GPS_PF_ROS(Node):

    

    def __init__(self) -> None:
        super().__init__('tilemap_pf')

        # Replace with your desired initial GPS coordinates and origin
        # self.init_gps_lat = 0 # Latitude
        # self.init_gps_long = 0 # Longitude
        self.lat = None
        self.long = None
        self.mag_heading = None
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        ori = 0
        res = 0
        # return self.lat, self.long
        
        # self.gnss_sub = self.create_subscription(Vector3Stamped, '/filter/positionlla', self.init_map_origin, 10) 
        self.gnss_sub = self.create_subscription(NavSatFix, '/gnss', self.init_map_origin, 10)
        self.mapinfo = MapMetaData()
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', qos_profile)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.set_map_client = self.create_client(SetMap, '/map_server/set_map')
        self.mag_sub = self.create_subscription(QuaternionStamped, '/filter/quaternion', self.mag_callback, 10)

        
        self.timer = self.create_timer(1.0, self.check_gnss_data)
        print("Node initialized, waiting for GNSS data...")

    def init_map_origin(self,msg):
            # self.lat = msg.vector.x
            # self.long = msg.vector.y
            self.lat = msg.latitude
            self.long = msg.longitude
            print(f"Initial GNSS coordinates received: lat={self.lat}, long={self.long}")
        
            self.destroy_subscription(self.gnss_sub)
    def mag_callback(self, msg):
        # Extract X and Y magnetometer values to calculate heading
        quat = msg.quaternion
        roll, pitch, yaw = self.quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)

        # Calculate heading in radians
        heading_rad = yaw
        
        # Convert heading to degrees and store it
        self.mag_heading = math.degrees(heading_rad)
        self.get_logger().info(f"Magnetic Heading: {self.mag_heading:.2f} degrees")

            
    
    def check_gnss_data(self):
        # Ensure lat and long are set before proceeding
            if self.lat is not None and self.long is not None:
                print("GNSS data received. Proceeding with map loading...")
            
            # Cancel the timer as GNSS data is now available
                self.timer.cancel()

            # Proceed with map loading
                a, self.cx, self.cy = getImageCluster(self.lat, self.long, 0.0015,  0.0015, 18)
                self.mask = np.zeros((5110, 5110), dtype=np.int8)
                print(f"Map image downloaded. Center coordinates: cx={self.cx}, cy={self.cy}")

                self.mask, SCALE_CONST = createImageMask(np.asarray(a))
                print(f"Processed mask shape: {self.mask.shape}")

            # Set map resolution based on scale constant
                
                self.compute_map_origin()
                # self.mapinfo.resolution = self.dconst / SCALE_CONST
                self.quat_sub = self.create_subscription(Imu, '/imu/data', self.quat_to_rpy, 10)
                
            
            # Start publishing the map
                self.map_pub_timer = self.create_timer(0.1, self.publish_map)
                print("Initialization complete.\n")

    # def get_map_image(self):
    #     # Download and process map image cluster from OpenStreetMap
    #     a, self.cx, self.cy = getImageCluster(self.init_gps_lat, self.init_gps_long, 0.0015, 0.0015, 19)
    #     print("Downloaded map image cluster.")
    #     return np.asarray(a), self.cx, self.cy

    # def process_mask(self, mask):
    #     print("Processing mask...")
    #     # color1 = np.asarray([0, 0, 255])  # LL
    #     # color2 = np.asarray([255, 255, 255])  # UL
    #     # SCALE_CONST = (2048 / img.shape[0])
    #     # mask = cv2.inRange(img, color1, color2)
    #     # mask = cv2.resize(mask.astype(np.uint8), dsize=(2048, 2048), interpolation=cv2.INTER_LINEAR)
    #     # mask = cv2.medianBlur(mask, 11)
    #     processed_mask = process_mask(mask)
    #     print("Mask processed.")
    #     return processed_mask #, SCALE_CONST
    def quat_to_rpy(self, msg):
        q = msg.orientation
        # roll, pitch, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        # self.get_logger().info(f"heading={yaw}")

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        # print(f"Map origin computed: x={self.mapinfo.origin.position.x}, y={self.mapinfo.origin.position.y}")
        # t.transform.translation.x = self.lat
        # t.transform.translation.y = self.long
        
            # Default to just using the IMU data if no magnetometer heading is available
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        self.tf_broadcaster.sendTransform(t)


    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(t3, t4)

        return roll, pitch, yaw
    
    

  

    def compute_map_origin(self):
        self.dconst = compute_dist_constant(zoom_level=18, latitude=self.lat)
        print(self.dconst)
        # origin_x_meter = tile_meter_to_pixel(dconst * (self.long + 180) / 360)
        # origin_y_meter = tile_meter_to_pixel(dconst * (1 - math.log(math.tan(radians(self.lat)) + (1 / math.cos(radians(self.lat))))) / math.pi / 2)
        
        self.mapinfo.origin.position.x = -float(self.cx* self.dconst)
        self.mapinfo.origin.position.y = -float(self.cy * self.dconst)
        self.get_logger().info(f"Map Origin: Lat: {self.lat:.9f}, Long: {self.long:.9f} degrees")

    # def compute_map_origin(self):
    # # Compute the distance constant for the zoom level and latitude
    #     self.dconst = compute_dist_constant(zoom_level=18, latitude=self.lat)

    # # Compute the world position for the map origin using the GPS coordinates (lat, lon)
    # # Latitude and longitude are converted to world coordinates (in meters) based on the zoom level.
    
    # # Convert the center tile coordinates (cx, cy) to world coordinates (in meters)
    #     world_x = self.cx * self.dconst
    #     world_y = self.cy * self.dconst

    # # The map origin (self.mapinfo.origin) should represent the world coordinates (latitude, longitude).
    # # Map the world coordinates of the center (cx, cy) to the map's coordinate system:
    #     self.mapinfo.origin.position.x = world_x - (self.dconst * (self.long + 180) / 360)  # Adjust based on the longitude
    #     self.mapinfo.origin.position.y = world_y - (self.dconst * (1 - math.log(math.tan(radians(self.lat)) + (1 / math.cos(radians(self.lat))))) / math.pi / 2)  # Adjust based on the latitude

    #     self.get_logger().info(f"Map Origin: Lat: {self.lat:.9f}, Long: {self.long:.9f} degrees")

    # def compute_map_origin(self):
    # # Directly use latitude and longitude as the map origin
    #     self.mapinfo.origin.position.x = self.lat  # Longitude as x
    #     self.mapinfo.origin.position.y = self.long   # Latitude as y
    #     self.mapinfo.origin.position.z = 0.0        # Flat map (z = 0)
    #     self.mapinfo.resolution = 4e-1  # Set resolution (degrees per grid cell)
    #     # print(f"Map origin set to lat={self.lat}, lon={self.long} (degrees).")
    #     self.get_logger().info(f"Map Origin: Lat: {self.lat:.7f}, Long: {self.long:.7f} degrees")

    def publish_map(self):
    # Create the occupancy grid message
        
        message = numpy_to_occupancy_grid(self.mask, self.mapinfo.origin, self.mapinfo.resolution, self.mapinfo)
        message.header.stamp = self.get_clock().now().to_msg()  # Update the timestamp
        message.header.frame_id = 'map'
        # message.child_frame_id = 'map'
    
    # Publish the map
        self.map_pub.publish(message)
        self.get_logger().info(f"Map Origin: Lat: {self.mapinfo.origin.position.x:.7f}, Long: {self.mapinfo.origin.position.y:.7f} degrees")
        self.get_logger().info(f"Published occupancy grid with length: {len(message.data)}")

    # Set the static map using the SetMap service
        # if self.set_map_client.wait_for_service(timeout_sec=1.0):
        #     future = self.set_map_client.call_async(SetMap.Request(map=message))
        #     rclpy.spin_until_future_complete(self, future)
        #     if future.result() is not None:
        #        print("Successfully set static map")
        #     else:
        #         print("Failed to set static map")
        # else:
        #     print("SetMap service is not available.")
    # def map_2_base_tf(self):
    #     t = TransformStamped()

    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = 'map'
    #     t.child_frame_id = 'base_link'

    #     t.transform.rotation.x = 

        

    def process_mask_to_occupancy_grid(self, mask):
    # Flatten the mask if it's a 2D array
        print("Converting mask to occupancy grid data...")
        mask_flat = mask.flatten()
        occupancy_grid_data = [
        100 if pixel == 100 else -1  # Set to -1 for unknown pixels
        for pixel in mask_flat
        ]
        print("Conversion complete.")
        return occupancy_grid_data


def main(args=None):
    rclpy.init(args=args)
    print("Starting ROS node...")

    node = GPS_PF_ROS()

    rclpy.spin(node)

    rclpy.shutdown()
    print("Node shutdown.")


if __name__ == '__main__':
    main()