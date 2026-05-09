import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sar_msgs.msg import ObjectDetection, ObjectDetectionArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from rclpy.executors import ExternalShutdownException

class RobotPerception(Node):


    def __init__(self):
        super().__init__('robot_perception')

         ## Subscribe to camera ##

        self.sub_camera = self.create_subscription(Image,"/camera/image",self.image_callback,10)
        self.sub_Lidar = self.create_subscription(LaserScan,"/scan",self.lidar_callback,10)
        self.sub_odom = self.create_subscription(Odometry,"/odom",self.odom_callback,10)


        ## publish custom message ##
        self.pub = self.create_publisher(ObjectDetectionArray, '/detected_objects',10)

        ## Create bridge

        self.bridge = CvBridge()

        ## obj that will hold location of objects detected 
        self.objects = [
        {'name': 'red_cylinder',  'color': 'red',    'type': 'cylinder', 'meaning': 'fire',            'x': 0.0, 'y': 0.0},
        {'name': 'blue_cube',     'color': 'blue',   'type': 'cube',     'meaning': 'dam',             'x': 0.0,   'y': 0.0},
        {'name': 'green_cuboid',  'color': 'green',  'type': 'cuboid',   'meaning': 'survivor',        'x': 0.0,  'y': 0.0},
        {'name': 'yellow_cube',   'color': 'yellow', 'type': 'cube',     'meaning': 'medical_kit',     'x': 0.0,  'y': 0.0},
        {'name': 'purple_cylinder','color': 'purple', 'type': 'cylinder', 'meaning': 'exit',           'x': 0.0,   'y': 0.0},
        {'name': 'black_square',  'color': 'black',  'type': 'square',   'meaning': 'docking_station', 'x': 0.0, 'y': 0.0},
        ]

        ## delcare variables ## 
        self.scan = None 
        self.robot_x = 0.0 
        self.robot_y = 0.0
        self.robot_yaw = 0.0

    ### Function used to get distance info infront of robot ###

    def get_front_distance(self):
        n = len(self.scan.ranges) # gets all scan data 
        front = self.scan.ranges[n//2 - 15 : n//2 + 15] # reduces this to whats in front 
        valid = [r for r in front if not math.isinf(r) and not math.isnan(r)] #Ensures it dosnt return inf or nan
        return min(valid) if valid else 0.0 # Return the min value found i.e object in from 

    def lidar_callback(self,msg):
        self.scan = msg # Get msg Data

    def odom_callback(self,msg):
        # Get odom Data # 
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def get_colour_mask(self, hsv, color):
        # Find what colour is in front 
        # We use HSV to avoid issues with lighting 
        ranges = {
        'red':    [(0,   120,  70), (10,  255, 255)],
        'blue':   [(100, 150,  50), (130, 255, 255)],
        'green':  [(40,  70,   50), (80,  255, 255)],
        'yellow': [(20,  100, 100), (35,  255, 255)],
        'purple': [(130, 50,   50), (160, 255, 255)],
        'black':  [(0,   0,    0),  (180, 50,   50)],
    }
        low, high = ranges[color]
        return cv2.inRange(hsv, np.array(low), np.array(high))

    def image_callback(self,msg):
    
        if self.scan is None:
            return
        
        ## Get current frame ## 
        frame = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        ## Get msg ##
        array_msg = ObjectDetectionArray()

        for obj in self.objects:
            # Find colour mask #
            mask = self.get_colour_mask(hsv,obj['color'])

            # Object is dected if more than 500 pixels is detected #
            detected= cv2.countNonZero(mask) >500 
            
            # creat your detection obj #
            detection = ObjectDetection()
            detection.name = obj['name']
            detection.color = obj['color']
            detection.type = obj['type']
            detection.meaning = obj['meaning']
            detection.detected = detected
            
            # if obj is found get distance info #
            if detected: 
                distance = self.get_front_distance()
                detection.x = self.robot_x + (distance * math.cos(self.robot_yaw))
                detection.y = self.robot_y + (distance * math.sin(self.robot_yaw))
            else:
                detection.x= 0.0 
                detection.y= 0.0

            array_msg.objects.append(detection)

        self.pub.publish(array_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = RobotPerception()
        rclpy.spin(node)
    except ExternalShutdownException:
        pass

if __name__ == '__main__':
    main()


