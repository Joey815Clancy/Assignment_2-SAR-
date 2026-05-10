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
        {'name': 'red_cylinder',   'colour': 'red',    'type': 'cylinder', 'meaning': 'fire',            'x': 0.0, 'y': 0.0, 'detected': False},
        {'name': 'blue_cube',      'colour': 'blue',   'type': 'cube',     'meaning': 'dam',             'x': 0.0, 'y': 0.0, 'detected': False},
        {'name': 'green_cuboid',   'colour': 'green',  'type': 'cuboid',   'meaning': 'survivor',        'x': 0.0, 'y': 0.0, 'detected': False},
        {'name': 'yellow_cube',    'colour': 'yellow', 'type': 'cube',     'meaning': 'medical_kit',     'x': 0.0, 'y': 0.0, 'detected': False},
        {'name': 'purple_cylinder','colour': 'purple', 'type': 'cylinder', 'meaning': 'exit',            'x': 0.0, 'y': 0.0, 'detected': False},
        {'name': 'black_square',   'colour': 'black',  'type': 'square',   'meaning': 'docking_station', 'x': 0.0, 'y': 0.0, 'detected': False},
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

    def get_colour_mask(self, hsv, colour):
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
        low, high = ranges[colour]
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
            mask = self.get_colour_mask(hsv,obj['colour'])

            # Object is detected if more than 500 pixels found, stays detected once seen
            if cv2.countNonZero(mask) > 500:
                obj['detected'] = True

            # creat your detection obj #
            detection = ObjectDetection()
            detection.name = obj['name']
            detection.colour = obj['colour']
            detection.type = obj['type']
            detection.meaning = obj['meaning']
            detection.detected = obj['detected']
            
            # if obj is in centre of frame, lidar is pointing at it so update position
            if detection.detected:
                M = cv2.moments(mask)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    frame_centre = frame.shape[1] // 2
                    if abs(cx - frame_centre) < 80:
                        distance = self.get_front_distance()
                        obj['x'] = self.robot_x + (distance * math.cos(self.robot_yaw))
                        obj['y'] = self.robot_y + (distance * math.sin(self.robot_yaw))

            detection.x = obj['x']
            detection.y = obj['y']

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


