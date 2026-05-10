import py_trees
import math
from nav_msgs.msg import Odometry
from sar_msgs.msg import ObjectDetectionArray
from geometry_msgs.msg import Twist


class ScanDam(py_trees.behaviour.Behaviour):

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.dam_x = None
        self.dam_y = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.state = 'start'
        self.start_yaw = None

    def _wrap_angle(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def setup(self, **kwargs):
        self.node.create_subscription(ObjectDetectionArray, '/detected_objects', self.detection_callback, 10)
        self.node.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self):
        if self.dam_x is None:
            return py_trees.common.Status.RUNNING

        twist = Twist()

        if self.state == 'start':
            self.start_yaw = math.atan2(self.dam_y - self.robot_y, self.dam_x - self.robot_x)
            self.state = 'face_dam'
            return py_trees.common.Status.RUNNING

        elif self.state == 'face_dam':
            angle_error = self._wrap_angle(self.start_yaw - self.robot_yaw)
            if abs(angle_error) > 0.1:
                twist.angular.z = 0.3 if angle_error > 0 else -0.3
                self.cmd_pub.publish(twist)
            else:
                self.state = 'rotate_left'
            return py_trees.common.Status.RUNNING

        elif self.state == 'rotate_left':
            rotated = self._wrap_angle(self.robot_yaw - self.start_yaw)
            if rotated < math.radians(10):
                twist.angular.z = 0.3
                self.cmd_pub.publish(twist)
            else:
                self.state = 'rotate_right'
            return py_trees.common.Status.RUNNING

        elif self.state == 'rotate_right':
            rotated = self._wrap_angle(self.robot_yaw - self.start_yaw)
            if rotated > math.radians(-10):
                twist.angular.z = -0.3
                self.cmd_pub.publish(twist)
            else:
                self.state = 'end'
            return py_trees.common.Status.RUNNING

        elif self.state == 'end':
            return py_trees.common.Status.SUCCESS

    def detection_callback(self, msg):
        for obj in msg.objects:
            if obj.meaning == 'dam' and obj.detected:
                self.dam_x = obj.x
                self.dam_y = obj.y

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def terminate(self, new_status):
        self.state = 'start'
        self.start_yaw = None
