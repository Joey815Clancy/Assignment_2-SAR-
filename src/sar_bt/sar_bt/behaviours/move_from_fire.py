import py_trees
import math
from nav_msgs.msg import Odometry
from sar_msgs.msg import ObjectDetectionArray
from geometry_msgs.msg import Twist


class MoveFromFire(py_trees.behaviour.Behaviour):

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.fire_x = None
        self.fire_y = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.min_distance = 3.0

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
        if self.fire_x is None:
            return py_trees.common.Status.SUCCESS

        distance = math.sqrt((self.fire_x - self.robot_x)**2 + (self.fire_y - self.robot_y)**2)

        if distance >= self.min_distance:
            return py_trees.common.Status.SUCCESS

        angle_away = math.atan2(self.robot_y - self.fire_y, self.robot_x - self.fire_x)
        angle_error = self._wrap_angle(angle_away - self.robot_yaw)

        twist = Twist()
        if abs(angle_error) > 0.1:
            twist.angular.z = 0.5 if angle_error > 0 else -0.5
        else:
            twist.linear.x = 0.3
        self.cmd_pub.publish(twist)
        return py_trees.common.Status.RUNNING

    def detection_callback(self, msg):
        for obj in msg.objects:
            if obj.meaning == 'fire' and obj.detected:
                self.fire_x = obj.x
                self.fire_y = obj.y

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def terminate(self, new_status):
        self.cmd_pub.publish(Twist())
