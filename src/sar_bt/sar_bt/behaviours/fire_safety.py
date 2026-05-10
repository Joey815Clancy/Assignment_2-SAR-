import py_trees
import math
from nav_msgs.msg import Odometry
from sar_msgs.msg import ObjectDetectionArray


class FireSafety(py_trees.behaviour.Behaviour):

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.fire_x = None
        self.fire_y = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.min_distance = 3.0

    def setup(self, **kwargs):
        self.node.create_subscription(ObjectDetectionArray, '/detected_objects', self.detection_callback, 10)
        self.node.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def update(self):
        if self.fire_x is None:
            return py_trees.common.Status.SUCCESS

        distance = math.sqrt((self.fire_x - self.robot_x)**2 + (self.fire_y - self.robot_y)**2)

        if distance >= self.min_distance:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def detection_callback(self, msg):
        for obj in msg.objects:
            if obj.meaning == 'fire' and obj.detected:
                self.fire_x = obj.x
                self.fire_y = obj.y

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def terminate(self, new_status):
        pass
