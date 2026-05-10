import py_trees
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class SpinSearch(py_trees.behaviour.Behaviour):
    def __init__(self,name,node):
        super().__init__(name)
        self.node = node
        self.start_yaw = None
        self.robot_yaw = 0.0
        self.total_rotated = 0.0 
        self.prev_yaw = None

    def setup(self, **kwargs):
        self.node.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.cmd_pub= self.node.create_publisher(Twist,'/cmd_vel',10)
    def _wrap_angle(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def update(self):
        ## Recorded starting yaw ##
        if self.start_yaw is None:
            self.start_yaw = self.robot_yaw
            self.prev_yaw = self.robot_yaw
            return py_trees.common.Status.RUNNING
        ## Find how much we have rotates ##
        angle_rotated = self._wrap_angle(self.robot_yaw - self.prev_yaw)
        self.total_rotated += abs(angle_rotated)
        self.prev_yaw = self.robot_yaw 
        ## Finished when done a full circle ## 
        if self.total_rotated >= 2*math.pi:
            self.cmd_pub.publish(Twist()) 
            return py_trees.common.Status.SUCCESS
        twist = Twist()
        twist.angular.z = 0.5
        self.cmd_pub.publish(twist)
        return py_trees.common.Status.RUNNING

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)
    def terminate(self, new_status):
        self.start_yaw =None
        self.total_rotated = 0.0 
        self.prev_yaw = None 