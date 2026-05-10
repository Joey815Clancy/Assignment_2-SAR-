import py_trees
import math
from nav_msgs.msg import Odometry
from sar_msgs.msg import ObjectDetectionArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from rclpy.time import Time

class SurvivorTf(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.survivor_x = None
        self.survivor_y = None
        self.tf_broadcaster = None 
        self.published= False 
    def setup(self, **kwargs):
        self.tf_broadcaster= StaticTransformBroadcaster(self.node)
        self.node.create_subscription(ObjectDetectionArray, '/detected_objects', self.detection_callback, 10)
    def update(self):
        ## wait for survivor position
        if self.survivor_x is None:
            return py_trees.common.Status.RUNNING
        ## publish the tf frame ## 
        if not self.published:
            t = TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'survivor'
            t.transform.translation.x = self.survivor_x
            t.transform.translation.y = self.survivor_y
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
            self.published = True
        return  py_trees.common.Status.SUCCESS
    
    def detection_callback(self,msg):
        for obj in msg.objects:
            if obj.meaning == 'survivor' and obj.detected:
                self.survivor_x = obj.x
                self.survivor_y = obj.y

    def terminate(self,new_status):
        pass