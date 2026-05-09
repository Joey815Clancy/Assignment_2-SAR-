#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from sar_msgs.msg import ObjectDetectionArray


class TravelTo(Node):

    def __init__(self):
        super().__init__('robot_travel')

        #self.declare_parameter('goal_x', 0.0)
        #self.declare_parameter('goal_y', 0.0)

        #self.goal_x = self.get_parameter('goal_x').value
        #self.goal_y = self.get_parameter('goal_y').value
        self.declare_parameter('target', 'survivor')
        self.target = self.get_parameter('target').value

        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0
        self.distance = 0.0

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_obj_data = self.create_subscription(ObjectDetectionArray,'/detected_objects',self.object_callback,10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _wrap_angle(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def object_callback(self,msg):

        for obj in msg.objects:
            if obj.meaning == self.target and obj.detected:
                self.goal_x = obj.x
                self.goal_y = obj.y

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y

        distance = math.sqrt(dx**2 + dy**2)

        angle_to_goal = math.atan2(dy, dx)
        angle_error = self._wrap_angle(angle_to_goal - self.yaw)
        
        ## To be includded in custom msg later ##
        stopping_distance = 0.1

        twist = Twist()

        if distance < stopping_distance:
            self.get_logger().info('Goal reached')
            self.pub.publish(twist)
            return

        if abs(angle_error) > 0.1:
            self.get_logger().info(f'Turning to correct angle, error: {angle_error:.2f}')
            twist.linear.x = 0.0
            if angle_error > 0:
                twist.angular.z = 0.5
            else:
                twist.angular.z = -0.5
        else:
            self.get_logger().info(f'Driving to goal, distance: {distance:.2f}')
            twist.linear.x = 0.5

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TravelTo()
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


if __name__ == '__main__':
    main()
