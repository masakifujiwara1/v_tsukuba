#!/usr/bin/env python3
import rospy
import math
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
# from tf.broadcaster import quaternion_from_euler

class OdomCalculator:
    def __init__(self):
        self.wheel_radius = 0.40632  # ホイール半径[m]
        self.tread_width = 0.42875   # トレッド幅[m]
        self.last_time = rospy.Time.now()
        self.last_x = 0
        self.last_y = 0
        self.last_yaw = 0

        # self.init_cmd_vel = Twist()
        # self.init_cmd_vel.linear.x = 0
        # self.cmd_vel_callback(self.init_cmd_vel)

        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        self.br = tf.TransformBroadcaster()

    def cmd_vel_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # 差分駆動型の運動モデルによる軌跡計算
        delta_x = (msg.linear.x * dt) * math.cos(self.last_yaw)
        delta_y = (msg.linear.x * dt) * math.sin(self.last_yaw)
        delta_yaw = (msg.angular.z * dt)

        # オドメトリ情報の計算
        self.last_x += delta_x
        self.last_y += delta_y
        self.last_yaw += delta_yaw

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # 位置と姿勢のセット
        odom_msg.pose.pose.position.x = self.last_x
        odom_msg.pose.pose.position.y = self.last_y
        odom_msg.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.last_yaw))

        # 速度のセット
        odom_msg.twist.twist.linear.x = msg.linear.x
        odom_msg.twist.twist.angular.z = msg.angular.z

        # オドメトリ情報をパブリッシュ
        self.odom_pub.publish(odom_msg)

        self.br.sendTransform(
            (self.last_x, self.last_y, 0),
            quaternion_from_euler(0, 0, self.last_yaw),
            current_time,
            "base_footprint",
            "odom"
        )

        self.last_time = current_time

if __name__ == '__main__':
    rospy.init_node('odom_calculator')
    odom_calculator = OdomCalculator()
    rospy.spin()
