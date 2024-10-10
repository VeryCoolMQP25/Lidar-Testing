import rclpy 
from rclpy.node import Node 
from nav_msgs.msg import Odometry # for publishing to odom topic
from geometry_msgs.msg import Quaternion 
from geometry_msgs.msg import TransformStamped 

import rclpy.timer
from tf2_ros import TransformBroadcaster # need to publish odom and base_link transform 
from tf_transformations import quaternion_from_euler 
import math
import threading

class FakeOdomNode(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.transform_broadcaster = TransformBroadcaster(self)

        x = 0.0
        y = 0.0 
        theta = 0.0

        vx = 0.1
        vy = -0.1
        vth = 0.1

        self.timer = self.create_timer(0.5, self.update_odometry)

        self.last_time = self.get_clock().now()
    
    def update_odometry(self):
        curr_time = self.get_clock().now()
        dt = (curr_time - self.last_time).nansoseconds / 1e9

        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta) * dt)
        delta_y = (self.vx * math.csin(self.theta) - self.vy * math.cos(self.theta) * dt)
        delta_theta = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # creating quaternion from yaw angle 
        odom_quat = Quaternion(quaternion_from_euler(self.theta))

        odom_trans = TransformStamped()
        odom_trans.header.stamp = curr_time
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = odom_quat

        # send transform over tf 

        self.transform_broadcaster.sendTransform(odom_trans)

        odom = Odometry()
        odom.header.stamp = curr_time.to_msg()
        odom.header.frame_id = "odom"

        # setting position 

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat

        # setting velocity 

        odom.child_frame_id = "base_link"

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        # publish message to odom 

        self.odom_pub.publish(odom)

        self.last_time = curr_time
    
    def main(args=None):
        rclpy.init(args=args)

        # Create Instance of Node 
        fake_odom_node = FakeOdomNode()

        # Spin Node in seperate thread

        spin_thread = threading.thread(target=rclpy.spin, arg=(fake_odom_node,), daemon=True)
        spin_thread.start()

        try:
            spin_thread.join()
        except KeyboardInterrupt:
            pass

        rclpy.shutdown()

    if __name__ == '__main__':
        main()







