#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

# 静态map->odom只发布一次
def publish_static_map2odom():
    static_br = StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "odom"
    # x,y,z, roll,pitch,yaw 全部0，map和odom重合
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.w = 1.0 # 无旋转，四元数w=1
    static_br.sendTransform(t)
    rospy.loginfo("Static TF map -> odom published.")

def odom_callback(msg: Odometry):
    # 动态广播 odom -> base_link
    br = TransformBroadcaster()
    tf_msg = TransformStamped()
    tf_msg.header.stamp = msg.header.stamp
    tf_msg.header.frame_id = "odom"
    tf_msg.child_frame_id = "base_link"

    tf_msg.transform.translation.x = msg.pose.pose.position.x
    tf_msg.transform.translation.y = msg.pose.pose.position.y
    tf_msg.transform.translation.z = msg.pose.pose.position.z
    tf_msg.transform.rotation = msg.pose.pose.orientation

    br.sendTransform(tf_msg)

if __name__ == "__main__":
    rospy.init_node("odom_tf_all_node")
    rospy.loginfo("Start odom to base_link + map->odom static tf")
    
    # 发布map->odom静态tf
    publish_static_map2odom()
    
    # 订阅里程计话题
    rospy.Subscriber("/odin1/odometry", Odometry, odom_callback)
    rospy.spin()
