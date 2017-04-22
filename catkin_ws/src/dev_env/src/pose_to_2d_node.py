#!/usr/bin/env python

import rospy
import sys
import signal
import math
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

TAG = "Pose to 2D:"

#------------------------------- NODE TEMPLATE --------------------------------#

# SIGINT Signal handler
def sigint_handler(signal, frame):
        print ""
        print TAG,"Interrupt!"
        print TAG,"Terminated"
        sys.exit(0)

def callback(data):
    global pub
    global tpub

    h = Header()
    h.stamp = data.header.stamp
    h.frame_id = "/tango_origin"

    p = Pose()
    position = Point()
    orientation = Quaternion()
    position.x = data.pose.position.x
    position.y = data.pose.position.y
    position.z = data.pose.position.z
    orientation.x = 0
    orientation.y = data.pose.orientation.y
    orientation.z = data.pose.orientation.z
    orientation.w = data.pose.orientation.w

    #Euler Angles
    qw = data.pose.orientation.w
    qx = data.pose.orientation.x
    qy = data.pose.orientation.y
    qz = data.pose.orientation.z

    ysqr = qy*qy

    t0 = +2.0 * (qw*qx + qy*qz)
    t1 = +1.0 - 2.0*(qx*qx + ysqr)
    roll = math.atan2(t0,t1)

    t2 = +2.0 * (qw*qy - qz*qx)
    if t2 > 1.0:
        t2 = 1.0
    elif t2 < -1.0:
        t2 = -1.0
    pitch = math.asin(t2)

    t3 = +2.0 * (qw*qw + qx*qy)
    t4 = +1.0 - 2.0*(ysqr + qz*qz)
    yaw = math.atan2(t3,t4)

    #print roll*57.2958, pitch*57.2958, yaw*57.2958

    # Back to quaternion
    roll = 0.0
    yaw = 0.0

    t0 = math.cos(yaw * 0.5)
    t1 = math.sin(yaw * 0.5)
    t2 = math.cos(roll * 0.5)
    t3 = math.sin(roll * 0.5)
    t4 = math.cos(pitch * 0.5)
    t5 = math.sin(pitch * 0.5)

    nqw = t0*t2*t4 + t1*t3*t5
    nqx = t0*t3*t4 - t1*t2*t5
    nqy = t0*t2*t5 + t1*t3*t4
    nqz = t1*t2*t4 - t0*t3*t5

    # Multiplying the original quaternion and quaternion with nullified roll and yaw
    # This will "project" the pose to XY plane and will negate gimbal lock to some extent.
    # While tested on "TANGO", there are some issues with tilting it sideways.
    # Not a problem for normal use, when device is parallel to the ground as it should be.
    orientation.w = qw*nqw - qx*nqx - qy*nqy - qz*nqz
    orientation.x = qw*nqx + qx*nqw + qy*nqz - qz*nqy
    orientation.y = qw*nqy - qx*nqz + qy*nqw + qz*nqx
    orientation.z = qw*nqz + qx*nqy + qy*nqx + qz*nqw

    msg = PoseStamped()
    msg.header = h
    p.position = position
    p.orientation = orientation
    msg.pose = p
    pub.publish(msg)

    tpub.sendTransform((position.x,position.y,position.z),
                        (orientation.x,orientation.y,orientation.z,orientation.w),
                        rospy.Time.now(),
                        "tango_pose_2d",
                        "tango_origin")

    #print msg

if __name__ == '__main__':
    print TAG,"Started"

    # Assigning the SIGINT handler.
    signal.signal(signal.SIGINT, sigint_handler)
    # Starting the node
    rospy.init_node('pose_to_2d', anonymous=True)
    tpub = tf.TransformBroadcaster()
    rospy.Subscriber("/tango_sensors/tango_pose", PoseStamped , callback)
    pub = rospy.Publisher("/pose_to_2d/", PoseStamped, queue_size=10)
    rospy.spin()

    print TAG,"Terminated"
