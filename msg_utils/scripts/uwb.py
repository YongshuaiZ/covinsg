#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import message_filters
import random
import math
from std_msgs.msg import Header
from msg_utils.msg import uwb


global mu, sigma
global uwb0_pub, uwb1_pub, uwb2_pub, uwb3_pub, uwb4_pub

def callback01(pose0_msg: PoseStamped, pose1_msg: PoseStamped):
    global mu, sigma
    global uwb0_pub, uwb1_pub

    uwb0_pub = rospy.Publisher('/uwb0', uwb, queue_size=100)
    uwb1_pub = rospy.Publisher('/uwb1', uwb, queue_size=100)

    uwb0_msg, uwb1_msg = uwb(), uwb()

    x0, y0, z0 = pose0_msg.pose.position.x, pose0_msg.pose.position.y, pose0_msg.pose.position.z
    x1, y1, z1 = pose1_msg.pose.position.x, pose1_msg.pose.position.y, pose1_msg.pose.position.z



    dist = math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0)) + random.gauss(mu, sigma)
    if dist < 0:
        dist = 0.0

    uwb0_msg.header = pose0_msg.header
    uwb1_msg.header = pose1_msg.header


    uwb0_msg.dest_id.append(1)
    uwb1_msg.dest_id.append(0)

    uwb0_msg.dist.append(dist)
    uwb1_msg.dist.append(dist)

    print('--------------------', dist)

    uwb0_pub.publish(uwb0_msg)
    uwb1_pub.publish(uwb1_msg)

def callback012(pose0_msg: PoseStamped, pose1_msg: PoseStamped, pose2_msg: PoseStamped):
    global mu, sigma
    global uwb0_pub, uwb1_pub, uwb2_pub

    uwb0_pub = rospy.Publisher('/uwb0', uwb, queue_size=100)
    uwb1_pub = rospy.Publisher('/uwb1', uwb, queue_size=100)
    uwb2_pub = rospy.Publisher('/uwb2', uwb, queue_size=100)


    uwb0_msg, uwb1_msg, uwb2_msg = uwb(), uwb(), uwb()

    x0, y0, z0 = pose0_msg.pose.position.x, pose0_msg.pose.position.y, pose0_msg.pose.position.z
    x1, y1, z1 = pose1_msg.pose.position.x, pose1_msg.pose.position.y, pose1_msg.pose.position.z
    x2, y2, z2 = pose2_msg.pose.position.x, pose2_msg.pose.position.y, pose2_msg.pose.position.z


    dist01 = math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0)) + random.gauss(mu, sigma)
    dist02 = math.sqrt((x2 - x0) * (x2 - x0) + (y2 - y0) * (y2 - y0) + (z2 - z0) * (z2 - z0)) + random.gauss(mu, sigma)
    dist12 = math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2)) + random.gauss(mu, sigma)

    uwb0_msg.header = pose0_msg.header
    uwb1_msg.header = pose1_msg.header
    uwb2_msg.header = pose2_msg.header


    uwb0_msg.dest_id.append(1)
    uwb0_msg.dest_id.append(2)
    uwb0_msg.dest_id.append(3)
    uwb0_msg.dest_id.append(4)

    uwb1_msg.dest_id.append(0)
    uwb1_msg.dest_id.append(2)
    uwb1_msg.dest_id.append(3)
    uwb1_msg.dest_id.append(4)

    uwb2_msg.dest_id.append(0)
    uwb2_msg.dest_id.append(1)
    uwb2_msg.dest_id.append(3)
    uwb2_msg.dest_id.append(4)

    uwb0_msg.dist.append(dist01)
    uwb0_msg.dist.append(dist02)

    uwb1_msg.dist.append(dist01)
    uwb1_msg.dist.append(dist12)

    uwb2_msg.dist.append(dist02)
    uwb2_msg.dist.append(dist12)


    uwb0_pub.publish(uwb0_msg)
    uwb1_pub.publish(uwb1_msg)
    uwb2_pub.publish(uwb2_msg)

def callback0123(pose0_msg: PoseStamped, pose1_msg: PoseStamped, pose2_msg: PoseStamped, pose3_msg: PoseStamped):
    global mu, sigma
    global uwb0_pub, uwb1_pub, uwb2_pub, uwb3_pub

    uwb0_pub = rospy.Publisher('/uwb0', uwb, queue_size=100)
    uwb1_pub = rospy.Publisher('/uwb1', uwb, queue_size=100)
    uwb2_pub = rospy.Publisher('/uwb2', uwb, queue_size=100)
    uwb3_pub = rospy.Publisher('/uwb3', uwb, queue_size=100)

    uwb0_msg, uwb1_msg, uwb2_msg, uwb3_msg = uwb(), uwb(), uwb(), uwb()

    x0, y0, z0 = pose0_msg.pose.position.x, pose0_msg.pose.position.y, pose0_msg.pose.position.z
    x1, y1, z1 = pose1_msg.pose.position.x, pose1_msg.pose.position.y, pose1_msg.pose.position.z
    x2, y2, z2 = pose2_msg.pose.position.x, pose2_msg.pose.position.y, pose2_msg.pose.position.z
    x3, y3, z3 = pose3_msg.pose.position.x, pose3_msg.pose.position.y, pose3_msg.pose.position.z

    dist01 = math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0)) + random.gauss(mu, sigma)
    dist02 = math.sqrt((x2 - x0) * (x2 - x0) + (y2 - y0) * (y2 - y0) + (z2 - z0) * (z2 - z0)) + random.gauss(mu, sigma)
    dist03 = math.sqrt((x3 - x0) * (x3 - x0) + (y3 - y0) * (y3 - y0) + (z3 - z0) * (z3 - z0)) + random.gauss(mu, sigma)
    dist12 = math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2)) + random.gauss(mu, sigma)
    dist13 = math.sqrt((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3) + (z1 - z3) * (z1 - z3)) + random.gauss(mu, sigma)
    dist23 = math.sqrt((x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3) + (z2 - z3) * (z2 - z3)) + random.gauss(mu, sigma)

    uwb0_msg.header = pose0_msg.header
    uwb1_msg.header = pose1_msg.header
    uwb2_msg.header = pose2_msg.header
    uwb3_msg.header = pose3_msg.header

    uwb0_msg.dest_id.append(1)
    uwb0_msg.dest_id.append(2)
    uwb0_msg.dest_id.append(3)
    uwb0_msg.dest_id.append(4)

    uwb1_msg.dest_id.append(0)
    uwb1_msg.dest_id.append(2)
    uwb1_msg.dest_id.append(3)
    uwb1_msg.dest_id.append(4)

    uwb2_msg.dest_id.append(0)
    uwb2_msg.dest_id.append(1)
    uwb2_msg.dest_id.append(3)
    uwb2_msg.dest_id.append(4)

    uwb3_msg.dest_id.append(0)
    uwb3_msg.dest_id.append(1)
    uwb3_msg.dest_id.append(2)
    uwb3_msg.dest_id.append(4)

    uwb0_msg.dist.append(dist01)
    uwb0_msg.dist.append(dist02)
    uwb0_msg.dist.append(dist03)

    uwb1_msg.dist.append(dist01)
    uwb1_msg.dist.append(dist12)
    uwb1_msg.dist.append(dist13)

    uwb2_msg.dist.append(dist02)
    uwb2_msg.dist.append(dist12)
    uwb2_msg.dist.append(dist23)

    uwb3_msg.dist.append(dist03)
    uwb3_msg.dist.append(dist13)
    uwb3_msg.dist.append(dist23)

    uwb0_pub.publish(uwb0_msg)
    uwb1_pub.publish(uwb1_msg)
    uwb2_pub.publish(uwb2_msg)
    uwb3_pub.publish(uwb3_msg)

def callback01234(pose0_msg: PoseStamped, pose1_msg: PoseStamped, pose2_msg: PoseStamped, pose3_msg: PoseStamped, pose4_msg: PoseStamped):
    global mu, sigma
    global uwb0_pub, uwb1_pub, uwb2_pub, uwb3_pub, uwb4_pub

    uwb0_pub = rospy.Publisher('/uwb0', uwb, queue_size=100)
    uwb1_pub = rospy.Publisher('/uwb1', uwb, queue_size=100)
    uwb2_pub = rospy.Publisher('/uwb2', uwb, queue_size=100)
    uwb3_pub = rospy.Publisher('/uwb3', uwb, queue_size=100)
    uwb4_pub = rospy.Publisher('/uwb4', uwb, queue_size=100)

    uwb0_msg, uwb1_msg, uwb2_msg, uwb3_msg, uwb4_msg = uwb(), uwb(), uwb(), uwb(), uwb()

    x0, y0, z0 = pose0_msg.pose.position.x, pose0_msg.pose.position.y, pose0_msg.pose.position.z
    x1, y1, z1 = pose1_msg.pose.position.x, pose1_msg.pose.position.y, pose1_msg.pose.position.z
    x2, y2, z2 = pose2_msg.pose.position.x, pose2_msg.pose.position.y, pose2_msg.pose.position.z
    x3, y3, z3 = pose3_msg.pose.position.x, pose3_msg.pose.position.y, pose3_msg.pose.position.z
    x4, y4, z4 = pose4_msg.pose.position.x, pose4_msg.pose.position.y, pose4_msg.pose.position.z



    dist01 = math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0)) + random.gauss(mu, sigma)
    dist02 = math.sqrt((x2 - x0) * (x2 - x0) + (y2 - y0) * (y2 - y0) + (z2 - z0) * (z2 - z0)) + random.gauss(mu, sigma)
    dist03 = math.sqrt((x3 - x0) * (x3 - x0) + (y3 - y0) * (y3 - y0) + (z3 - z0) * (z3 - z0)) + random.gauss(mu, sigma)
    dist04 = math.sqrt((x4 - x0) * (x4 - x0) + (y4 - y0) * (y4 - y0) + (z4 - z0) * (z4 - z0)) + random.gauss(mu, sigma)
    dist12 = math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2)) + random.gauss(mu, sigma)
    dist13 = math.sqrt((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3) + (z1 - z3) * (z1 - z3)) + random.gauss(mu, sigma)
    dist14 = math.sqrt((x1 - x4) * (x1 - x4) + (y1 - y4) * (y1 - y4) + (z1 - z4) * (z1 - z4)) + random.gauss(mu, sigma)
    dist23 = math.sqrt((x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3) + (z2 - z3) * (z2 - z3)) + random.gauss(mu, sigma)
    dist24 = math.sqrt((x2 - x4) * (x2 - x4) + (y2 - y4) * (y2 - y4) + (z2 - z4) * (z2 - z4)) + random.gauss(mu, sigma)
    dist34 = math.sqrt((x3 - x4) * (x3 - x4) + (y3 - y4) * (y3 - y4) + (z3 - z4) * (z3 - z4)) + random.gauss(mu, sigma)


    uwb0_msg.header = pose0_msg.header
    uwb1_msg.header = pose1_msg.header
    uwb2_msg.header = pose2_msg.header
    uwb3_msg.header = pose3_msg.header
    uwb4_msg.header = pose4_msg.header


    uwb0_msg.dest_id.append(1)
    uwb0_msg.dest_id.append(2)
    uwb0_msg.dest_id.append(3)
    uwb0_msg.dest_id.append(4)

    uwb1_msg.dest_id.append(0)
    uwb1_msg.dest_id.append(2)
    uwb1_msg.dest_id.append(3)
    uwb1_msg.dest_id.append(4)

    uwb2_msg.dest_id.append(0)
    uwb2_msg.dest_id.append(1)
    uwb2_msg.dest_id.append(3)
    uwb2_msg.dest_id.append(4)

    uwb3_msg.dest_id.append(0)
    uwb3_msg.dest_id.append(1)
    uwb3_msg.dest_id.append(2)
    uwb3_msg.dest_id.append(4)

    uwb4_msg.dest_id.append(0)
    uwb4_msg.dest_id.append(1)
    uwb4_msg.dest_id.append(2)
    uwb4_msg.dest_id.append(3)

    uwb0_msg.dist.append(dist01)
    uwb0_msg.dist.append(dist02)
    uwb0_msg.dist.append(dist03)
    uwb0_msg.dist.append(dist04)

    uwb1_msg.dist.append(dist01)
    uwb1_msg.dist.append(dist12)
    uwb1_msg.dist.append(dist13)
    uwb1_msg.dist.append(dist14)

    uwb2_msg.dist.append(dist02)
    uwb2_msg.dist.append(dist12)
    uwb2_msg.dist.append(dist23)
    uwb2_msg.dist.append(dist24)

    uwb3_msg.dist.append(dist03)
    uwb3_msg.dist.append(dist13)
    uwb3_msg.dist.append(dist23)
    uwb3_msg.dist.append(dist34)

    uwb4_msg.dist.append(dist04)
    uwb4_msg.dist.append(dist14)
    uwb4_msg.dist.append(dist24)
    uwb4_msg.dist.append(dist34)


    uwb0_pub.publish(uwb0_msg)
    uwb1_pub.publish(uwb1_msg)
    uwb2_pub.publish(uwb2_msg)
    uwb3_pub.publish(uwb3_msg)
    uwb4_pub.publish(uwb4_msg)

def listener():
    rospy.init_node('uwb')
    global mu, sigma

    mu, sigma = 0, 0.05
    pose0_sub = message_filters.Subscriber('/vicon/pose0', PoseStamped)
    pose1_sub = message_filters.Subscriber('/vicon/pose1', PoseStamped)
    pose2_sub = message_filters.Subscriber('/vicon/pose2', PoseStamped)
    pose3_sub = message_filters.Subscriber('/vicon/pose3', PoseStamped)
    pose4_sub = message_filters.Subscriber('/vicon/pose4', PoseStamped)

    # ts = message_filters.ApproximateTimeSynchronizer([pose0_sub, pose1_sub, pose2_sub, pose3_sub, pose4_sub], 100, 0.1)
    # ts.registerCallback(callback01234)
    ts = message_filters.ApproximateTimeSynchronizer([pose0_sub, pose1_sub, pose2_sub, pose3_sub], 100, 0.1)
    ts.registerCallback(callback0123)
    rospy.spin()

if __name__ == '__main__':
    listener()