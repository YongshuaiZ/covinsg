#!/usr/bin/env python
import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import time, sys, os
import argparse
import cv2
import csv

# structure
# dataset/cam0/data/TIMESTAMP.png
# dataset/camN/data/TIMESTAMP.png
# dataset/imu0/data.csv
# dataset/imuN/data.csv
# dataset/leica0/data.csv

# setup the argument list
parser = argparse.ArgumentParser(description='Create a ROS bag using the images and imu data.')
parser.add_argument('--folder', metavar='folder', nargs='?', help='Data folder')
parser.add_argument('--output_bag', metavar='output_bag', default="output.bag", help='ROS bag file %(default)s')
parser.add_argument('--drone_id', metavar='drone_id', default="0", help='Drone Id')
# print help if no argument is specified
if len(sys.argv) < 3:
    parser.print_help()
    sys.exit(0)

# parse the args
parsed = parser.parse_args()


def getImageFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    image_files = list()
    timestamps = list()
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
                    image_files.append(os.path.join(path, f))
                    timestamps.append(os.path.splitext(f)[0])
                    # sort by timestamp
    sort_list = sorted(zip(timestamps, image_files))
    image_files = [file[1] for file in sort_list]
    return image_files


def getCamFoldersFromDir(dir):
    '''Generates a list of all folders that start with cam e.g. cam0'''
    cam_folders = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for folder in folders:
                if folder[0:3] == "cam":
                    cam_folders.append(folder)
    return cam_folders


def getImuFoldersFromDir(dir):
    '''Generates a list of all folders that start with imu e.g. cam0'''
    imu_folders = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for folder in folders:
                if folder[0:3] == "imu":
                    imu_folders.append(folder)
    return imu_folders


def getImuCsvFiles(dir):
    '''Generates a list of all csv files that start with imu'''
    imu_files = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for file in files:
                if file[0:3] == 'imu' and os.path.splitext(file)[1] == ".csv":
                    imu_files.append(os.path.join(path, file))

    return imu_files


def loadImageToRosMsg(filename):
    image_np = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)

    timestamp_nsecs = os.path.splitext(os.path.basename(filename))[0]
    timestamp = rospy.Time(secs=int(timestamp_nsecs[0:-9]), nsecs=int(timestamp_nsecs[-9:]))

    rosimage = Image()
    rosimage.header.stamp = timestamp
    rosimage.height = image_np.shape[0]
    rosimage.width = image_np.shape[1]
    rosimage.step = rosimage.width  # only with mono8! (step = width * byteperpixel * numChannels)
    rosimage.encoding = "mono8"
    rosimage.data = image_np.tobytes()

    return rosimage, timestamp


def createImuMessge(timestamp_int, omega, alpha):
    timestamp_nsecs = str(timestamp_int)
    timestamp = rospy.Time(int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]))

    rosimu = Imu()
    rosimu.header.stamp = timestamp
    rosimu.angular_velocity.x = float(omega[0])
    rosimu.angular_velocity.y = float(omega[1])
    rosimu.angular_velocity.z = float(omega[2])
    rosimu.linear_acceleration.x = float(alpha[0])
    rosimu.linear_acceleration.y = float(alpha[1])
    rosimu.linear_acceleration.z = float(alpha[2])

    return rosimu, timestamp

# create the bag
bag = rosbag.Bag(parsed.output_bag, 'w')
drone_id = parsed.drone_id
# write images
camfolders = getCamFoldersFromDir(parsed.folder)
for camfolder in camfolders:
    camdir = parsed.folder + "/{0}".format(camfolder) + "/data"
    image_files = getImageFilesFromDir(camdir)
    for image_filename in image_files:
        image_msg, timestamp = loadImageToRosMsg(image_filename)
        bag.write("/{0}/image_raw{1}".format(camfolder, drone_id), image_msg, timestamp)

# write imu data
imufolders = getImuFoldersFromDir(parsed.folder)
for imufolder in imufolders:
    imufile = parsed.folder + "/" + imufolder + "/data.csv"
    topic = os.path.splitext(os.path.basename(imufolder))[0]
    with open(imufile, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        headers = next(reader, None)
        for row in reader:
            imumsg, timestamp = createImuMessge(row[0], row[1:4], row[4:7])
            bag.write("/imu{0}".format(drone_id), imumsg, timestamp)

# write vicon data
viconfile = parsed.folder + "/state_groundtruth_estimate0/data.csv"
with open(viconfile, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    headers = next(reader, None)
    for row in reader:
        timestamp_nsecs = str(row[0])
        timestamp = rospy.Time( int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )

        pose = PoseStamped()
        pose.header.stamp = timestamp
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(row[1])
        pose.pose.position.y = float(row[2])
        pose.pose.position.z = float(row[3])

        pose.pose.orientation.w = float(row[4])
        pose.pose.orientation.x = float(row[5])
        pose.pose.orientation.y = float(row[6])
        pose.pose.orientation.z = float(row[7])

        bag.write("/vicon/pose{0}".format(drone_id), pose, timestamp)

bag.close()