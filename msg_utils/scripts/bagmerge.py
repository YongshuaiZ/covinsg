#!/usr/bin/env python
import sys
import roslib
import rospy
import rosbag
from rospy import rostime
import argparse
import os
import json

from genpy import Time

def dict2json(file_name,the_dict):
    '''
    将字典文件写如到json文件中
    :param file_name: 要写入的json文件名(需要有.json后缀),str类型
    :param the_dict: 要写入的数据，dict类型
    :return: 1代表写入成功,0代表写入失败
    '''
    try:
        json_str = json.dumps(the_dict,indent=4,ensure_ascii=False)
        with open(file_name, 'w') as json_file:
            json_file.write(json_str)
        return 1
    except:
        return 0

time_dict = {
    '/cam0/image_raw0':[None, None],
    '/cam1/image_raw0':[None, None],
    '/cam0/image_raw1':[None, None],
    '/cam1/image_raw1':[None, None],
    '/cam0/image_raw2':[None, None],
    '/cam1/image_raw2':[None, None],
    '/cam0/image_raw3':[None, None],
    '/cam1/image_raw3':[None, None],
    '/cam0/image_raw4':[None, None],
    '/cam1/image_raw4':[None, None],
    '/imu0':[None, None],
    '/imu1':[None, None],
    '/imu2':[None, None],
    '/imu3':[None, None],
    '/imu4':[None, None],
    '/vicon/pose0':[None, None],
    '/vicon/pose1':[None, None],
    '/vicon/pose2':[None, None],
    '/vicon/pose3':[None, None],
    '/vicon/pose4':[None, None],
    '0': [None, None],
    '1': [None, None],
    '2': [None, None],
    '3': [None, None],
    '4': [None, None],
    'bag': [None, None]
}

str_time_dict = {
    '/cam0/image_raw0':[None, None],
    '/cam1/image_raw0':[None, None],
    '/cam0/image_raw1':[None, None],
    '/cam1/image_raw1':[None, None],
    '/cam0/image_raw2':[None, None],
    '/cam1/image_raw2':[None, None],
    '/cam0/image_raw3':[None, None],
    '/cam1/image_raw3':[None, None],
    '/cam0/image_raw4':[None, None],
    '/cam1/image_raw4':[None, None],
    '/imu0':[None, None],
    '/imu1':[None, None],
    '/imu2':[None, None],
    '/imu3':[None, None],
    '/imu4':[None, None],
    '/vicon/pose0':[None, None],
    '/vicon/pose1':[None, None],
    '/vicon/pose2':[None, None],
    '/vicon/pose3':[None, None],
    '/vicon/pose4':[None, None],
    '0': [None, None],
    '1': [None, None],
    '2': [None, None],
    '3': [None, None],
    '4': [None, None],
    'bag': [None, None]
}

def parse_args():
    parser = argparse.ArgumentParser(
        prog = 'bagmerge.py',
        description='Merges two bagfiles.')
    parser.add_argument('-o', type=str, help='name of the output file',
                        default = 'output.bag', metavar = "output_file")
    parser.add_argument('-t', type=str, help='topics which should be merged to the main bag',
                        default = None, metavar = "topics")
    parser.add_argument('-i', help='reindex bagfile',
                        default = False, action="store_true")
    parser.add_argument('main_bagfile', type=str, help='path to a bagfile, which will be the main bagfile')
    parser.add_argument('bagfile', type=str, help='path to a bagfile which should be merged to the main bagfile')
    args = parser.parse_args()
    return args

def get_next(bag_iter, topics = None):
    try:
        result = bag_iter.__next__()
        result[1].header.stamp = ( result[1].header.stamp - time_dict[result[0][-1]][0] ) + time_dict['bag'][0]
        if topics != None:
            while not result[0] in topics:
                result = bag_iter.__next__()
        return (result[0], result[1], result[1].header.stamp)
    except StopIteration:
        return None

def merge_bag(main_bagfile, bagfile, outfile = None, topics = None,
              reindex = True):
    #get min and max time in bagfile
    get_limits(main_bagfile)
    get_limits(bagfile)
    #check output file
    if outfile == None:
        pattern = main_bagfile + "_merged_%i.bag"
        outfile = main_bagfile + "_merged.bag"
        index = 0
        while (os.path.exists(outfile)):
            outfile = pattern%index
            index += 1
    #output some information
    print("merge bag %s in %s"%(bagfile, main_bagfile))
    print("topics filter: ", topics)
    print("writing to %s."%outfile)
    #merge bagfile
    outbag = rosbag.Bag(outfile, 'w')
    main_bag = rosbag.Bag(main_bagfile).__iter__()
    bag = rosbag.Bag(bagfile).__iter__()
    main_next = get_next(main_bag)
    next = get_next(bag)
    try:
        while main_next != None or next != None:
            if main_next == None:
                outbag.write(next[0], next[1], next[2])
                next = get_next(bag)
            elif next == None:
                outbag.write(main_next[0], main_next[1], main_next[2])
                main_next = get_next(main_bag)
            elif next[2] < main_next[2]:
                outbag.write(next[0], next[1], next[2])
                next = get_next(bag)
            else:
                outbag.write(main_next[0], main_next[1], main_next[2])
                main_next = get_next(main_bag)
    finally:
        outbag.close()

def get_limits(bagfile):
    print("Determine start and end index of %s..."%bagfile)

    for topic, msg, t in rosbag.Bag(bagfile).read_messages():
        msg_time = msg.header.stamp
        if time_dict[topic[-1]][0] == None or msg_time < time_dict[topic[-1]][0]:
            time_dict[topic[-1]][0] = msg_time
            str_time_dict[topic[-1]][0] = msg_time.to_sec()
        if time_dict[topic[-1]][1] == None or msg_time > time_dict[topic[-1]][1]:
            time_dict[topic[-1]][1] = msg_time
            str_time_dict[topic[-1]][1] = msg_time.to_sec()
        if time_dict[topic][0] == None or msg_time < time_dict[topic][0]:
            time_dict[topic][0] = msg_time
            str_time_dict[topic][0] = msg_time.to_sec()
        if time_dict[topic][1] == None or msg_time > time_dict[topic][1]:
            time_dict[topic][1] = msg_time
            str_time_dict[topic][1] = msg_time.to_sec()
        if time_dict['bag'][0] == None or msg_time < time_dict['bag'][0]:
            time_dict['bag'][0] = msg_time
            str_time_dict['bag'][0] = msg_time.to_sec()
        if time_dict['bag'][1] == None or msg_time > time_dict['bag'][1]:
            time_dict['bag'][1] = msg_time
            str_time_dict['bag'][1] = msg_time.to_sec()

if __name__ == "__main__":
    args = parse_args()
    print(args)
    if args.t != None:
        args.t = args.t.split(',')
    merge_bag(args.main_bagfile,
              args.bagfile,
              outfile = args.o,
              topics = args.t,
              reindex = args.i)
    file_name = './' + args.o[:args.o.find('.')] + '.json'
    json_str = json.dumps(str_time_dict,indent=4,ensure_ascii=False)
    with open(file_name, 'w') as json_file:
        json_file.write(json_str)