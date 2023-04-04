#!/usr/bin/env python
import rosbag
import argparse
import os
import math
import random
from msg_utils.msg import uwb

def parse_args():
    parser = argparse.ArgumentParser(
        prog = 'uwb_with_python.py',
        description='calculate simulate uwb distance')
    parser.add_argument('-o', type=str, help='name of the output file',
                        default = None, metavar = "output_file")
    parser.add_argument('-n', type=str, help='if add noise',
                        default='True', metavar = "noise")
    parser.add_argument('bagfile', type=str, help='path to a bagfile')
    args = parser.parse_args()
    return args

def simulate_uwb(bagfile, outfile = None, noise = True):
    if outfile == None:
        pattern = bagfile + "_with_uwb_%i.bag"
        outfile = bagfile + "_with_uwb.bag"
        index = 0
        while (os.path.exists(outfile)):
            outfile = pattern%index
            index += 1
    outbag = rosbag.Bag(outfile, 'w')

    drone_num = get_drone_num(bagfile)
    msg_time_list, header_list = [[], [], [], [], [], []], [[], [], [], [], [], []]
    x_list, y_list, z_list = [[], [], [], [], [], []], [[], [], [], [], [], []], [[], [], [], [], [], []]
    time_list = [[], [], [], [], [], []]
    num0, num1 = 0, 0
    for topic, msg, t in rosbag.Bag(bagfile).read_messages():
        outbag.write( topic, msg, t )
        if topic[1:6] == 'vicon':
            drone_id = int(topic[-1])
            if drone_id == 0:
                num0 += 1
            if drone_id == 1:
                num1 += 1
            msg_time = msg.header.stamp
            msg_time_list[drone_id].append( msg_time )
            header_list[drone_id].append(msg.header)
            x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            x_list[drone_id].append(x)
            y_list[drone_id].append(y)
            z_list[drone_id].append(z)
            time_list[drone_id].append(t)
    print(num0, num1)
    print(len(msg_time_list[0]), len(msg_time_list[1]))
    for i in range(drone_num):
            for k in range(len(msg_time_list[i])):
                msg_time = msg_time_list[i][k]
                q_list = [None, None, None, None, None, None]
                for j in range(drone_num):
                    if j == i:
                        continue
                    try:
                        q = msg_time_list[j].index(msg_time)
                        q_list[j] = q
                    except ValueError:
                        q_list[j] = None
                        print( '无' )
                xi, yi, zi = x_list[i][k], y_list[i][k], z_list[i][k]
                header = header_list[i][k]
                bag_time = time_list[i][k]
                uwb_msg = uwb()

                flag = True
                for j in range(drone_num):
                    if j == i:
                        continue
                    if q_list[j] == None:
                        flag = False
                        break
                if flag == True:
                    print(q_list[:drone_num])
                    for j in range(drone_num):
                        if j == i:
                            continue
                        q = q_list[j]
                        xj, yj, zj = x_list[j][q], y_list[j][q], z_list[j][q]
                        if noise == 'True':
                            print("with noise")
                            distij = math.sqrt((xi - xj) * (xi - xj) + (yi - yj) * (yi - yj) + (zi - zj) * (zi - zj)) + random.gauss(0, 0.05)
                        else:
                            print("without noise")
                            distij = math.sqrt((xi - xj) * (xi - xj) + (yi - yj) * (yi - yj) + (zi - zj) * (zi - zj))
                        if distij < 0:
                            distij = 0
                        uwb_msg.header = header
                        uwb_msg.dest_id.append(j)
                        uwb_msg.dist.append(distij)
                        outbag.write( '/uwb' + str(i), uwb_msg, bag_time )


    outbag.close()

def get_drone_num(bagfile):
    drone_num = -1

    for topic, msg, t in rosbag.Bag(bagfile).read_messages():
        drone_num = max( drone_num, int(topic[-1]) )

    return drone_num + 1

if __name__ == "__main__":
    args = parse_args()
    print(args)
    print(args.n)
    simulate_uwb(args.bagfile,
              outfile=args.o, noise=args.n)