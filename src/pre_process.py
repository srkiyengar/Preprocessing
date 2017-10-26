__author__ = 'srkiyengar'

import numpy as np


gripper_file = "127871-2017-10-24-20-48-Servo-displacement"



def pre_process(file_name):

    temp_lines = []
    max_val = 0

    with open(file_name) as f:
        lines = f.readlines()

    for line in lines[2:-4]:
        y = line.strip().split(",")
        del y[0]
        y = map(int,y)
        if y[0] and y[1] and y[2] and y[3]:
            temp_lines.append(line.strip())
            if y[0] > max_val:
                max_val = y[0]

    i=0
    for line in reversed(temp_lines):
        y = line.strip().split(",")
        del y[0]
        y = map(int,y)
        if y[0] == max_val:
            break
        i=i+1

    return temp_lines[:-i]




if __name__=="__main__":

    processed_list = pre_process(gripper_file)
    for line in processed_list:
        print line



