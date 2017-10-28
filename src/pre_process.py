__author__ = 'srkiyengar'

import numpy as np


gripper_file = "127871-2017-10-24-20-48-Servo-displacement"

class process_gripper_file:

    def __init__(self,some_file):

        self.original_file = some_file
        try:
            with open(some_file) as f:
                self.lines = f.readlines()
                self.processed_lines = []
        except IOError,e:
            print("Failure during opening gripper file {}".format(e))
            self.status = 0


    def pre_process(self):

        max_val = 0

        for line in self.lines[2:-4]:
            y = line.strip().split(",")
            del y[0]
            y = map(int,y)
            if y[0] and y[1] and y[2] and y[3]:
                self.processed_lines.append(line.strip())
                if y[0] > max_val:
                    max_val = y[0]

        i=0
        for line in reversed(self.processed_lines):  # from the bottom identify the largest value of servo motor 1
            y = line.strip().split(",")
            del y[0]
            y = map(int,y)
            if y[0] == max_val:
                break
            i=i+1

        while(i != 0):  #removing lines that are below the largest since they indicate gripper opening after closing
            del self.processed_lines[-i]
            i=i-1
        return

    def save_processed_file(self):

        try:
            with open(self.original_file+"-preprocessed","w") as f:
                for i in self.processed_lines:
                    i = i + "\n"
                    f.write(i)
        except IOError,e:
            print("While opening the preprocessed file for writing {}".format(e))
            self.status = 0

class process_labview_file:

    def __init__(self,some_file):
        self.original_file = some_file
        try:
            with open(some_file) as f:
                self.lines = f.readlines()
                self.processed_lines = []
        except IOError,e:
            print("Failure during opening labview file {}".format(e))
            self.status = 0



if __name__=="__main__":

    p = process_gripper_file(gripper_file)
    p.pre_process()
    p.save_processed_file()

    pass




