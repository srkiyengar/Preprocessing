__author__ = 'srkiyengar'

import numpy as np


gripper_file = "127871-2017-10-24-20-48-Servo-displacement"
labview_ndi_file = "691960-2017-10-29-17-03-42.txt"

class process_gripper_file:

    def __init__(self,some_file):

        self.original_file = some_file
        try:
            with open(some_file) as f:
                self.lines = f.readlines()
                self.processed_lines = []
        except IOError,e:
            print("Failure during opening gripper file {}".format(e))
            raise IOError ("Unable to open Gripper data file {}".format(some_file))


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
            raise IOError ("Unable to create pre-processed Gripper data file")

class process_labview_file:

    def __init__(self,some_file):
        self.original_file = some_file
        try:
            with open(some_file) as f:
                self.lines = f.readlines()
                self.processed_lines = []
        except IOError,e:
            print("Failure during opening labview file {}".format(e))
            raise IOError ("Unable to open NDI Labview file {}".format(some_file))

    def preprocess(self):
    # preprocess cannot delete the lines based on NDI x-axis since 449 and 339 are in different position
        max_x = -1000.0
        for line in self.lines[6:]:
            y = line.strip().split(",")
            del y[0]
            if "Both" not in y[1]:
                if "449" in self.lines[0]:
                    y[1] = str(449)
                    y[9] = str(339)
                else:
                    y[9] = str(449)
                    y[1] = str(339)

                if (float(y[2])==0.0) and (float(y[3])==0.0) and (float(y[4])==0.0): # just checking if x,y,z are zero which means no values
                    y[1]=y[9]
                    y[2]=y[10]
                    y[3]=y[11]
                    y[4]=y[12]
                    y[5]=y[13]
                    y[6]=y[14]
                    y[7]=y[15]
                    y[8]=y[16]

                y = y[:9]
                newline = ','.join(y)
                self.processed_lines.append(newline)


    def save_processed_file(self):

        try:
            with open(self.original_file+"-preprocessed","w") as f:
                for i in self.processed_lines:
                    i = i + "\n"
                    f.write(i)
        except IOError,e:
            print("While opening the preprocessed file for writing {}".format(e))
            raise IOError ("Unable to create pre-processed NDI data file")



if __name__=="__main__":

    #p = process_gripper_file(gripper_file)
    #p.pre_process()
    #p.save_processed_file()
    p = process_labview_file(labview_ndi_file)
    p.preprocess()
    p.save_processed_file()

    pass




