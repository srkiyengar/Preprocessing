__author__ = 'srkiyengar'

from datetime import datetime, timedelta
from scipy.interpolate import UnivariateSpline

file1 = "blah"
file2 = "blah2"

class combine:

    def __init__(self,processed_ndi,processed_servo):

        self.ndi = processed_ndi
        self.gripper = processed_servo
        try:
            with open(processed_ndi) as f:
                self.ndi_lines = f.readlines()
        except IOError,e:
            print("Failure during opening processed_ndi file {}".format(e))
            raise IOError ("Unable to open processed data file {}".format(processed_ndi))

        try:
            with open(processed_servo) as f:
                self.servo_lines = f.readlines()
        except IOError,e:
            print("Failure during opening processed_servo file {}".format(e))
            raise IOError ("Unable to open processed data file {}".format(processed_servo))

    def merge_data(self):

        clock_diff = int(self.servo_lines[0])   #Positive means the gripper is behind ndi
        gripper_time =[]
        sf1 = []
        sf2 = []
        sf3 = []
        sf4 = []

        y = self.servo_lines[1].strip().split(",")
        dateSetting = '%Y-%m-%d %H:%M:%S.%f'
        t0 = datetime.strptime(y[0],dateSetting)

        for line in self.servo_lines[1:]:
            y = line.strip().split(",")
            clock_diff = timedelta(microseconds=clock_diff)
            servo_time = datetime.strptime(y[0], dateSetting)
            gripper_time.append((servo_time-t0+clock_diff).total_seconds())
            sf1.append(int(y[1]))
            sf2.append(int(y[2]))
            sf3.append(int(y[3]))
            sf4.append(int(y[4]))

        spl1 = UnivariateSpline(gripper_time,sf1)
        spl2 = UnivariateSpline(gripper_time,sf2)
        spl3 = UnivariateSpline(gripper_time,sf3)
        spl4 = UnivariateSpline(gripper_time,sf4)

        start = self.ndi_lines[0].strip().split(",")[0:1]
        dateSetting = '%Y-%m-%d-%H-%M-%S.%f'
        t0 = datetime.strptime(start, dateSetting)
        for line in self.ndi_lines:
            t,tool= line.strip().split(",")[0:2]
            x, y, z, Rx,Ry,Rz = map(float,(line.strip().split(",")[2:]))
            time_diff = (t-t0).total_seconds()
            finger1 = spl1(time_diff)
            finger2 = spl2(time_diff)
            finger3 = spl3(time_diff)
            finger4 = spl4(time_diff)
            print time_diff,x,y,z,Rx,Ry,Rz,finger1,finger2,finger3,finger4



if __name__== "__main__":

    combine(file1,file2)