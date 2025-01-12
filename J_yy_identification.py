import os, fnmatch
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from scipy.signal import find_peaks

class BagToDataDict:
    def __init__(self,directory):

        self.directory = directory
        self.file_names = []


    def find_file_names(self):

        file_names = []

        for root, dirs, files in os.walk(self.directory):
            for name in files:
                if fnmatch.fnmatch(name, '*.bag'):
                    file_names.append(name)

        self.file_names = sorted(file_names)

        number_of_files = len(self.file_names)

        print("Number of files: " + str(number_of_files))

        return self.file_names

    def get_data(self, file_name):

        print('******************************************************')
        print("Reading bag file: " + self.directory + "/" + file_name)

        # Directory of bag file
        bag_file_dir = self.directory + '/' + file_name
        bag = rosbag.Bag(bag_file_dir)

        # Actual data list to store
        actual_rpm_time = []
        actual_rpm_data = []

        cmd_raw_time = []
        cmd_raw_data = []

        # IMU data list to store
        imu_time = []
        imu_quaternion = []
        imu_angular_velocity = []

        for topic, msg, t in bag.read_messages(topics=['/imu/data']):

            if topic == '/imu/data':
                imu_time.append(t.to_sec())
                imu_quaternion.append([
                    msg.orientation.w,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z
                ])
                imu_angular_velocity.append(
                    [
                        msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z
                    ]
                )


        imu_time_array = np.array(imu_time)
        # time = np.linspace(imu_time[0], imu_time[-1], num=len(imu_time))
        # time_array = np.array(time)
        imu_quaternion_array = np.array(imu_quaternion)
        imu_angular_velocity_array = np.array(imu_angular_velocity)

        data_dict = {'imu_time': imu_time_array,
                    'imu_quaternion': imu_quaternion_array,
                    'imu_angular_velocity': imu_angular_velocity_array}
        bag.close()

        return data_dict


# How to use BagToDataDict object
if __name__ == '__main__':
    directory = 'bag/J_yy_data_imu_only'
    BagToDataObj = BagToDataDict(directory)
    file_names = BagToDataObj.find_file_names()

    avg_peak_time = 0

    natual_freq_0 = []

    j = 0

    for file_name in file_names:
        # print(file_name)
        data_dict = BagToDataObj.get_data(file_name)

        t0 = data_dict['imu_time'][0]

        time = data_dict['imu_time'] - t0

        qw = data_dict['imu_quaternion'][:,0]
        qx = data_dict['imu_quaternion'][:,1]
        qy = data_dict['imu_quaternion'][:,2]
        qz = data_dict['imu_quaternion'][:,3]

        theta = []

        is_initialized = False

        initial_step = []


        for i in range(len(data_dict['imu_time'])):

            theta_temp = (-np.pi/2
                          + 2*np.arctan2(
                np.sqrt(1+2*(qw[i]*qy[i] -qx[i]*qz[i])),
                np.sqrt(1-2*(qw[i]*qy[i] -qx[i]*qz[i])))
                )
            theta.append(theta_temp)

        peak_indices, _ = find_peaks(theta, height = -0.18)

        print(peak_indices)

        # plt.plot(time, psi)

        time_diff_sum = time[peak_indices[-1]] - time[peak_indices[1]]


        w_n = 2*np.pi*(len(peak_indices)-2)/time_diff_sum
        print('Time diff avg:',time_diff_sum)
        print('The number of peaks:',len(peak_indices)-2)
        print('natural frequency: ', w_n)
        print('******************************************************')
        # plt.grid('True')
        # plt.show()

        natual_freq_0.append(w_n)

        if np.mod(j,10) == 0:
            plt.plot(time,theta)

            for peak_index in peak_indices:
                plt.plot(time[peak_index],theta[peak_index],'x')

            plt.title('Theta')
            plt.grid('True')
            plt.show()

        j+=1

natural_freq_avg = np.average(natual_freq_0)


print('natual frequency for Jyy: ', natual_freq_0)

m = 2.190
g = 9.81
r = 0.0236
mgr = m*g*r

J_yy = mgr/natural_freq_avg**2
print('J_{yy}: ', J_yy, r'kg*m^2')