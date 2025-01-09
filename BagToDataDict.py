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

        for topic, msg, t in bag.read_messages(topics=['/imu/data', '/actual_rpm', '/cmd_raw']):

            if topic == '/actual_rpm':
                # actual_rpm_time.append(t.to_sec())
                actual_rpm_time.append(msg.stamp.to_sec())
                actual_rpm_data.append([msg.rpm[0],
                                        msg.rpm[1],
                                        msg.rpm[2],
                                        msg.rpm[3]])

            elif topic == '/cmd_raw':
                cmd_raw_time.append(t.to_sec())
                cmd_raw_data.append([msg.raw[0],
                                     msg.raw[1],
                                     msg.raw[2],
                                     msg.raw[3]])

            elif topic == '/imu/data':
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

        actual_rpm_time_array = np.array(actual_rpm_time)
        actual_rpm_data_array = np.array(actual_rpm_data)

        cmd_raw_time_array = np.array(cmd_raw_time)
        cmd_raw_data_array = np.array(cmd_raw_data)

        imu_time_array = np.array(imu_time)
        # time = np.linspace(imu_time[0], imu_time[-1], num=len(imu_time))
        # time_array = np.array(time)
        imu_quaternion_array = np.array(imu_quaternion)
        imu_angular_velocity_array = np.array(imu_angular_velocity)

        # Interpolate RPM data to match IMU timestamps
        interp_rpm = interpolate.interp1d(actual_rpm_time_array,
                                          actual_rpm_data_array,
                                          axis=0,
                                          fill_value='extrapolate')

        synchronized_rpm = interp_rpm(imu_time_array)

        interp_cmd = interpolate.interp1d(cmd_raw_time_array,
                                          cmd_raw_data_array,
                                          axis=0,
                                          fill_value='extrapolate')
        synchronized_cmd = interp_cmd(imu_time_array)

        data_dict = {'imu_time': imu_time_array,
                     'actual_rpm_time': actual_rpm_time_array,
                    'imu_quaternion': imu_quaternion_array,
                    'imu_angular_velocity': imu_angular_velocity_array,
                    'synchronized_rpm': synchronized_rpm,
                     'synchronized_cmd': synchronized_cmd,
                     'actual_rpm': actual_rpm_data_array}
        bag.close()

        return data_dict


# How to use BagToDataDict object
if __name__ == '__main__':
    directory = 'bag/J_zz_data'
    BagToDataObj = BagToDataDict(directory)
    file_names = BagToDataObj.find_file_names()

    C_M = 2e-9

    MaxRpm = 9800
    MaxBit = 8191

    avg_peak_time = 0

    for file_name in file_names:
        # print(file_name)
        data_dict = BagToDataObj.get_data(file_name)

        t0 = data_dict['imu_time'][0]

        time = data_dict['imu_time'] - t0

        # plt.plot(data_dict['imu_time'] - t0, data_dict['imu_angular_velocity'][:,2])
        # plt.plot(data_dict['imu_time'] - t0, data_dict['synchronized_rpm'][:])

        Mz_interp = []
        Mz_cmd_interp = []
        Mz = []

        qw = data_dict['imu_quaternion'][:,0]
        qx = data_dict['imu_quaternion'][:,1]
        qy = data_dict['imu_quaternion'][:,2]
        qz = data_dict['imu_quaternion'][:,3]

        psi = []

        psi_steady = np.arctan2(2*(qw[-1]*qz[-1]+qx[-1]*qy[-1]),
                                  1-2*(qy[-1]*qy[-1]+qz[-1]*qz[-1]))


        is_initialized = False

        initial_step = []

        for i in range(len(data_dict['actual_rpm_time'])):
            Mz_temp = C_M * data_dict['actual_rpm'][i, 0] ** 2 \
                      - C_M * data_dict['actual_rpm'][i, 1] ** 2 \
                      + C_M * data_dict['actual_rpm'][i, 2] ** 2 \
                      - C_M * data_dict['actual_rpm'][i, 3] ** 2
            Mz.append(Mz_temp)


        for i in range(len(data_dict['imu_time'])):

            Mz_temp = C_M*data_dict['synchronized_rpm'][i,0]**2 \
            - C_M * data_dict['synchronized_rpm'][i,1]**2 \
            + C_M * data_dict['synchronized_rpm'][i,2]**2 \
            - C_M * data_dict['synchronized_rpm'][i,3]**2
            Mz_interp.append(Mz_temp)

            rpm_cmd1 = data_dict['synchronized_cmd'][i, 0] * MaxRpm/MaxBit
            rpm_cmd2 = data_dict['synchronized_cmd'][i, 1] * MaxRpm/MaxBit
            rpm_cmd3 = data_dict['synchronized_cmd'][i, 2] * MaxRpm/MaxBit
            rpm_cmd4 = data_dict['synchronized_cmd'][i, 3] * MaxRpm/MaxBit

            Mz_cmd_temp = C_M*rpm_cmd1**2 \
                        - C_M*rpm_cmd2**2 \
                        + C_M*rpm_cmd3**2 \
                        - C_M*rpm_cmd4**2
            Mz_cmd_interp.append(Mz_cmd_temp)

            psi_temp = np.arctan2(2*(qw[i]*qz[i]+qx[i]*qy[i]),
                                  1-2*(qy[i]*qy[i]+qz[i]*qz[i]))
            psi.append(psi_temp)

            if np.abs(Mz_cmd_temp) >= 0.05 and is_initialized == False:
                initial_step = i
                is_initialized = True

        max_psi_index = np.argmax(psi)

        peak_time = time[max_psi_index] - time[initial_step]
        print('Max Moment: ', np.max(Mz_cmd_interp))
        print('Initial Moment: ', Mz_interp[0])

        Mp = (psi[max_psi_index] - psi[0] - psi_steady)/(psi_steady + psi[0])
        sheta = -np.log(Mp)/np.sqrt(np.pi**2 + np.log(Mp)**2)

        print('Peak time: ',peak_time)
        print('%OS: ', Mp*100.0)

        w_d = np.pi/peak_time

        w_n = w_d/np.sqrt(1 - sheta**2)

        K_p = 0.3

        J_zz = K_p/(w_n**2)

        print('J_zz : ',J_zz)

        plt.plot(time, Mz_interp, label='Synch RPM')
        plt.plot(time, Mz_cmd_interp, label='Synch cmd')
        plt.plot(time, psi)
        plt.plot(time[max_psi_index], psi[max_psi_index],'x')
        plt.plot(time[initial_step], psi[initial_step],'x')


        plt.grid('True')

        plt.show()
