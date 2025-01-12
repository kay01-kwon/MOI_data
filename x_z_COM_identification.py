import os, fnmatch
import rosbag
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
from scipy import interpolate

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

        for topic, msg, t in bag.read_messages(topics=['/imu/data','/actual_rpm','/cmd_raw']):

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
            elif topic == '/actual_rpm':
                actual_rpm_time.append(t.to_sec())
                actual_rpm_data.append([
                    msg.rpm[0],
                    msg.rpm[1],
                    msg.rpm[2],
                    msg.rpm[3]
                ])
            elif topic == '/cmd_raw':
                cmd_raw_time.append(t.to_sec())
                cmd_raw_data.append([
                    msg.raw[0],
                    msg.raw[1],
                    msg.raw[2],
                    msg.raw[3]
                ])


        imu_time_array = np.array(imu_time)
        imu_quaternion_array = np.array(imu_quaternion)
        imu_angular_velocity_array = np.array(imu_angular_velocity)

        actual_rpm_time_array = np.array(actual_rpm_time)
        actual_rpm_data_array = np.array(actual_rpm_data)

        cmd_raw_time_array = np.array(cmd_raw_time)
        cmd_raw_data_array = np.array(cmd_raw_data)

        data_dict = {'imu_time': imu_time_array,
                    'imu_quaternion': imu_quaternion_array,
                    'imu_angular_velocity': imu_angular_velocity_array,
                     'actual_rpm_time': actual_rpm_time_array,
                     'actual_rpm': actual_rpm_data_array,
                     'cmd_raw_time': cmd_raw_time_array,
                     'cmd_raw': cmd_raw_data_array}
        bag.close()

        return data_dict


# How to use BagToDataDict object
if __name__ == '__main__':
    directory = 'bag/x_z_COM'
    BagToDataObj = BagToDataDict(directory)
    file_names = BagToDataObj.find_file_names()

    # Thrust parameter
    C_T = 1.465e-7
    l = 0.330/2.0

    # System parameter
    m = 2.190
    g = 9.81

    W = m*g
    My_avg_list = []

    theta_avg_list = []

    for file_name in file_names:
        # print(file_name)
        data_dict = BagToDataObj.get_data(file_name)

        t0 = data_dict['imu_time'][0]

        time = data_dict['imu_time'] - t0

        # Quaternion data
        qw = data_dict['imu_quaternion'][:,0]
        qx = data_dict['imu_quaternion'][:,1]
        qy = data_dict['imu_quaternion'][:,2]
        qz = data_dict['imu_quaternion'][:,3]

        theta = []
        My = []

        is_initialized = False

        initial_step = []


        # Append roll angle
        for i in range(len(data_dict['imu_time'])):
            theta_temp = (-np.pi/2
                          + 2*np.arctan2(
                np.sqrt(1+2*(qw[i]*qy[i] - qx[i]*qz[i])),
                np.sqrt(1-2*(qw[i]*qy[i] - qx[i]*qz[i]))
                    )
                )
            theta_temp = theta_temp * 180 / np.pi
            theta.append(theta_temp)

        # Append the data of moment along x
        for i in range(len(data_dict['actual_rpm_time'])):
            My_temp = l*C_T*(
                - data_dict['actual_rpm'][i,0]**2
                - data_dict['actual_rpm'][i,1]**2
                + data_dict['actual_rpm'][i,2]**2
                + data_dict['actual_rpm'][i,3]**2
            )
            My.append(My_temp)

        theta_avg = np.average(theta, axis=0)
        My_avg = np.average(My, axis=0)

        theta_avg_list.append(theta_avg)
        My_avg_list.append(My_avg)


        # plt.plot(time, phi, label='phi')
        # plt.plot(data_dict['actual_rpm_time'],Mx, label='Mx')

        print('******************************************************')
        # plt.grid('True')
        # plt.show()

    NC = 2
    NR = len(theta_avg_list)

    A = []

    for i in range(NR):
        row = []
        row.append([-W*np.cos(np.deg2rad(theta_avg_list[i])),
                    -W*np.sin(np.deg2rad(theta_avg_list[i]))])
        A.append(row)

    A_mat = np.array(A).reshape(NR,NC)
    My_vec = np.array(My_avg_list).reshape(-1,1)

    # print(A_mat0)
    print(A_mat.shape)
    print(My_vec.shape)
    x_z_est = inv(np.transpose(A_mat) @ A_mat) @ np.transpose(A_mat) @ My_vec

    print('Estimated x_COM: ', x_z_est[0]*1000,'mm')
    print('Estimated z_COM: ', x_z_est[1]*1000,'mm')

    # Data : Convert list to np.array
    theta_avg_array = np.array(theta_avg_list)
    My_array = np.array(My_avg_list)

    # Prediction Storage
    theta_domain = np.linspace(-60,50,100)
    tau_predicted = (-W*x_z_est[0]*np.cos(np.deg2rad(theta_domain))
                     - W*x_z_est[1]*np.sin(np.deg2rad(theta_domain)))

    plt.plot(theta_avg_list, My_array*1000, 'x', label='Data')
    plt.plot(theta_domain, tau_predicted*1000, label='Predicted')
    plt.xlabel(r'$\theta$ (deg)')
    plt.ylabel(r'$\tau$ (mNm)')
    plt.legend()
    plt.grid('True')
    plt.show()
