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
    directory = 'bag/y_z_COM'
    BagToDataObj = BagToDataDict(directory)
    file_names = BagToDataObj.find_file_names()

    # Thrust parameter
    C_T = 148e-9
    l = 0.330/2.0

    # System parameter
    m0 = 2.190
    g = 9.81
    m_added = 0.510
    m1 = m0 + m_added

    W0 = m0*g
    W1 = m1*g
    Mx_avg_list0 = []
    Mx_avg_list1 = []

    phi_avg_list0 = []
    phi_avg_list1 = []

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

        phi = []
        Mx = []

        is_initialized = False

        initial_step = []


        # Append roll angle
        for i in range(len(data_dict['imu_time'])):
            phi_temp = np.arctan2(2*(qw[i]*qx[i]+qy[i]*qz[i]),
                                  1-2*(qx[i]*qx[i]+qy[i]*qy[i]))
            phi_temp = phi_temp*180/np.pi
            phi.append(phi_temp)

        # Append the data of moment along x
        for i in range(len(data_dict['actual_rpm_time'])):
            Mx_temp = l*C_T*(
                data_dict['actual_rpm'][i,0]**2
                - data_dict['actual_rpm'][i,1]**2
                - data_dict['actual_rpm'][i,2]**2
                + data_dict['actual_rpm'][i,3]**2
            )
            Mx.append(Mx_temp)

        phi_avg = np.average(phi, axis=0)
        Mx_avg = np.average(Mx, axis=0)

        if 'weight' not in file_name:
            phi_avg_list0.append(phi_avg)
            Mx_avg_list0.append(Mx_avg)
        else:
            phi_avg_list1.append(phi_avg)
            Mx_avg_list1.append(Mx_avg)


        # plt.plot(time, phi, label='phi')
        # plt.plot(data_dict['actual_rpm_time'],Mx, label='Mx')

        print('******************************************************')
        # plt.grid('True')
        # plt.show()

    NC = 2
    NR0 = len(phi_avg_list0)
    NR1 = len(phi_avg_list1)

    A0 = []
    A1 = []

    for i in range(NR0):
        row = []
        row.append([W0*np.cos(np.deg2rad(phi_avg_list0[i])),
                    -W0*np.sin(np.deg2rad(phi_avg_list0[i]))])
        A0.append(row)

    for i in range(NR1):
        row = []
        row.append([W1 * np.cos(np.deg2rad(phi_avg_list1[i])),
                    -W1 * np.sin(np.deg2rad(phi_avg_list1[i]))])
        A1.append(row)

    A_mat0 = np.array(A0).reshape(NR0,NC)
    Mx_vec0 = np.array(Mx_avg_list0).reshape(-1,1)

    A_mat1 = np.array(A1).reshape(NR1, NC)
    Mx_vec1 = np.array(Mx_avg_list1).reshape(-1, 1)

    # print(A_mat0)
    print(A_mat0.shape)
    print(Mx_vec0.shape)
    y_z_est0 = inv(np.transpose(A_mat0) @ A_mat0) @ np.transpose(A_mat0) @ Mx_vec0
    y_z_est1 = inv(np.transpose(A_mat1) @ A_mat1) @ np.transpose(A_mat1) @ Mx_vec1

    print('Estimated y_COM: ', y_z_est0[0]*1000,'mm')
    print('Estimated z_COM: ', y_z_est0[1]*1000,'mm')

    print('After weight added')

    print('Estimated y_COM: ', y_z_est1[0]*1000,'mm')
    print('Estimated z_COM: ', y_z_est1[1]*1000,'mm')

    # Theoretical z_COM
    L_added = -0.200
    z_COM_theory = (m0*y_z_est0[1] + m_added*L_added)/(m0 + m_added)
    print('Theoretical z_COM: ', z_COM_theory*1000,' mm')

    # Error between Theoretical z_COM and Experimental z_COM
    error = y_z_est1[1] - z_COM_theory
    print('Error: ', error*1000, ' mm')

    # Data : Convert list to np.array
    phi_avg_array = np.array(phi_avg_list0)
    Mx_array = np.array(Mx_avg_list0)

    # Prediction Storage
    phi_domain = np.linspace(-40,40,100)
    tau_predicted = (W0*y_z_est0[0]*np.cos(np.deg2rad(phi_domain))
                     - W0*y_z_est0[1]*np.sin(np.deg2rad(phi_domain)))


    plt.plot(phi_avg_list0, Mx_array*1000, 'x', label='Data')
    plt.plot(phi_domain, tau_predicted*1000, label='Predicted')
    plt.xlabel(r'$\phi$ (deg)')
    plt.ylabel(r'$\tau$ (mNm)')
    plt.legend()
    plt.grid('True')
    plt.show()


