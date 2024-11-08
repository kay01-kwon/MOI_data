import os, fnmatch
import rosbag
from ros_libcanard.msg import actual_rpm
from sensor_msgs.msg import Imu
import numpy as np
import scipy.io

def find_files(directory):
    file_names = []

    for root, dirs, files in os.walk(directory):
        for name in files:
            if fnmatch.fnmatch(name, '*.bag'):
                file_names.append(directory + '/' + name)
                print(directory+'/'+name)

    return file_names

def get_data(file_name):
    bag = rosbag.Bag(file_name)

    # Actual data to store
    actual_rpm_secs = []
    actual_rpm_nsecs = []
    actual_rpm1 = []
    actual_rpm2 = []
    actual_rpm3 = []
    actual_rpm4 = []

    for topic, msg, time in bag.read_messages(topics=['/actual_rpm']):
        actual_rpm_secs.append(msg.stamp.secs)
        actual_rpm_nsecs.append(msg.stamp.nsecs)
        actual_rpm1.append(msg.rpm[0])
        actual_rpm2.append(msg.rpm[1])
        actual_rpm3.append(msg.rpm[2])
        actual_rpm4.append(msg.rpm[3])

    actual_rpm_dict = {'actual_rpm_secs': actual_rpm_secs,
                         'actual_rpm_nsecs': actual_rpm_nsecs,
                         'actual_rpm1': actual_rpm1,
                         'actual_rpm2': actual_rpm2,
                         'actual_rpm3': actual_rpm3,
                         'actual_rpm4': actual_rpm4
                         }

    imu_secs = []
    imu_nsecs = []

    imu_qw = []
    imu_qx = []
    imu_qy = []
    imu_qz = []

    imu_wx = []
    imu_wy = []
    imu_wz = []

    for topic, msg, time in bag.read_messages(topics=['/mavros/imu/data']):
        imu_secs.append(msg.header.stamp.secs)
        imu_nsecs.append(msg.header.stamp.nsecs)

        imu_qw.append(msg.orientation.w)
        imu_qx.append(msg.orientation.x)
        imu_qy.append(msg.orientation.y)
        imu_qz.append(msg.orientation.z)

        imu_wx.append(msg.angular_velocity.x)
        imu_wy.append(msg.angular_velocity.y)
        imu_wz.append(msg.angular_velocity.z)

    imu_data_dict = {'imu_secs': imu_secs,
                     'imu_nsecs': imu_nsecs,
                     'imu_qw': imu_qw,
                     'imu_qx': imu_qx,
                     'imu_qy': imu_qy,
                     'imu_qz': imu_qz,
                     'imu_wx': imu_wx,
                     'imu_wy': imu_wy,
                     'imu_wz': imu_wz}

    bag.close()

    return actual_rpm_dict, imu_data_dict

if __name__ == '__main__':

    directory = 'J_zz'

    dir_str_len = len(directory)

    file_names = find_files(directory)

    mat_folder_directory = directory + '/' + 'mat_folder'


    for file_name in file_names:
        actual_data_dict = []
        actual_data_dict = get_data(file_name)
        bag_name = file_name.replace(directory, '')
        bag_name = bag_name.replace('.bag', '')
        mat = scipy.io.savemat(mat_folder_directory + '/' + bag_name + '.mat',
                               {'dict_array': actual_data_dict})