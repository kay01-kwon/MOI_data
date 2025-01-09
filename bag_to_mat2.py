import os, fnmatch
import rosbag
import numpy as np
import scipy.io
from scipy import interpolate

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
    actual_rpm_time = []
    actual_rpm_data = []

    imu_time = []
    imu_quaternion = []
    imu_angular_velocity = []

    for topic, msg, time in bag.read_messages(topics=['/actual_rpm', '/mavros/imu/data']):

        if topic == '/actual_rpm':
            actual_rpm_time.append(time.to_sec())
            actual_rpm_data.append([msg.rpm[0], msg.rpm[1], msg.rpm[2], msg.rpm[3]])
        elif topic == '/mavros/imu/data':
            imu_time.append(time.to_sec())
            imu_quaternion.append([msg.orientation.w,
                                   msg.orientation.x,
                                   msg.orientation.y,
                                   msg.orientation.z])
            imu_angular_velocity.append([msg.angular_velocity.x,
                                         msg.angular_velocity.y,
                                         msg.angular_velocity.z])

    actual_rpm_time_array = np.array(actual_rpm_time)
    actual_rpm_data_array = np.array(actual_rpm_data)
    imu_time_array = np.array(imu_time)
    imu_quaternion_array = np.array(imu_quaternion)
    imu_angular_velocity_array = np.array(imu_angular_velocity)

    # Interpolate RPM data to match IMU timestamps
    interp_rpm = interpolate.interp1d(actual_rpm_time_array, actual_rpm_data_array, axis=0, fill_value='extrapolate')
    syncronized_rpm = interp_rpm(imu_time_array)

    data_dict = {'imu_time': imu_time_array,
                 'imu_quaternion': imu_quaternion_array,
                 'imu_angular_velocity': imu_angular_velocity_array,
                 'rpm_data': syncronized_rpm}

    bag.close()

    return data_dict

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