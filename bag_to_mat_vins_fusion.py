import os, fnmatch
import rosbag
# import csv
import pickle
import numpy as np
import scipy.io
from scipy import interpolate

def find_files(directory):
    file_names = []

    for root, dirs, files in os.walk(directory):
        for name in files:
            if fnmatch.fnmatch(name, '*.bag'):
                file_names.append(name)
    return file_names

def get_data(file_name):

    print(file_name)

    bag = rosbag.Bag(file_name)

    # Actual rpm data list to store
    actual_rpm_time = []
    actual_rpm = []

    # IMU data list to store
    imu_time = []
    imu_quaternion = []
    imu_angular_velocity = []

    # VINS FUSION - GPU data list to store
    vins_time = []
    vins_quaternion = []

    for topic, msg, time in bag.read_messages(topics=['/actual_rpm', '/imu/data','/vins_estimator/odometry']):

        if topic == '/actual_rpm':
            # Append actual rpm time and data
            actual_rpm_time.append(time.to_sec())
            actual_rpm.append([msg.rpm[0],
                               msg.rpm[1],
                               msg.rpm[2],
                               msg.rpm[3]])
        elif topic == '/imu/data':

            # Append imu time, quaternion,
            # and angular velocity
            imu_time.append(time.to_sec())

            imu_quaternion.append([msg.orientation.w,
                                   msg.orientation.x,
                                   msg.orientation.y,
                                   msg.orientation.z])

            imu_angular_velocity.append([msg.angular_velocity.x,
                                         msg.angular_velocity.y,
                                         msg.angular_velocity.z])

        elif topic == '/vins_estimator/odometry':
            # Append quaternion from vins fusion
            vins_time.append(time.to_sec())
            vins_quaternion.append([msg.pose.pose.orientation.w,
                                    msg.pose.pose.orientation.x,
                                    msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z])


        # Create array to interpolate and synchronize the vins fusion data
    actual_rpm_time_array = np.array(actual_rpm_time)
    actual_rpm_array = np.array(actual_rpm)

    imu_time_array = np.array(imu_time)
    imu_quaternion_array = np.array(imu_quaternion)
    imu_angular_velocity_array = np.array(imu_angular_velocity)

    vins_time_array = np.array(vins_time)
    vins_quaternion_array = np.array(vins_quaternion)

        # Create interpolation object
    interp_rpm_obj = interpolate.interp1d(actual_rpm_time_array,
                                          actual_rpm_array,
                                          axis = 0,
                                          fill_value = 'extrapolate')

    interp_imu_obj = interpolate.interp1d(imu_time_array,
                                          imu_angular_velocity_array,
                                          axis = 0,
                                          fill_value = 'extrapolate')

    synchronized_rpm = interp_rpm_obj(vins_time_array)

    synchronized_imu_angular_velocity = interp_imu_obj(vins_time_array)

    data_dictionary = {
        'vins_time': vins_time_array,
        'synchronized_rpm': synchronized_rpm,
        'synchronized_imu': synchronized_imu_angular_velocity,
        'vins_quaternion': vins_quaternion_array
    }
    bag.close()

    return data_dictionary

def write_pickle(file_name, data_dictionary):

    # try:
    #     with open(file_name, 'r') as csvfile:
    #         pass
    #     write_header = False
    # except FileNotFoundError:
    #     write_header = True # File does not exist
    pickle_file = open(file_name, 'wb')
    pickle.dump(data_dictionary, pickle_file)
    pickle_file.close()


if __name__ == '__main__':
    directory = 'J_zz_v2'

    dir_str_len = len(directory)
    file_names = find_files(directory)
    sorted_file_names = sorted(file_names)
    mat_folder_directory = directory + '/' + 'mat_folder'

    for file_name in sorted_file_names:
        data_dict = []
        data_dict = get_data(directory + '/' + file_name)
        bag_name = file_name.replace(directory, '')
        bag_name = bag_name.replace('.bag', '')
        mat = scipy.io.savemat(mat_folder_directory + '/' + bag_name + '.mat',
                               {'dict_array': data_dict})
