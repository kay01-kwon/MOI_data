import os, fnmatch
import rosbag
import numpy as np
import matplotlib.pyplot as plt
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


    for topic, msg, time in bag.read_messages(topics=['/actual_rpm', '/imu/data']):

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


        # Create array to interpolate and synchronize the vins fusion data
    actual_rpm_time_array = np.array(actual_rpm_time)
    actual_rpm_array = np.array(actual_rpm)

    imu_time_array = np.array(imu_time)
    imu_quaternion_array = np.array(imu_quaternion)
    imu_angular_velocity_array = np.array(imu_angular_velocity)


        # Create interpolation object
    interp_rpm_obj = interpolate.interp1d(actual_rpm_time_array,
                                          actual_rpm_array,
                                          axis = 0,
                                          fill_value = 'extrapolate')

    synchronized_rpm = interp_rpm_obj(imu_time_array)

    data_dictionary = {
        'imu_time': imu_time,
        'synchronized_rpm': synchronized_rpm,
        'imu_quaternion': imu_quaternion_array,
        'imu_angular_velocity': imu_angular_velocity_array
    }
    bag.close()

    return data_dictionary


if __name__ == '__main__':
    directory = 'bag/J_zz_data'

    file_names = find_files(directory)
    sorted_file_names = sorted(file_names)
    number_of_files = len(sorted_file_names)

    print('Number of files: ' + str(number_of_files))

    mat_folder_directory = directory + '/' + 'mat_folder'

    for file_name in sorted_file_names:
        data_dict = []
        data_dict = get_data(directory + '/' + file_name)
        plt.plot(data_dict['imu_time'], data_dict['imu_angular_velocity'][:,2])
        plt.show()