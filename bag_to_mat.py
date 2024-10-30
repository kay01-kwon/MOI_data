import os, fnmatch
import rosbag
from motor_control_msg.msg import actual_value
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
    actual_value_secs = []
    actual_value_nsecs = []
    actual_value_pos = []
    actual_value_vel = []

    for topic, msg, time in bag.read_messages(topics=['/actual_data']):
        actual_value_secs.append(msg.stamp.secs)
        actual_value_nsecs.append(msg.stamp.nsecs)
        actual_value_pos.append(msg.actual_pos)
        actual_value_vel.append(msg.actual_vel)


    actual_value_dict = {'actual_data_secs': actual_value_secs,
                         'actual_data_nsecs': actual_value_nsecs,
                         'actual_data_pos': actual_value_pos,
                         'actual_data_vel': actual_value_vel}

    bag.close()

    return actual_value_dict

if __name__ == '__main__':

    directory = 'J_xx'

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