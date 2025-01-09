import pickle
import numpy as np
import os, fnmatch

def find_files(directory):
    file_names = []

    for root, dirs, files in os.walk(directory):
        for name in files:
            if fnmatch.fnmatch(name, '*.data'):
                file_names.append(name)
    return file_names

if __name__ == '__main__':
    directory = 'J_zz_v2/pickle/'

    dir_str_len = len(directory)
    file_names = find_files(directory)

    fd = open(directory+file_names[0], 'rb')
    data = pickle.load(fd)
    print(data)

