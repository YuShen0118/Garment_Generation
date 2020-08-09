import time
import os
import numpy as np
import torch
from torch.autograd import Variable
from collections import OrderedDict
from subprocess import call
import fractions
import sys

from array import array
import torchvision.transforms as transforms
from random import gauss
import os.path as OS_path

from util.compute_maps import compute_maps

import multiprocessing


def dealWithOneCase(objFileName, bodyFileName):
    #TODO
    dispMap = []
    for i in range(512*512*3):
        dispMap.append(gauss(0, 1))

    dpMap = []
    for i in range(512*512*3):
        dpMap.append(gauss(0, 1))

    legalMap = []
    for i in range(512*512*1):
        legalMap.append(gauss(0, 1))
    return dispMap, dpMap, legalMap

def saveOnefile(filename, data):
    output_file = open(filename, 'wb')
    data = np.asarray(data.detach().cpu())
    data = np.reshape(data, data.size).tolist()
    float_array = array('f', data)
    float_array.tofile(output_file)
    output_file.close()

def saveFiles(path, idx, dispMap, dpMap, legalMap):
    dispMapFileName = path + "/train_displacement_map/displacement_map_" + format(idx, '06d') + ".dat"
    print(dispMapFileName)
    saveOnefile(dispMapFileName, dispMap)
    
    dpMapFileName = path + "/reconstruct_dp_map/dp_map_" + format(idx, '06d') + ".dat"
    saveOnefile(dpMapFileName, dpMap)

    legalMapFileName = path + "/train_legal_map/legal_map_" + format(idx, '06d') + ".dat"
    saveOnefile(legalMapFileName, legalMap)


def singleObjWithId(path, outPath, c, m, u):

    idx = c*10*250 + m*250 + u
    print("idx " + str(idx) + " begin")
    if not os.path.exists(outPath+'/train_displacement_map'):
        os.makedirs(outPath+'/train_displacement_map')
    if not os.path.exists(outPath+'/reconstruct_dp_map'):
        os.makedirs(outPath+'/reconstruct_dp_map')
    if not os.path.exists(outPath+'/train_legal_map'):
        os.makedirs(outPath+'/train_legal_map')
    
    h = 0
    while h < 100:
        filename = "C" + format(c, '03d') + "M" + format(m, '02d') + "H" + format(h, '02d') + "/" + format(u, '04d') + "_00.obj"
        if (OS_path.exists(path + filename)):
            break
        h = h + 1

    if (h >= 100):
        print("idx " + str(idx) + " end")
        return

    filename = "C" + format(c, '03d') + "M" + format(m, '02d') + "H" + format(h, '02d') + "/" + format(u, '04d') + "_00.obj"
    objFileName = path + filename
    print(objFileName)

    filename = "C" + format(c, '03d') + "M" + format(m, '02d') + "H" + format(h, '02d') + "/obs" + format(u, '04d') + "_00.obj"
    bodyFileName = path + filename
    print(bodyFileName)

    dispMap, dpMap, legalMap = compute_maps(objFileName, bodyFileName)

    saveFiles(outPath, idx, dispMap, dpMap, legalMap)

if __name__ == "__main__":
    singleObjWithId(sys.argv[1]+'/', sys.argv[2]+'/', int(sys.argv[3]),int(sys.argv[4]),int(sys.argv[5]))
    #singleObjWithId(0, 0, 0)

'''
    legal_map = getData()
    filename = path+"legal_map_000000.dat"
    #filename = path+"train_legal_map/legal_map_000000.dat"
    output_file = open(filename, 'wb')
    float_array = array('f', legal_map)
    float_array.tofile(output_file)
    output_file.close()
    legal_map_reshape = np.reshape(legal_map, (512,512,3))

    input_file = open(filename, 'rb')
    legal_map1 = array('f')
    legal_map1.fromstring(input_file.read())
    legal_map1_reshape = np.reshape(legal_map1, (512,512,3))

    legal_map_delta = legal_map_reshape - legal_map1_reshape

    print(np.ndarray.max(legal_map_reshape))
    print(np.ndarray.min(legal_map_reshape))

    print(np.ndarray.max(legal_map_delta))
    print(np.ndarray.min(legal_map_delta))
'''