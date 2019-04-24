# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

# examples/Python/Basic/icp_registration.py

from ransac import *
from team3RANSAC import *
import cv2
import numpy as np
import os
import sys
from open3d import *
import csv
import copy
from math import sqrt
# from sklearn import linear_model, datasets
from matplotlib import pyplot as plt
from open3d import *

#
# def draw_registration_result(source, target, transformation):
#     source_temp = copy.deepcopy(source)
#     target_temp = copy.deepcopy(target)
#     # source_temp.paint_uniform_color([1, 0.706,   0.0])
#     # target_temp.paint_uniform_color([0, 0.651, 0.929])
#     source_temp.transform(transformation)
#     draw_geometries([source_temp, target_temp])

if __name__ == "__main__":
    inputdir = sys.argv[1]#"./output_translated/"
    outputdir = sys.argv[2]#"./ICPOutput/"
    idx = []
    for file in os.listdir(inputdir):
        if file[-3:] != 'ply':
            continue
        idx = np.append(idx, file)
    idx.sort()

    # with open('VF_pointcloud_expanded_full.csv') as csv_file:
    #     csv_reader = csv.reader(csv_file, delimiter=',')
    #     next(csv_reader)
    #     data = [[] for i in range(4544)]  # 4544
    #     uniqueID = [[] for i in range(4544)]  # 4544
    #     for row in csv_reader:
    #         step = int(row[0])
    #         data[step].append(row[4:])
    #         uniqueID[step].append(row[3])
    # distance = 0
    # theta_best = 0
    # transpose = np.zeros((1,3))
    # reading_range = np.add(range(3), 55)
    reading_range = range(3)
    # for i in range(len(idx)):
    for i in reading_range:
        if i == reading_range[0]:
        # if i == 0:
            output = read_point_cloud(inputdir + idx[i])
        #     output = read_point_cloud("./output_mymap/" + idx[i])
            # draw_geometries([source])
            write_point_cloud(outputdir + idx[i], output)

        else:
            file_source = inputdir + idx[i]
            # file_target = inputdir + idx[i-1]
            file_target = outputdir + idx[i - 1]
            # file_source = inputdir + idx[i-1]
            # file_target = inputdir + idx[i]
            # file_source = "./output_mymap/" + idx[i]
            # file_target = "./output_mymap/" + idx[i - 1]
            source = read_point_cloud(file_source)
            target = read_point_cloud(file_target)
            # draw_geometries([target])
            sourcePoints = np.asarray(source.points)
            targetPoints = np.asarray(target.points)

            sourceColors = np.asarray(source.colors)
            targetColors = np.asarray(target.colors)

            threshold = 0.02
            trans_init = [[1.0, 0.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0, 0.0],
                          [0.0, 0.0, 1.0, 0.0],
                          [0.0, 0.0, 0.0, 1.0]]


            reg_p2p = registration_icp(source, target, threshold, trans_init,
                                        TransformationEstimationPointToPoint())
            # reg_p2p = registration_icp(target, source, threshold, trans_init,
            #                                     TransformationEstimationPointToPoint())

            # downsource = voxel_down_sample(source, voxel_size=0.05)#0.05
            #
            # estimate_normals(downsource, search_param=KDTreeSearchParamHybrid(
            #     radius=0.1, max_nn=30))
            #
            # downtarget = voxel_down_sample(target, voxel_size=0.05)
            #
            # estimate_normals(downtarget, search_param=KDTreeSearchParamHybrid(
            #     radius=0.1, max_nn=30))
            #
            # # reg_p2l = registration_icp(downsource, downtarget, threshold, reg_p2p.transformation,
            # #                            TransformationEstimationPointToPlane())
            # reg_p2l = registration_icp(downsource, downtarget, threshold, trans_init,
            #                            TransformationEstimationPointToPlane())

            augSourcePoints = np.ones((len(sourcePoints), 4))
            augSourcePoints[:, :3] = sourcePoints

            augDisplacedSourcePoints = np.matmul(reg_p2p.transformation, augSourcePoints.T)
            displacedSourcePoints = augDisplacedSourcePoints[:3, :]
            print(displacedSourcePoints.shape)
            displacedSourcePoints = displacedSourcePoints.T

###################################
            stepout = PointCloud()
            stepout.points = Vector3dVector(displacedSourcePoints)
            stepout.colors = Vector3dVector(sourceColors)
            write_point_cloud(outputdir + idx[i], stepout)

            outputPoints = np.asarray(output.points)
            outputColors = np.asarray(output.colors)
            finalPoints = np.append(outputPoints, displacedSourcePoints, axis=0)
            finalColors = np.append(outputColors, sourceColors, axis=0)

            output = PointCloud()
            output.points = Vector3dVector(finalPoints)
            output.colors = Vector3dVector(finalColors)

###################################################
    outputPoints = np.asarray(output.points)
    outputColors = np.asarray(output.colors)
    trans = np.load('transform.npy')
    output_large = np.ones((4, len(outputPoints)))
    output_large[:3, :] = outputPoints.T
    outputPoints = np.matmul(trans, output_large)
    outputPoints = outputPoints[:3, :]
    outputPoints = np.copy(outputPoints.T)
######################################
    # output_notzeros = np.copy(~np.all(outputPoints == 0, axis=1))
    # outputPoints = outputPoints[output_notzeros]
    # outputColors = outputColors[output_notzeros]
    output = PointCloud()
    output.points = Vector3dVector(outputPoints)
    output.colors = Vector3dVector(outputColors)
    write_point_cloud(outputdir+'/ICPCloud.ply', output)
