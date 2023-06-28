#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov  5 21:54:55 2022

@author: guanbo
"""
# from math import cos, sin, pi
import numpy as np
import json
import cv2
import pandas as pd
import copy
import glob
import os

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# chessboard factor
chessboardSize = (5,7)
size_of_chessboard_squares_mm = 18.5

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)
objp = objp * size_of_chessboard_squares_mm

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('image/*.png')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,5), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        # img = cv2.drawChessboardCorners(img, (7,5), corners2, ret)
        # cv2.imshow('img', img)
        # cv2.waitKey(500)

cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# print(ret, mtx, dist, rvecs, tvecs)



# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

#Set image path
image_path = glob.glob('image/*.png')
#Read camera data
# with open('nerf_images/calibrated_data.json', 'r') as openfile:
#     # Reading from json file
#     json_object = json.load(openfile)
#     newCameraMatrix = json_object['newCameraMatrix']
#     dist = json_object['dist']
#     cameraMatrix = json_object['CameraMatrix']
cameraMatrix = np.array(mtx) 
dist = np.array(dist)
#Read robot pose data
dataframe = pd.read_csv("image/pose_data.csv", header=None)
pose_data = dataframe.to_numpy()[:,1:7]

def rv2rm(rx, ry, rz):
    theta = np.linalg.norm([rx, ry, rz])
    kx = rx / theta
    ky = ry / theta
    kz = rz / theta

    c = np.cos(theta)
    s = np.sin(theta)
    v = 1 - c

    R = np.zeros((3, 3))
    R[0][0] = kx * kx * v + c
    R[0][1] = kx * ky * v - kz * s
    R[0][2] = kx * kz * v + ky * s

    R[1][0] = ky * kx * v + kz * s
    R[1][1] = ky * ky * v + c
    R[1][2] = ky * kz * v - kx * s

    R[2][0] = kz * kx * v - ky * s
    R[2][1] = kz * ky * v + kx * s
    R[2][2] = kz * kz * v + c

    return R

def pose2Homo_tran_mat(tv,rv):
    Rm = cv2.Rodrigues(rv)[0]
    print(rv, Rm)
    Homo_mat = np.column_stack([Rm, tv])  
    Homo_mat = np.row_stack((Homo_mat, np.array([0,0,0,1])))
    return Homo_mat

def point_swap(corners2):
    newcorners = copy.copy(corners2)
    corners2_0 = corners2[0:5,0,:]
    corners2_1 = corners2[5:10,0,:]
    corners2_2 = corners2[10:15,0,:]
    corners2_3 = corners2[15:20,0,:]
    corners2_4 = corners2[20:25,0,:]
    corners2_5 = corners2[25:30,0,:]
    corners2_6 = corners2[30:35,0,:]
    newcorners[0:5,0,:] = corners2_6
    newcorners[5:10,0,:] = corners2_5
    newcorners[10:15,0,:] = corners2_4
    newcorners[15:20,0,:] = corners2_3
    newcorners[20:25,0,:] = corners2_2
    newcorners[25:30,0,:] = corners2_1
    newcorners[30:35,0,:] = corners2_0
    return newcorners

def get_robot_R_T(pose_data):
    R_rob = []
    T_rob = []
    for i in range(pose_data.shape[0]):
        tv = pose_data[i,:3]
        rv = pose_data[i,-3:]
        H = pose2Homo_tran_mat(tv, rv)
        R_rob.append(H[:3,:3])
        T_rob.append(H[:3,3]*1000)
    return R_rob,T_rob

def get_camera_H(img_path,chessboardSize, cameraMatrix,dist,size_of_chessboard_squares_mm):
    img=cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    size = gray.shape[::-1]
    ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)
    if ret == True:
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        img = cv2.drawChessboardCorners(img, (7,5), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)
        newcorners = corners2
        # if corners2[0,0,0] > corners2[5,0,0]:
            # newcorners = point_swap(corners2)
            # img = cv2.drawChessboardCorners(img, (7,5), newcorners, ret)
            # cv2.imshow('img', img)
        cv2.waitKey(500)
        ret,rvec,tvec  = cv2.solvePnP(objp,newcorners, cameraMatrix, dist)
        H = pose2Homo_tran_mat(tvec, rvec)
        return H
    else: 
        return 0

def get_camera_R_T(img_path,chessboardSize, cameraMatrix,dist,size_of_chessboard_squares_mm):
    R_cam = []
    T_cam = []
    for i in range(len(image_path)):
        path = image_path[0][:6] + str(i+1) + image_path[0][7:]
        H = get_camera_H(path, chessboardSize, cameraMatrix, dist, size_of_chessboard_squares_mm)
        R_cam.append(H[:3,:3])
        T_cam.append(H[:3,3])
    return R_cam,T_cam

def refine_input_R_T(R_rob,T_rob,R_cam,T_cam):
    for i in range(len(R_rob)):
        if (R_cam[i]==0).all():
            R_rob.pop(i)
            T_rob.pop(i)
            R_cam.pop(i)
            T_cam.pop(i)
    return R_rob,T_rob,R_cam,T_cam


R_rob,T_rob = get_robot_R_T(pose_data)
R_cam,T_cam = get_camera_R_T(image_path, chessboardSize, cameraMatrix, dist, size_of_chessboard_squares_mm)
R_rob,T_rob,R_cam,T_cam = refine_input_R_T(R_rob, T_rob, R_cam, T_cam)
R_rob2cam,T_rob2cam = cv2.calibrateHandEye(R_rob,T_rob,R_cam,T_cam)
print(T_cam[0], T_rob[0])
print(R_rob2cam)
print(T_rob2cam)

from scipy.spatial.transform import Rotation as R

r = R.from_matrix(R_rob2cam)
print(r.as_quat())