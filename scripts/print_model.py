#!/usr/bin/env python3 
import open3d as o3d
import numpy as np
import os
import copy

pcd_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../pcd/")
pcd = o3d.io.read_point_cloud(pcd_dir + "pcd.pcd")
