"""
Description: Robotic Arm Motion Control Algorithm
Author: Zhang-sklda 845603757@qq.com
Date: 2025-11-25 23:07:14
Version: 1.0.0
LastEditors: Zhang-sklda 845603757@qq.com
LastEditTime: 2025-11-25 23:47:36
FilePath: /reachability_project/src/sampler.py
Copyright (c) 2025 by Zhang-sklda, All Rights Reserved.
symbol_custom_string_obkoro1_tech: Tech: Motion Control | MuJoCo | ROS | Kinematics
"""

import numpy as np  
from tqdm import tqdm
from robot_model import RobotModel
from reachability_map import ReachabilityMap
import transforms3d

def sample_fk_and_fill(robot_model: RobotModel,rmap:ReachabilityMap,sample=50000):
    """FK random sampling generation (fast)."""
    for _ in tqdm(range(sample)):
        joint_angles = robot_model.random_joint_sample()
        position, orientation = robot_model.fk(joint_angles)
        ic = rmap.fc(position[0], position[1], position[2])
        if ic is None:
            continue
        approach = orientation[:, 2]
        ia = rmap.fa(approach)

        x_axis = orientation[:, 0]
        ref = np.array([1.0, 0.0, 0.0])
        ref = ref - approach * np.dot(ref, approach)
        if np.linalg.norm(ref) < 1e-6:
            ref = np.array([0.0, 1.0, 0.0])
            ref = ref - approach * np.dot(ref, approach)
        ref = ref / np.linalg.norm(ref)
        gamma = np.arctan2(np.dot(np.cross(ref, x_axis), approach), np.dot(ref, x_axis))
        ir = rmap.fr(gamma)
        rmap.set_bin(ic,ia,ir)