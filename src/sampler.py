"""
Description: Robotic Arm Motion Control Algorithm
Author: Zhang-sklda 845603757@qq.com
Date: 2025-11-25 23:07:14
Version: 1.0.0
LastEditors: Zhang-sklda 845603757@qq.com
LastEditTime: 2025-11-26 22:08:11
FilePath: /reachability_project/src/sampler.py
Copyright (c) 2025 by Zhang-sklda, All Rights Reserved.
symbol_custom_string_obkoro1_tech: Tech: Motion Control | MuJoCo | ROS | Kinematics
"""

import numpy as np  
from tqdm import tqdm
from robot_model import RobotModel
from reachability_map import ReachabilityMap
import transforms3d

"""
brief: 
param {RobotModel} robot_model
param {ReachabilityMap} rmap
param {*} sample
return {*}
"""
def sample_fk_and_fill(robot_model: RobotModel,rmap:ReachabilityMap,sample=50000 ,report_interval=1000):
    """FK random sampling generation (fast)."""
    new_bin_buffer = 0  # 统计最近 interval 次新设置的 bins
    global_new = 0      # 统计全局新发现的 bins

    for _ in tqdm(range(sample)):
        # 随机关节采样   从 joint limit 内等概率采样   得到一个机械臂姿态 q
        joint_angles = robot_model.random_joint_sample()
        position, orientation = robot_model.fk(joint_angles)
        # 根据论文的 reachability map 格子结构  将连续的 SE(3) 空间分解为 voxel grid   只保留落在工作空间体素中的样本
        ic = rmap.fc(position[0], position[1], position[2])
        if ic is None:
            continue
        approach = orientation[:, 2]
        ia = rmap.fa(approach)

        x_axis = orientation[:, 0]
        ref = np.array([1.0, 0.0, 0.0])

        # roll 角 γ 计算
        ref = ref - approach * np.dot(ref, approach)
        if np.linalg.norm(ref) < 1e-6:
            ref = np.array([0.0, 1.0, 0.0])
            ref = ref - approach * np.dot(ref, approach)
        ref = ref / np.linalg.norm(ref)

        # 计算 x 轴相对 ref 的旋转角
        gamma = np.arctan2(np.dot(np.cross(ref, x_axis), approach), np.dot(ref, x_axis))
        ir = rmap.fr(gamma)

        is_new = rmap.set_bin(ic,ia,ir)
        if is_new:
            new_bin_buffer += 1
            global_new += 1

        # 每 interval 次报告统计
        if (_ + 1) % report_interval == 0:
            total_true = rmap.count_true_bins()
            recent_rate = new_bin_buffer / report_interval

            print("\n========================= Reachability Progress =========================")
            print(f"Samples processed: {_+1}")
            print(f"Global new bins discovered so far: {global_new}")
            print(f"Total reachable bins marked in map: {total_true}")
            print(f"New bin discovery rate in last {report_interval} samples: {recent_rate:.6f}")
            print("=======================================================================\n")

            new_bin_buffer = 0  # 清空窗口统计