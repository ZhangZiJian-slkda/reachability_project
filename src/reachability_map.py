"""
Description: Robotic Arm Motion Control Algorithm
Author: Zhang-sklda 845603757@qq.com
Date: 2025-11-24 23:24:26
Version: 1.0.0
LastEditors: Zhang-sklda 845603757@qq.com
LastEditTime: 2025-11-26 22:18:14
FilePath: /reachability_project/src/reachability_map.py
Copyright (c) 2025 by Zhang-sklda, All Rights Reserved.
symbol_custom_string_obkoro1_tech: Tech: Motion Control | MuJoCo | ROS | Kinematics
"""
import numpy as np
import math
from bitarray import bitarray


class ReachabilityMap:
    """
    brief: 
    param {*} self
    param {*} origin
    param {*} dims
    param {*} voxel_r
    param {*} da
    param {*} dr
    param {*} my
    param {*} mz
    param {*} Dy
    param {*} Dz
    param {*} dtype
    param {*} self
    param {*} self
    return {*}
    """
    def __init__(self,origin,dims,voxel_r,da=50,dr=10):
        """
        origin: (mx,my,mz) lower corner of grid (meters)
        dims: (Dx,Dy,Dz) number of voxels along axes
        voxel_r: resolution in meters
        da: approach direction discretization (sphere)
        dr: roll discretization
        """
        self.origin = np.array(origin,dtype=float)
        self.Dx,self.Dy,self.Dz = dims
        self.r = voxel_r
        self.da = da
        self.dr = dr
        self.voxel_count = self.Dx * self.Dy * self.Dz
        self.bits_per_voxel = da * dr
        total_bits = self.voxel_count * self.bits_per_voxel
        # 使用bitarray节省内存
        self.bits = bitarray(total_bits)
        self.bits.setall(False)
        # 预计算方向采样（改进的球面采样）
        self.direction_samples = self._precompute_directions()


    def _precompute_directions(self):
        """预计算均匀球面采样方向"""
        directions = []
        golden_ratio = (1+math.sqrt(5))/2

        for i in range(self.da):
            theta = 2 * math.pi * i /golden_ratio
            phi = math.acos(1-2*(i+0.5)/self.da)
            x = math.sin(phi) * math.cos(theta)
            y = math.sin(phi) * math.sin(theta)
            z = math.cos(phi)
            directions.append(np.array([x,y,z]))
            
        return directions

    def fc(self,tx,ty,tz):
        """将三维位置映射到体素索引"""
        ix = int(math.floor((tx - self.origin[0]) / self.r))
        iy = int(math.floor((ty - self.origin[1]) / self.r))
        iz = int(math.floor((tz - self.origin[2]) / self.r))
        
        if not (0 <= ix < self.Dx and 0 <= iy < self.Dy and 0 <= iz < self.Dz):
            return None
        
        return ix + iy * self.Dx + iz * (self.Dx * self.Dy)

    def fa(self, direction_vec):
        """
        改进的方向映射：使用最近邻搜索在预计算的方向中
        """
        direction_vec = np.array(direction_vec)
        direction_vec = direction_vec / np.linalg.norm(direction_vec)  # 归一化
        
        # 找到最近的方向
        best_idx = 0
        best_dot = -1.0
        
        for i, sample_dir in enumerate(self.direction_samples):
            dot_product = np.dot(direction_vec, sample_dir)
            if dot_product > best_dot:
                best_dot = dot_product
                best_idx = i
        
        return best_idx

    def fr(self, gamma):
        """滚转角度映射"""
        # 规范化角度到 [0, 2pi)
        gamma_norm = gamma % (2 * math.pi)
        ir = int(math.floor(gamma_norm / (2 * math.pi / self.dr)))
        return min(max(0, ir), self.dr - 1)

    def set_bin(self, ic, ia, ir):
        """标记体素为可达"""
        if ic is None:
            return False
        bit_idx = ic * self.bits_per_voxel + ia * self.dr + ir
        if 0 <= bit_idx < len(self.bits):
            self.bits[bit_idx] = True
            return True

    def count_true_bins(self):
        """统计可达体素数量"""
        return self.bits.count(True)
    
    def count_true_voxels(self):
        """统计包含至少一个可达方向的体素数量"""
        count = 0
        for ix in range(self.voxel_count):
            if any(self.bits[ix * self.bits_per_voxel:(ix+1)*self.bits_per_voxel]):
                count += 1
        return count
    

    def query_bin(self, ic, ia, ir):
        """查询体素是否可达"""
        if ic is None:
            return False
        bit_idx = ic * self.bits_per_voxel + ia * self.dr + ir
        if 0 <= bit_idx < len(self.bits):
            return self.bits[bit_idx]
        return False

    def set_pose(self, position, rotation_matrix, roll_angle=0):
        """
        从机器人位姿设置可达性
        position: [x, y, z] 末端位置
        rotation_matrix: 3x3 旋转矩阵
        roll_angle: 绕z轴的滚转角度（弧度）
        """
        # 提取approach方向（通常为z轴）
        approach_dir = rotation_matrix[:, 2]
        
        # 计算索引
        ic = self.fc(position[0], position[1], position[2])
        ia = self.fa(approach_dir)
        ir = self.fr(roll_angle)
        
        # 设置可达性
        self.set_bin(ic, ia, ir)
        return ic is not None

    def query_pose(self, position, rotation_matrix, roll_angle=0):
        """查询位姿是否可达"""
        approach_dir = rotation_matrix[:, 2]
        ic = self.fc(position[0], position[1], position[2])
        ia = self.fa(approach_dir)
        ir = self.fr(roll_angle)
        return self.query_bin(ic, ia, ir)

    def get_coverage(self):
        """获取覆盖率统计"""
        total_bins = len(self.bits)
        filled_bins = self.bits.count()
        return filled_bins / total_bins if total_bins > 0 else 0
