"""
Description: Sampler 测试代码
Author: Zhang-sklda 845603757@qq.com
Date: 2025-11-25 23:45:00
Version: 1.0.0
LastEditors: Zhang-sklda 845603757@qq.com
LastEditTime: 2025-11-25 23:45:00
FilePath: /reachability_project/test/test_sampler.py
Copyright (c) 2025 by Zhang-sklda, All Rights Reserved.
symbol_custom_string_obkoro1_tech: Tech: Motion Control | MuJoCo | ROS | Kinematics | Reachability Analysis
"""
import numpy as np
import os
import sys

# 添加路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from robot_model import RobotModel
from reachability_map import ReachabilityMap
from sampler import sample_fk_and_fill


class TestSampler:
    """测试采样器功能"""
    
    def setup_method(self):
        """设置测试环境"""
        # 使用真实模型路径
        self.model_path = "/home/zhang/reachability_project/models/iiwa14.xml"
        
        # 检查模型文件是否存在
        if not os.path.exists(self.model_path):
            import pytest
            pytest.skip(f"模型文件不存在: {self.model_path}")
        
        # 初始化机器人模型
        self.robot = RobotModel(self.model_path)
        
        # 创建小的可达性地图用于测试
        self.reach_map = ReachabilityMap(
            origin=[-0.5, -0.5, 0.0],  # 工作空间原点
            dims=[10, 10, 10],         # 10x10x10体素
            voxel_r=0.1,               # 10cm分辨率
            da=8,                      # 8个方向（简化测试）
            dr=4                       # 4个滚转角度（简化测试）
        )
    
    def test_sampler_initialization(self):
        """测试采样器相关组件的初始化"""
        print("=== 测试采样器初始化 ===")
        
        # 检查机器人模型
        assert self.robot is not None
        assert self.robot.model is not None
        assert self.robot.data is not None
        assert len(self.robot.joint_names) == 7
        
        # 检查可达性地图
        assert self.reach_map is not None
        assert len(self.reach_map.bits) == 10 * 10 * 10 * 8 * 4  # 32000 bits
        
        print("✓ 采样器初始化测试通过")
    
    def test_single_fk_sample(self):
        """测试单个FK采样"""
        print("\n=== 测试单个FK采样 ===")
        
        # 记录初始覆盖率
        initial_coverage = self.reach_map.get_coverage()
        print(f"初始覆盖率: {initial_coverage:.6f}")
        
        # 手动执行一次采样流程
        q = self.robot.random_joint_sample()
        print(f"随机关节角度: {q}")
        
        position, orientation = self.robot.fk(q)
        print(f"FK结果 - 位置: {position}, 方向矩阵形状: {orientation.shape}")
        
        # 计算索引
        ic = self.reach_map.fc(position[0], position[1], position[2])
        approach = orientation[:, 2]
        ia = self.reach_map.fa(approach)
        
        # 计算滚转角（使用采样器中的方法）
        x_axis = orientation[:, 0]
        ref = np.array([1.0, 0.0, 0.0])
        ref = ref - approach * np.dot(ref, approach)
        if np.linalg.norm(ref) < 1e-6:
            ref = np.array([0.0, 1.0, 0.0])
            ref = ref - approach * np.dot(ref, approach)
        ref = ref / np.linalg.norm(ref)
        gamma = np.arctan2(np.dot(np.cross(ref, x_axis), approach), np.dot(ref, x_axis))
        ir = self.reach_map.fr(gamma)
        
        print(f"计算得到的索引: ic={ic}, ia={ia}, ir={ir}")
        
        # 验证索引在有效范围内
        if ic is not None:
            assert 0 <= ic < self.reach_map.voxel_count
            assert 0 <= ia < self.reach_map.da
            assert 0 <= ir < self.reach_map.dr
            
            # 设置并验证
            initial_state = self.reach_map.query_bin(ic, ia, ir)
            self.reach_map.set_bin(ic, ia, ir)
            final_state = self.reach_map.query_bin(ic, ia, ir)
            
            assert not initial_state, "初始状态应该为False"
            assert final_state, "设置后状态应该为True"
            
            print("✓ 单个采样设置验证通过")
        else:
            print("位置在工作空间外，跳过设置验证")
        
        print("✓ 单个FK采样测试通过")
    
    def test_sample_fk_and_fill_small(self):
        """测试小批量FK采样"""
        print("\n=== 测试小批量FK采样 ===")
        
        initial_coverage = self.reach_map.get_coverage()
        print(f"采样前覆盖率: {initial_coverage:.6f}")
        
        # 运行小批量采样
        sample_count = 1000
        sample_fk_and_fill(self.robot, self.reach_map, sample_count)
        
        final_coverage = self.reach_map.get_coverage()
        print(f"采样后覆盖率: {final_coverage:.6f}")
        
        # 验证覆盖率增加
        assert final_coverage > initial_coverage, "采样后覆盖率应该增加"
        assert final_coverage > 0, "采样后覆盖率应该大于0"
        
        print("✓ 小批量FK采样测试通过")
    
    def test_sampler_statistics(self):
        """测试采样统计信息"""
        print("\n=== 测试采样统计 ===")
        
        # 重置可达性地图
        self.reach_map.bits.setall(False)
        
        initial_coverage = self.reach_map.get_coverage()
        assert abs(initial_coverage - 0.0) < 1e-6, "重置后覆盖率应该为0"
        
        # 运行采样
        sample_count = 500
        sample_fk_and_fill(self.robot, self.reach_map, sample_count)
        
        final_coverage = self.reach_map.get_coverage()
        
        # 统计被设置的体素数量
        filled_bins = self.reach_map.bits.count()
        total_bins = len(self.reach_map.bits)
        
        print(f"总位数: {total_bins}")
        print(f"设置的位数: {filled_bins}")
        print(f"计算覆盖率: {filled_bins / total_bins:.6f}")
        print(f"返回覆盖率: {final_coverage:.6f}")
        
        # 验证一致性
        assert abs(filled_bins / total_bins - final_coverage) < 1e-10, "覆盖率计算应该一致"
        
        print("✓ 采样统计测试通过")
    
    def test_roll_angle_calculation(self):
        """测试滚转角计算"""
        print("\n=== 测试滚转角计算 ===")
        
        # 测试几个已知的旋转矩阵
        test_cases = [
            (np.eye(3), 0.0, "单位矩阵，滚转0"),
            (np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]), np.pi/2, "绕Z轴旋转90度"),
            (np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]]), np.pi, "绕Z轴旋转180度"),
        ]
        
        for rot_matrix, expected_roll, description in test_cases:
            # 使用采样器中的方法计算滚转角
            approach = rot_matrix[:, 2]
            x_axis = rot_matrix[:, 0]
            ref = np.array([1.0, 0.0, 0.0])
            ref = ref - approach * np.dot(ref, approach)
            if np.linalg.norm(ref) < 1e-6:
                ref = np.array([0.0, 1.0, 0.0])
                ref = ref - approach * np.dot(ref, approach)
            ref = ref / np.linalg.norm(ref)
            computed_roll = np.arctan2(np.dot(np.cross(ref, x_axis), approach), np.dot(ref, x_axis))
            
            # 归一化角度到 [0, 2pi) 以便比较
            computed_roll_norm = computed_roll % (2 * np.pi)
            expected_roll_norm = expected_roll % (2 * np.pi)
            
            print(f"{description}: 期望 {expected_roll_norm:.3f}, 计算 {computed_roll_norm:.3f}")
            
            # 允许小的数值误差
            assert abs(computed_roll_norm - expected_roll_norm) < 1e-10, \
                f"滚转角计算错误: {description}"
        
        print("✓ 滚转角计算测试通过")
    
    def test_edge_cases(self):
        """测试边界情况"""
        print("\n=== 测试边界情况 ===")
        
        # 测试工作空间外的点
        out_of_bounds_positions = [
            [-1.0, 0.0, 0.0],  # X太小
            [1.0, 0.0, 0.0],   # X太大
            [0.0, -1.0, 0.0],  # Y太小
            [0.0, 1.0, 0.0],   # Y太大
        ]
        
        for pos in out_of_bounds_positions:
            ic = self.reach_map.fc(pos[0], pos[1], pos[2])
            assert ic is None, f"位置 {pos} 应该返回 None"
        
        print("✓ 边界情况测试通过")


def run_sampler_tests():
    """运行所有采样器测试"""
    print("开始测试 Sampler 功能...")
    print("=" * 60)
    
    test_instance = TestSampler()
    
    try:
        test_instance.setup_method()
        test_instance.test_sampler_initialization()
        test_instance.test_single_fk_sample()
        test_instance.test_sample_fk_and_fill_small()
        test_instance.test_sampler_statistics()
        test_instance.test_roll_angle_calculation()
        test_instance.test_edge_cases()
        
        print("=" * 60)
        print("🎉 所有采样器测试通过！sample_fk_and_fill 函数实现正确。")
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        raise


if __name__ == "__main__":
    run_sampler_tests()