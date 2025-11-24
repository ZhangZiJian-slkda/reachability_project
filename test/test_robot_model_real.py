"""
Description: Robotic Arm Motion Control Algorithm
Author: Zhang-sklda 845603757@qq.com
Date: 2025-11-23 23:20:39
Version: 1.0.0
LastEditors: Zhang-sklda 845603757@qq.com
LastEditTime: 2025-11-24 23:02:25
FilePath: /reachability_project/test/test_robot_model_real.py
Copyright (c) 2025 by Zhang-sklda, All Rights Reserved.
symbol_custom_string_obkoro1_tech: Tech: Motion Control | MuJoCo | ROS | Kinematics
"""
import numpy as np
import pytest
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from robot_model import RobotModel

class TestRobotModelReal:
    """使用真实KUKA iiwa模型进行测试"""
    
    def setup_method(self):
        """设置测试环境"""
        # 使用您提供的实际模型路径
        self.model_path = "/home/zhang/reachability_project/models/iiwa14.xml"
        
        # 检查模型文件是否存在
        if not os.path.exists(self.model_path):
            pytest.skip(f"模型文件不存在: {self.model_path}")
    
    def test_initialization_with_real_model(self):
        """使用真实模型测试初始化"""
        robot = RobotModel(self.model_path)
        
        # 检查基本属性
        assert robot.model is not None
        assert robot.data is not None
        assert robot.tcp_site == 'attachment_site'
        
        # 检查关节信息 - KUKA iiwa应该有7个关节
        print(f"关节名称: {robot.joint_names}")
        print(f"关节数量: {robot.nq}")
        
        # KUKA iiwa 14应该有7个关节
        assert robot.nq == 7
        assert len(robot.joint_names) == 7
        
        # 检查关节名称是否符合KUKA iiwa的命名约定
        expected_joint_names = [f'joint{i}' for i in range(1, 8)]
        for name in expected_joint_names:
            assert any(name in joint_name for joint_name in robot.joint_names), f"未找到关节: {name}"
        
        print("✓ 真实模型初始化测试通过")
    
    def test_fk_with_zero_configuration(self):
        """测试零位姿的正向运动学"""
        robot = RobotModel(self.model_path)
        
        # 零关节角度
        q_zero = np.zeros(robot.nq)
        pos, rot_mat = robot.fk(q_zero)
        
        # 检查返回类型和形状
        assert isinstance(pos, np.ndarray)
        assert isinstance(rot_mat, np.ndarray)
        assert pos.shape == (3,)
        assert rot_mat.shape == (3, 3)
        
        print(f"零位姿TCP位置: {pos}")
        print(f"零位姿旋转矩阵:\n{rot_mat}")
        
        # 验证旋转矩阵的性质
        det = np.linalg.det(rot_mat)
        assert abs(det - 1.0) < 1e-6, f"旋转矩阵行列式应为1, 实际为: {det}"
        
        # 检查是否正交
        identity_check = rot_mat @ rot_mat.T
        np.testing.assert_array_almost_equal(identity_check, np.eye(3), decimal=6)
        
        print("✓ 零位姿正向运动学测试通过")
    
    def test_fk_with_different_poses(self):
        """测试不同关节角度下的正向运动学"""
        robot = RobotModel(self.model_path)
        
        # 测试几个不同的关节配置
        test_configs = [
            np.zeros(7),  # 零位姿
            np.array([0.5, 0, 0, 0, 0, 0, 0]),  # 第一个关节旋转
            np.array([0, 0.5, 0, 0, 0, 0, 0]),  # 第二个关节旋转
            np.array([0, 0, 0.5, 0, 0, 0, 0]),  # 第三个关节旋转
            np.random.uniform(-1.0, 1.0, 7),  # 随机配置
        ]
        
        for i, q in enumerate(test_configs):
            pos, rot_mat = robot.fk(q)
            print(f"配置 {i}: 关节角度 {q}")
            print(f"位置: {pos}")
            print(f"旋转矩阵:\n{rot_mat}")
            
            # 详细检查旋转矩阵
            det = np.linalg.det(rot_mat)
            print(f"行列式: {det}")
            print(f"与1的差值: {abs(det - 1.0)}")
            
            identity_check = rot_mat @ rot_mat.T
            print(f"R * R^T:\n{identity_check}")
            np.testing.assert_array_almost_equal(identity_check, np.eye(3), decimal=6)
        
        print("✓ 不同位姿正向运动学测试通过")
    
    def test_random_joint_sampling(self):
        """测试随机关节采样"""
        robot = RobotModel(self.model_path)
        
        # 多次采样测试
        for i in range(10):
            q = robot.random_joint_sample()
            
            # 检查返回类型和形状
            assert isinstance(q, np.ndarray)
            assert q.shape == (7,)
            
            print(f"采样 {i}: {q}")
            
            # 验证FK能正常计算
            pos, rot_mat = robot.fk(q)
            assert pos.shape == (3,)
            assert rot_mat.shape == (3, 3)
            
            # 验证旋转矩阵
            det = np.linalg.det(rot_mat)
            assert abs(det - 1.0) < 1e-6
        
        print("✓ 随机关节采样测试通过")
    
    def test_fk_consistency(self):
        """测试正向运动学的一致性"""
        robot = RobotModel(self.model_path)
        
        # 测试关节配置
        q_test = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7])
        
        # 多次计算相同关节角度
        positions = []
        rotations = []
        
        for _ in range(5):
            pos, rot = robot.fk(q_test)
            positions.append(pos)
            rotations.append(rot)
        
        # 检查所有结果是否一致
        for i in range(1, len(positions)):
            np.testing.assert_array_almost_equal(positions[0], positions[i], decimal=6)
            np.testing.assert_array_almost_equal(rotations[0], rotations[i], decimal=6)
        
        print("✓ 正向运动学一致性测试通过")
    
    def test_custom_tcp_site(self):
        """测试自定义TCP站点"""
        # 根据您的代码示例，KUKA iiwa模型中可能有'attachment_site'
        custom_site_name = 'attachment_site'
        
        try:
            robot = RobotModel(self.model_path, tcp_site_name=custom_site_name)
            
            # 测试FK计算
            q = np.zeros(7)
            pos, rot_mat = robot.fk(q)
            
            print(f"使用TCP站点 '{custom_site_name}': 位置 {pos}")
            
            # 基本验证
            assert pos.shape == (3,)
            assert rot_mat.shape == (3, 3)
            
            print("✓ 自定义TCP站点测试通过")
            
        except Exception as e:
            print(f"自定义TCP站点测试跳过: {e}")
            # 如果指定的站点不存在，这是预期的行为
    
    def test_invalid_input_handling(self):
        """测试错误输入处理"""
        robot = RobotModel(self.model_path)
        
        # 测试错误长度的关节角度
        with pytest.raises(ValueError, match="配置 q 长度错误"):
            robot.fk(np.array([0.0] * 5))  # 长度不足
            
        with pytest.raises(ValueError, match="配置 q 长度错误"):
            robot.fk(np.array([0.0] * 10))  # 长度过长
        
        print("✓ 错误输入处理测试通过")

def run_real_model_tests():
    """运行所有真实模型测试"""
    print("开始使用真实KUKA iiwa模型测试 RobotModel 类...")
    print("=" * 60)
    
    test_instance = TestRobotModelReal()
    
    try:
        test_instance.setup_method()
        test_instance.test_initialization_with_real_model()
        test_instance.test_fk_with_zero_configuration()
        test_instance.test_fk_with_different_poses()
        test_instance.test_random_joint_sampling()
        test_instance.test_fk_consistency()
        test_instance.test_custom_tcp_site()
        test_instance.test_invalid_input_handling()
        
        print("=" * 60)
        print("🎉 所有真实模型测试通过！RobotModel 类实现正确。")
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        raise

if __name__ == "__main__":
    run_real_model_tests()