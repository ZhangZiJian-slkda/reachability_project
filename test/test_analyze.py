"""
Description: Robotic Arm Motion Control Algorithm
Author: Zhang-sklda 845603757@qq.com
Date: 2025-11-26 23:17:31
Version: 1.0.0
LastEditors: Zhang-sklda 845603757@qq.com
LastEditTime: 2025-11-26 23:25:03
FilePath: /reachability_project/test/test_analyze.py
Copyright (c) 2025 by Zhang-sklda, All Rights Reserved.
symbol_custom_string_obkoro1_tech: Tech: Motion Control | MuJoCo | ROS | Kinematics
"""

import numpy as np
import os
import sys
import tempfile

# 添加路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from reachability_map import ReachabilityMap
from analyze import compute_capability, visualize_3d_voxels, save_capability_data, load_capability_data


class TestAnalyze:
    """测试分析功能"""
    
    def setup_method(self):
        """设置测试环境"""
        # 创建测试用的可达性地图
        self.rmap = ReachabilityMap(
            origin=[-0.5, -0.5, 0.0],
            dims=[5, 5, 5],      # 较小的地图用于测试
            voxel_r=0.2,
            da=4,                # 简化方向
            dr=2                 # 简化滚转
        )
        
        # 手动设置一些可达体素用于测试
        self._setup_test_voxels()
    
    def _setup_test_voxels(self):
        """设置测试体素"""
        # 设置一些体素为可达
        test_voxels = [
            (1, 1, 1, 0, 0),  # 中心体素，方向0，滚转0
            (1, 1, 1, 1, 0),  # 中心体素，方向1，滚转0
            (1, 1, 1, 2, 1),  # 中心体素，方向2，滚转1
            (2, 2, 2, 0, 0),  # 角落体素
            (0, 0, 0, 3, 1),  # 另一个角落
        ]
        
        for ix, iy, iz, ia, ir in test_voxels:
            ic = ix + iy * self.rmap.Dx + iz * (self.rmap.Dx * self.rmap.Dy)
            self.rmap.set_bin(ic, ia, ir)
    
    def test_compute_capability(self):
        """测试灵活性计算"""
        print("=== 测试灵活性计算 ===")
        
        # 计算灵活性
        cap = compute_capability(self.rmap)
        
        # 验证输出
        assert isinstance(cap, np.ndarray)
        assert cap.shape == (self.rmap.voxel_count,)
        assert cap.dtype == np.float64
        
        # 验证数值范围
        assert np.all(cap >= 0) and np.all(cap <= 1), "灵活性应该在0-1范围内"
        
        # 检查已知体素的灵活性
        center_voxel_idx = 1 + 1 * self.rmap.Dx + 1 * (self.rmap.Dx * self.rmap.Dy)
        center_cap = cap[center_voxel_idx]
        
        # 中心体素有3/8=0.375的灵活性
        expected_center_cap = 3 / (self.rmap.da * self.rmap.dr)  # 3/8 = 0.375
        assert abs(center_cap - expected_center_cap) < 1e-10, f"中心体素灵活性计算错误: {center_cap} != {expected_center_cap}"
        
        print(f"中心体素灵活性: {center_cap:.3f} (期望: {expected_center_cap:.3f})")
        
        # 统计不同灵活性级别的体素数量
        zero_cap = np.sum(cap == 0)
        low_cap = np.sum((cap > 0) & (cap <= 0.5))
        high_cap = np.sum(cap > 0.5)
        
        print(f"零灵活性体素: {zero_cap}")
        print(f"低灵活性体素 (0-0.5): {low_cap}")
        print(f"高灵活性体素 (>0.5): {high_cap}")
        
        print("✓ 灵活性计算测试通过")
    
    def test_capability_statistics(self):
        """测试灵活性统计"""
        print("\n=== 测试灵活性统计 ===")
        
        cap = compute_capability(self.rmap)
        
        # 计算统计信息
        mean_cap = np.mean(cap)
        max_cap = np.max(cap)
        min_cap = np.min(cap)
        non_zero_count = np.sum(cap > 0)
        
        print(f"平均灵活性: {mean_cap:.3f}")
        print(f"最大灵活性: {max_cap:.3f}")
        print(f"最小灵活性: {min_cap:.3f}")
        print(f"非零灵活性体素数量: {non_zero_count}")
        
        # 验证统计信息
        assert mean_cap >= 0 and mean_cap <= 1
        assert max_cap >= 0 and max_cap <= 1
        assert min_cap >= 0 and min_cap <= 1
        assert non_zero_count <= self.rmap.voxel_count
        
        print("✓ 灵活性统计测试通过")
    
    def test_visualization_data_preparation(self):
        """测试可视化数据准备（不实际打开窗口）"""
        print("\n=== 测试可视化数据准备 ===")
        
        cap = compute_capability(self.rmap)
        
        # 测试最小灵活性阈值
        points_high_threshold = []
        points_low_threshold = []
        
        # 手动实现可视化逻辑但不显示
        voxel_idx = 0
        for ix in range(self.rmap.Dx):
            for iy in range(self.rmap.Dy):
                for iz in range(self.rmap.Dz):
                    center = (self.rmap.origin + 
                             np.array([ix, iy, iz]) * self.rmap.r + 
                             np.array([self.rmap.r/2, self.rmap.r/2, self.rmap.r/2]))
                    
                    if cap[voxel_idx] >= 0.5:  # 高阈值
                        points_high_threshold.append(center)
                    if cap[voxel_idx] >= 0.01:  # 低阈值
                        points_low_threshold.append(center)
                    
                    voxel_idx += 1
        
        print(f"高阈值(0.5)下的点数: {len(points_high_threshold)}")
        print(f"低阈值(0.01)下的点数: {len(points_low_threshold)}")
        
        # 验证阈值过滤
        assert len(points_high_threshold) <= len(points_low_threshold)
        assert len(points_low_threshold) > 0, "至少应该有一些点满足低阈值"
        
        print("✓ 可视化数据准备测试通过")
    
    def test_file_io(self):
        """测试文件读写功能"""
        print("\n=== 测试文件IO功能 ===")
        
        # 创建临时文件
        with tempfile.NamedTemporaryFile(suffix='.pkl', delete=False) as tmp_file:
            temp_path = tmp_file.name
        
        try:
            # 计算灵活性
            cap = compute_capability(self.rmap)
            
            # 保存数据
            save_capability_data(self.rmap, cap, temp_path)
            
            # 验证文件存在
            assert os.path.exists(temp_path), "文件应该被创建"
            
            # 加载数据
            loaded_data = load_capability_data(temp_path)
            
            # 验证加载的数据
            assert 'capability' in loaded_data
            assert 'origin' in loaded_data
            assert 'dims' in loaded_data
            assert 'resolution' in loaded_data
            
            # 验证数据一致性
            np.testing.assert_array_equal(cap, loaded_data['capability'])
            np.testing.assert_array_equal(self.rmap.origin, loaded_data['origin'])
            assert self.rmap.r == loaded_data['resolution']
            
            print("✓ 文件IO测试通过")
            
        finally:
            # 清理临时文件
            if os.path.exists(temp_path):
                os.unlink(temp_path)
    
    def test_edge_cases(self):
        """测试边界情况"""
        print("\n=== 测试边界情况 ===")
        
        # 测试空地图
        empty_map = ReachabilityMap(
            origin=[-0.5, -0.5, 0.0],
            dims=[2, 2, 2],
            voxel_r=0.2,
            da=4,
            dr=2
        )
        
        empty_cap = compute_capability(empty_map)
        assert np.all(empty_cap == 0), "空地图的灵活性应该全为0"
        
        # 测试全满地图
        full_map = ReachabilityMap(
            origin=[-0.5, -0.5, 0.0],
            dims=[2, 2, 2],
            voxel_r=0.2,
            da=2,  # 较小的方向数以加快测试
            dr=2
        )
        
        # 设置所有体素为可达
        for ic in range(full_map.voxel_count):
            for ia in range(full_map.da):
                for ir in range(full_map.dr):
                    full_map.set_bin(ic, ia, ir)
        
        full_cap = compute_capability(full_map)
        assert np.all(full_cap == 1.0), "全满地图的灵活性应该全为1"
        
        print("✓ 边界情况测试通过")


def run_analyze_tests():
    """运行所有分析测试"""
    print("开始测试 Analyze 功能...")
    print("=" * 60)
    
    test_instance = TestAnalyze()
    
    try:
        test_instance.setup_method()
        test_instance.test_compute_capability()
        test_instance.test_capability_statistics()
        test_instance.test_visualization_data_preparation()
        test_instance.test_file_io()
        test_instance.test_edge_cases()
        
        print("=" * 60)
        print("🎉 所有分析测试通过！Analyze 模块实现正确。")
        
        # 询问是否运行实际可视化
        response = input("\n是否运行实际的可视化测试？(y/n): ")
        if response.lower() == 'y':
            print("\n运行可视化测试...")
            cap = compute_capability(test_instance.rmap)
            
            # 首先尝试Open3D可视化
            try:
                print("尝试Open3D可视化...")
                visualize_3d_voxels(test_instance.rmap, cap, min_capability=0.01)
            except Exception as e:
                print(f"Open3D可视化失败: {e}")
                print("尝试Matplotlib可视化...")
                
                # 回退到Matplotlib
                try:
                    from analyze import visualize_2d_slices
                    visualize_2d_slices(test_instance.rmap, cap)
                except Exception as e2:
                    print(f"所有可视化方法都失败了: {e2}")
                    print("请检查您的显示环境或安装必要的库")
            
            print("可视化测试完成")
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        raise
    finally:
        # 清理资源
        pass

if __name__ == "__main__":
    run_analyze_tests()