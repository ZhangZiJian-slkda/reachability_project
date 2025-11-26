"""
Description: Robotic Arm Motion Control Algorithm
Author: Zhang-sklda 845603757@qq.com
Date: 2025-11-26 22:52:59
Version: 1.0.1  # 更新版本
LastEditors: Zhang-sklda 845603757@qq.com
LastEditTime: 2025-11-26 23:24:25
FilePath: /reachability_project/src/analyze.py
Copyright (c) 2025 by Zhang-sklda, All Rights Reserved.
symbol_custom_string_obkoro1_tech: Tech: Motion Control | MuJoCo | ROS | Kinematics
"""
import numpy as np
import open3d as o3d
import pickle
from reachability_map import ReachabilityMap

def compute_capability(rmap: ReachabilityMap):
    """
    计算每个体素的灵活性指数
    
    Args:
        rmap: ReachabilityMap实例
        
    Returns:
        np.ndarray: 灵活性指数数组，形状为 (voxel_count,)
    """
    cap = [] 
    for ic in range(rmap.voxel_count):
        # 提取体素对应的位块
        start_idx = ic * rmap.bits_per_voxel
        end_idx = (ic + 1) * rmap.bits_per_voxel
        block = rmap.bits[start_idx:end_idx]
        
        # 计算可达方向比例
        reachable_count = block.count(True)
        capability = reachable_count / rmap.bits_per_voxel
        cap.append(capability)
        
    return np.array(cap)


def visualize_3d_voxels(rmap, cap, min_capability=0.01):
    """
    使用Open3D可视化可达性地图
    
    Args:
        rmap: ReachabilityMap实例
        cap: 灵活性指数数组
        min_capability: 最小灵活性阈值，低于此值的不显示
    """
    points = []
    colors = []
    
    # 遍历所有体素
    voxel_idx = 0
    for ix in range(rmap.Dx):
        for iy in range(rmap.Dy):
            for iz in range(rmap.Dz):
                # 计算体素中心坐标
                center = (rmap.origin + 
                         np.array([ix, iy, iz]) * rmap.r + 
                         np.array([rmap.r/2, rmap.r/2, rmap.r/2]))
                
                # 只显示灵活性大于阈值的体素
                if cap[voxel_idx] >= min_capability:
                    points.append(center)
                    
                    # 根据灵活性设置颜色 (红色高，蓝色低)
                    capability = cap[voxel_idx]
                    # 使用热力图颜色映射: 蓝色(低) -> 绿色(中) -> 红色(高)
                    if capability < 0.5:
                        # 蓝色到绿色
                        r = 0
                        g = 2 * capability
                        b = 1 - 2 * capability
                    else:
                        # 绿色到红色
                        r = 2 * (capability - 0.5)
                        g = 1 - 2 * (capability - 0.5)
                        b = 0
                    
                    colors.append([r, g, b])
                
                voxel_idx += 1
    
    if not points:
        print("没有找到满足条件的体素进行可视化")
        return
    
    # 创建点云
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.array(points))
    cloud.colors = o3d.utility.Vector3dVector(np.array(colors))
    
    try:
        # 尝试使用简单的可视化方法
        print("正在启动Open3D可视化窗口...")
        o3d.visualization.draw_geometries([cloud], 
                                         window_name="Reachability Map",
                                         width=1200,
                                         height=800,
                                         left=50,
                                         top=50)
        print("可视化窗口已关闭")
        
    except Exception as e:
        print(f"Open3D可视化失败: {e}")
        print("尝试替代方案...")
        
        # 替代方案1：保存为文件
        try:
            import datetime
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"reachability_map_{timestamp}.ply"
            o3d.io.write_point_cloud(filename, cloud)
            print(f"点云已保存到文件: {filename}")
            print("您可以使用MeshLab或其他3D查看器打开此文件")
        except Exception as save_error:
            print(f"保存点云文件失败: {save_error}")
            
        # 替代方案2：文本输出
        print("\n可达性地图文本摘要:")
        print(f"总体素数: {rmap.voxel_count}")
        print(f"显示体素数: {len(points)}")
        print(f"平均灵活性: {np.mean(cap):.3f}")
        print(f"最大灵活性: {np.max(cap):.3f}")
        print(f"灵活性分布:")
        for threshold in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
            count = np.sum((cap >= threshold) & (cap < threshold + 0.1))
            if count > 0:
                print(f"  {threshold:.1f}-{threshold+0.1:.1f}: {count}个体素")

def save_capability_data(rmap, cap, filepath):
    """保存灵活性数据到文件"""
    data = {
        'capability': cap,
        'origin': rmap.origin,
        'dims': [rmap.Dx, rmap.Dy, rmap.Dz],
        'resolution': rmap.r,
        'da': rmap.da,
        'dr': rmap.dr
    }
    
    with open(filepath, 'wb') as f:
        pickle.dump(data, f)
    
    print(f"灵活性数据已保存到: {filepath}")

def load_capability_data(filepath):
    """从文件加载灵活性数据"""
    with open(filepath, 'rb') as f:
        data = pickle.load(f)
    
    print(f"灵活性数据已从 {filepath} 加载")
    return data