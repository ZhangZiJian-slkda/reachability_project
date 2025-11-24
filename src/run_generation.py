"""
Description: Robotic Arm Motion Control Algorithm
Author: Zhang-sklda 845603757@qq.com
Date: 2025-11-24 23:21:22
Version: 1.0.0
LastEditors: Zhang-sklda 845603757@qq.com
LastEditTime: 2025-11-24 23:24:12
FilePath: /reachability_project/src/run_generation.py
Copyright (c) 2025 by Zhang-sklda, All Rights Reserved.
symbol_custom_string_obkoro1_tech: Tech: Motion Control | MuJoCo | ROS | Kinematics
"""
import os
from src.robot_model import RobotModel

model_path = "/home/zhang/reachability_project/models/iiwa14.xml"
def main():
    
    robot = RobotModel(model_path,tcp_site_name='attachment_site')
    