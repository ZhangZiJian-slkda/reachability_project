'''
Author: Zhang-sklda 845603757@qq.com
Date: 2025-11-24 00:06:37
LastEditors: Zhang-sklda 845603757@qq.com
LastEditTime: 2025-11-24 00:08:51
FilePath: /reachability_project/src/__init__.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
"""
Reachability Project - KUKA iiwa Reachability and Dexterity Analysis

This package provides tools for analyzing the reachability and dexterity
of KUKA iiwa robotic arms using MuJoCo simulation.
"""

__version__ = "0.1.0"
__author__ = "Your Name"
__email__ = "your.email@example.com"

# Import main classes for easier access
from .robot_model import RobotModel

# Define what gets imported with "from src import *"
__all__ = ["RobotModel"]