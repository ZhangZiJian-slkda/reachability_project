'''
Author: Zhang-sklda 845603757@qq.com
Date: 2025-11-23 23:32:48
LastEditors: Zhang-sklda 845603757@qq.com
LastEditTime: 2025-11-24 00:10:50
FilePath: /reachability_project/test/run_real_test.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
#!/usr/bin/env python3
"""
使用真实KUKA iiwa模型测试RobotModel
"""

import sys
import os

# 添加当前目录到Python路径
# sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from test.test_robot_model_real import run_real_model_tests

if __name__ == "__main__":
    run_real_model_tests()