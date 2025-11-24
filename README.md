<!--
 * @Author: Zhang-sklda 845603757@qq.com
 * @Date: 2025-11-23 21:59:57
 * @LastEditors: Zhang-sklda 845603757@qq.com
 * @LastEditTime: 2025-11-24 23:05:06
 * @FilePath: /reachability_project/README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
# reachability_project
基于kuka-iiwa-LBRMed系列机械臂的可达性和灵活性仿真研究


reachability_project/
├── models/                      # 放 MJCF / URDF 等机器人模型
│   └── kuka_iiwa.xml
├── src/
│   ├── robot_model.py           # RobotModel 封装 MuJoCo
│   ├── reachability_map.py      # ReachabilityMap 数据结构与 IO
│   ├── sampler.py               # FK sampler / HYB 控制流程
│   ├── collision.py             # 点云 KD-tree + primitive collision checker
│   ├── analyze.py               # Capability 计算、体积统计、可视化
│   └── run_generation.py        # 脚本：配置 + 运行 map 生成
├── notebooks/                   # jupyter 分析可视化
└── README.md


robot_model.py
1.初始化测试 - 成功加载 KUKA iiwa 模型，正确识别7个关节
2.零位姿正向运动学 - 返回正确的位置 [0, 0, 1.306] 和单位旋转矩阵
3.不同位姿正向运动学 - 各种关节配置下FK计算正确
4.随机关节采样 - 成功生成随机关节角度
5.正向运动学一致性 - 多次计算相同配置结果一致
6.自定义TCP站点 - 正确使用 attachment_site
7.错误输入处理测试