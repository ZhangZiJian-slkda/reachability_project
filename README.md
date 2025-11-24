<!--
 * @Author: Zhang-sklda 845603757@qq.com
 * @Date: 2025-11-23 21:59:57
 * @LastEditors: Zhang-sklda 845603757@qq.com
 * @LastEditTime: 2025-11-24 23:27:36
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





机器人模型准备：获取/生成 KUKA iiwa LBR MED 的 URDF 或 MJCF（包含连杆、关节、末端工具），并在 MuJoCo 中能计算 FK（位置/姿态）与做碰撞检测（MuJoCo 自带碰撞/contacts 可用）。
基础模块实现：
RobotModel：封装 MuJoCo 模型载入，FK（给定关节角返回 TCP pose）、设置关节、从模型读取关节上下限、关节自由度等。
ReachabilityMap：基于论文 III 节实现“分解索引”（ic, ia, ir）、内存布局（按位存储）与查询/写操作接口。
Sampler：实现 FK 随机采样模块（生成随机 joint vector → FK → map 写入），实现 IK 验证接口（可接 ikpy 或基于 Jacobian 的数值求解）用于 HYB 补全。
混合生成 HYB 流程（实现论文描述的切换阈值策略）并测量预测精度（随机抽样 vs map 查询 vs IK 求解）。
环境与碰撞：实现环境点云（或 CAD 网格离散为点集）导入 + KD-tree 加速 radius queries + primitive 近似碰撞（球/圆柱）或直接使用 MuJoCo 碰撞检测（速度可能略慢，但更精确）。
分析/可视化与优化：计算 Capability（reachability index）、可视化横截面热图、统计体积/高 dexterity 区域（论文 Table III/IV 风格），并进行参数 sweep（r, da, dr）与内存/精度折中（参考论文 Table II）。