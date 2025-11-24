<!--
 * @Author: Zhang-sklda 845603757@qq.com
 * @Date: 2025-11-23 21:59:57
 * @LastEditors: Zhang-sklda 845603757@qq.com
 * @LastEditTime: 2025-11-25 00:15:02
 * @FilePath: /reachability_project/README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
# reachability_project
基于kuka-iiwa-LBRMed系列机械臂的可达性和灵活性仿真研究

## 项目简介

本项目实现了一个基于 MuJoCo 的机械臂运动控制算法框架，专注于机械臂工作空间的可达性分析。通过离散化工作空间和末端执行器方向，构建高效的可达性地图，用于运动规划和任务可行性分析。

## 核心特性

### 🤖 机器人模型
- **多关节机械臂支持**：支持 KUKA iiwa 等 7 自由度机械臂
- **正向运动学计算**：准确的末端执行器位置和方向计算
- **随机采样**：在关节限制范围内生成随机配置

### 🗺️ 可达性地图
- **高效存储**：使用位数组技术，极大节省内存
- **多维度离散化**：位置、方向、滚转三自由度离散化
- **快速查询**：支持位姿级别的可达性查询
- **覆盖率统计**：实时计算工作空间覆盖率

### 🧪 测试框架
- **单元测试**：完整的模块测试覆盖
- **真实模型测试**：基于实际机器人模型的验证
- **性能测试**：内存使用和计算效率监控

## 项目结构
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


reachability_map.py
1.初始化测试 - 验证可达性地图参数正确设置
2.位置映射测试 - 测试三维坐标到体素索引的转换
3.方向映射测试 - 验证单位向量到方向索引的映射
4.滚转映射测试 - 测试滚转角度到索引的转换
5.体素操作测试 - 验证单个体素的设置和查询
6.位姿操作测试 - 测试完整的机器人位姿（位置+方向+滚转）设置和查询
7.覆盖率计算测试 - 验证可达性地图的覆盖率统计功能
8.边界情况测试 - 测试超出工作空间范围的位置处理
9.内存效率测试 - 验证位数组存储的内存效率



机器人模型准备：获取/生成 KUKA iiwa LBR MED 的 URDF 或 MJCF（包含连杆、关节、末端工具），并在 MuJoCo 中能计算 FK（位置/姿态）与做碰撞检测（MuJoCo 自带碰撞/contacts 可用）。
基础模块实现：
RobotModel：封装 MuJoCo 模型载入，FK（给定关节角返回 TCP pose）、设置关节、从模型读取关节上下限、关节自由度等。
ReachabilityMap：基于论文 III 节实现“分解索引”（ic, ia, ir）、内存布局（按位存储）与查询/写操作接口。
Sampler：实现 FK 随机采样模块（生成随机 joint vector → FK → map 写入），实现 IK 验证接口（可接 ikpy 或基于 Jacobian 的数值求解）用于 HYB 补全。
混合生成 HYB 流程（实现论文描述的切换阈值策略）并测量预测精度（随机抽样 vs map 查询 vs IK 求解）。
环境与碰撞：实现环境点云（或 CAD 网格离散为点集）导入 + KD-tree 加速 radius queries + primitive 近似碰撞（球/圆柱）或直接使用 MuJoCo 碰撞检测（速度可能略慢，但更精确）。
分析/可视化与优化：计算 Capability（reachability index）、可视化横截面热图、统计体积/高 dexterity 区域（论文 Table III/IV 风格），并进行参数 sweep（r, da, dr）与内存/精度折中（参考论文 Table II）。


