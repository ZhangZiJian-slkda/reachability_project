<!--
 * @Author: Zhang-sklda 845603757@qq.com
 * @Date: 2025-11-23 21:59:57
 * @LastEditors: Zhang-sklda 845603757@qq.com
 * @LastEditTime: 2025-11-24 00:08:10
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



Day 0（现在）：我已经读完论文（引用见上）。接下来按下面步骤你可以马上开始并在本周内看到成果。
Day 1：在你的 Linux 机上按我给的依赖安装（或把你现有环境告诉我，我将给精确命令）。准备 KUKA iiwa MJCF/URDF 放到 models/。运行上面 run_generation.py（samples=20000）观察生成速度与 map 大小。
Day 2：视运行结果加入：1) map 存盘/加载功能；2) 可视化一个横截面（绘制 capability heatmap）；3) 用少量随机点做 IK 验证以估计 FK-only map 的 FP/FN。
Day 3–4：实现 HYB：在 FK 阶段统计新 bin 发现比率，低于阈值时列出未覆盖的 bins 并用 IK solver 验证（我会给 IK 数值实现或指导使用 ikpy / custom Newton method）。
Day 5–7：加入环境碰撞检查（点云 KD-tree + primitive），并做参数 sweep（r, da, dr）记录 memory/accuracy trade-off（论文中给了类似试验，可复现）。生成 capability 可视化与体积统计（参考论文表格格式）。