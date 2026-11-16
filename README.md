# 机器人软硬件架构

**在线阅读：<https://wikieden.github.io/robot-architecture/>** · [讲义目录](https://wikieden.github.io/robot-architecture/lessons/index.html) · [全部动画](https://wikieden.github.io/robot-architecture/animations/index.html) · [复习卡片](https://wikieden.github.io/robot-architecture/review.html)

> 10 章 · 10 个动画（GIF + 可交互页面）· 40 张复习卡片（A01–A40，本书 10 章对应 A01–A30）· 覆盖 Lynch & Park《Modern Robotics》第 2、4–6 章与 §8.9、第 11 章里和整机架构有关的部分，其余（传感器、总线、ROS 2、自主栈、数据、安全）为自编

## 这本书讲什么

从"一台机器人 = 一组快慢嵌套的闭环"这张总图讲起，按"骨肉 → 神经 → 反射 → 思考 → 机器之外"的顺序，一层层拆开：关节角怎么变成手的位置、电机和减速器怎么选、传感器怎么标定和对时、几块电脑和几条总线怎么接、关节怎么"听话"、ROS 2 怎么把模块串起来、VLA 和双系统放在哪一层、仿真和数据怎么转成飞轮，最后讲安全分层，并拆五类整机。

- **适合谁**：学过一点 AI（比如 VLA、强化学习），想知道"模型下面那台机器到底长什么样"的人。需要的数学（三角函数、矩阵、雅可比、二阶系统、期望）在用到的那一章现场补。
- **读完能做什么**：拿到一台机器人（或一份方案），能画出它的分层闭环图，说出每层多快、跑在哪块硬件上、层间传什么；能用第 10 章的九步清单从零做一遍选型；接着去读 VLA 讲义 和 ROS 2 专题时知道每个模块坐在哪。
- **怎么读**：按章顺序读。每章先看讲解和动画，再自己做练习，最后用复习卡片隔几天考自己。

## 讲解的特点

- **一次一小步**：每个概念按"白话类比 → 公式（每个符号说清楚怎么读）→ 小数字亲手算一遍"讲：端水走路讲分层、自行车变速讲减速比、星光讲相机延迟、开门讲阻抗控制。
- **每章一个动画**：关键原理做成可交互页面，▶ 播放、→ 单步、↺ 从头开始，有滑块的可以拖动参数看结果怎么变（减速比、延迟、刚度、丢包率、随机化范围）；GIF 版直接嵌在正文里。
- **对照原文**：每章开头的"对应原文"标了本地文本的行号（Modern Robotics 的 `materials/text/`、ROS 2 官方文档、VLA 论文文本），原文没讲的部分写明去哪查。
- **保留真实的学习过程**：练习与作答记录会记下学习时的原话、对错和点评；答错、卡住的地方变成复习卡片。（目前 30 道练习还没作答。）
- **术语不设门槛**：缩写第一次出现就给英文全称并逐词解释（如 QDD = Quasi-Direct Drive：准 / 直驱），全书缩写见 [缩写与术语表](https://wikieden.github.io/robot-architecture/lessons/appendix-glossary.html)。
- **每章都有代码对照**：附录最后一节用 Python + NumPy 把本章的数算一遍（减速比、CAN 一圈多久、阻抗撞墙的力、QoS 队列、反应时间、停止距离……），配测试核对和讲义一致，顺带讲代码里才看得到的坑（角度要换弧度、位和字节差 8 倍、浮点误差让 `ceil` 多算一圈……）；只算讲过的例子，不提前给练习答案。每章还配了一个 Jupyter notebook（`solutions/chapter-code/notebooks/`），讲解和代码放在一起，一格一格边读边跑。
- **每章收尾有口诀**：几句话记住一章，全部口诀汇总在本页末尾。

## 示例动画

<table>
<tr>
<td width="33%" valign="top"><img src="https://wikieden.github.io/robot-architecture/animations/01-layered-loops.gif" width="100%" alt="第 1 章 · 分层闭环"><br><b>第 1 章 · 分层闭环</b><br>慢层给目标、快层追目标，拖滑块看策略频率变低会怎样<br><a href="https://wikieden.github.io/robot-architecture/lessons/01-layered-loops.html">讲义</a> · <a href="https://wikieden.github.io/robot-architecture/animations/01-layered-loops.html">▶ 交互</a></td>
<td width="33%" valign="top"><img src="https://wikieden.github.io/robot-architecture/animations/02-two-link-ik.gif" width="100%" alt="第 2 章 · 两连杆 IK"><br><b>第 2 章 · 两连杆 IK</b><br>手追目标，跑出工作空间时手臂伸直、det J 趋近 0<br><a href="https://wikieden.github.io/robot-architecture/lessons/02-kinematics.html">讲义</a> · <a href="https://wikieden.github.io/robot-architecture/animations/02-two-link-ik.html">▶ 交互</a></td>
<td width="33%" valign="top"><img src="https://wikieden.github.io/robot-architecture/animations/03-gear-ratio.gif" width="100%" alt="第 3 章 · 减速比"><br><b>第 3 章 · 减速比</b><br>力矩乘 N、速度除 N，反射惯量乘 N²<br><a href="https://wikieden.github.io/robot-architecture/lessons/03-actuators.html">讲义</a> · <a href="https://wikieden.github.io/robot-architecture/animations/03-gear-ratio.html">▶ 交互</a></td>
</tr>
<tr>
<td width="33%" valign="top"><img src="https://wikieden.github.io/robot-architecture/animations/05-bus-cycle.gif" width="100%" alt="第 5 章 · 总线周期"><br><b>第 5 章 · 总线周期</b><br>CAN 挨个点名，EtherCAT 一帧走全场<br><a href="https://wikieden.github.io/robot-architecture/lessons/05-electrical-compute.html">讲义</a> · <a href="https://wikieden.github.io/robot-architecture/animations/05-bus-cycle.html">▶ 交互</a></td>
<td width="33%" valign="top"><img src="https://wikieden.github.io/robot-architecture/animations/06-impedance.gif" width="100%" alt="第 6 章 · 阻抗控制"><br><b>第 6 章 · 阻抗控制</b><br>硬弹簧和软弹簧撞墙，稳态力 ≈ K × 陷入深度<br><a href="https://wikieden.github.io/robot-architecture/lessons/06-low-level-control.html">讲义</a> · <a href="https://wikieden.github.io/robot-architecture/animations/06-impedance.html">▶ 交互</a></td>
<td width="33%" valign="top"><img src="https://wikieden.github.io/robot-architecture/animations/10-safety-states.gif" width="100%" alt="第 10 章 · 安全降级"><br><b>第 10 章 · 安全降级</b><br>正常 → 降速 → SS1 → STO，一层层兜底<br><a href="https://wikieden.github.io/robot-architecture/lessons/10-safety-and-cases.html">讲义</a> · <a href="https://wikieden.github.io/robot-architecture/animations/10-safety-states.html">▶ 交互</a></td>
</tr>
</table>

全部动画见 [动画目录](https://wikieden.github.io/robot-architecture/animations/index.html)（另有运控、规划细化笔记的 4 个动画）。

## 目录

每章都配一个动画：**GIF** 在讲义正文里直接播放，**▶ 交互**是可以点按钮、拖滑块的页面（本地看需要起服务，见 [动画目录](https://wikieden.github.io/robot-architecture/animations/index.html)）。每章附录最后有代码对照，代码和测试在 `solutions/chapter-code/`。

| 章 | 主题 | 一句话精要 | 动画 | 卡片 | 状态 |
|---|---|---|---|---|---|
| [1](https://wikieden.github.io/robot-architecture/lessons/01-layered-loops.html) | 总图：一台机器人 = 一组快慢嵌套的闭环 | 大脑给目标、脊髓追目标；每层软件坐在一块硬件上 | [GIF](https://wikieden.github.io/robot-architecture/animations/01-layered-loops.gif) · [▶ 交互](https://wikieden.github.io/robot-architecture/animations/01-layered-loops.html) | A01–A03 | 已写，待讲 |
| [2](https://wikieden.github.io/robot-architecture/lessons/02-kinematics.html) | 本体与运动学：FK / IK / 雅可比 / 奇异 | 正解一路加角度，逆解要迭代；雅可比管速度也管力 | [GIF](https://wikieden.github.io/robot-architecture/animations/02-two-link-ik.gif) · [▶ 交互](https://wikieden.github.io/robot-architecture/animations/02-two-link-ik.html) | A04–A06 | 已写，待讲 |
| [3](https://wikieden.github.io/robot-architecture/lessons/03-actuators.html) | 执行器：电机 + 减速器，减速比的四个后果 | 力矩乘 N、速度除 N、惯量乘 N² | [GIF](https://wikieden.github.io/robot-architecture/animations/03-gear-ratio.gif) · [▶ 交互](https://wikieden.github.io/robot-architecture/animations/03-gear-ratio.html) | A07–A09 | 已写，待讲 |
| [4](https://wikieden.github.io/robot-architecture/lessons/04-sensors.html) | 传感器：本体 / 外部、标定、时间同步、融合 | 先标定、再对时；延迟误差 = v · Δt | [GIF](https://wikieden.github.io/robot-architecture/animations/04-time-sync.gif) · [▶ 交互](https://wikieden.github.io/robot-architecture/animations/04-time-sync.html) | A10–A12 | 已写，待讲 |
| [5](https://wikieden.github.io/robot-architecture/lessons/05-electrical-compute.html) | 电气与计算：主控 / 实时 / 驱动、EtherCAT vs CAN、电源、急停 | CAN 挨个点名，EtherCAT 一帧走全场；急停走硬线 | [GIF](https://wikieden.github.io/robot-architecture/animations/05-bus-cycle.gif) · [▶ 交互](https://wikieden.github.io/robot-architecture/animations/05-bus-cycle.html) | A13–A15 | 已写，待讲 |
| [6](https://wikieden.github.io/robot-architecture/lessons/06-low-level-control.html) | 底层控制：级联 PID、FOC、阻抗、WBC / MPC / RL | 内环快外环慢，电流即力矩；阻抗让手像弹簧 | [GIF](https://wikieden.github.io/robot-architecture/animations/06-impedance.gif) · [▶ 交互](https://wikieden.github.io/robot-architecture/animations/06-impedance.html) | A16–A18 | 已写，待讲 |
| [7](https://wikieden.github.io/robot-architecture/lessons/07-ros2-middleware.html) | 软件框架：ROS 2、QoS、tf2、ros2_control、实时 | 差网络里要新鲜不要完整；实时要自己隔出来 | [GIF](https://wikieden.github.io/robot-architecture/animations/07-ros2-qos.gif) · [▶ 交互](https://wikieden.github.io/robot-architecture/animations/07-ros2-qos.html) | A19–A21 | 已写，待讲 |
| [8](https://wikieden.github.io/robot-architecture/lessons/08-autonomy-stack.html) | 自主栈：模块化 vs 端到端 VLA vs 双系统 | 反应快慢看最快那条闭环 | [GIF](https://wikieden.github.io/robot-architecture/animations/08-latency-budget.gif) · [▶ 交互](https://wikieden.github.io/robot-architecture/animations/08-latency-budget.html) | A22–A24 | 已写，待讲 |
| [9](https://wikieden.github.io/robot-architecture/lessons/09-sim-data-deploy.html) | 仿真、数据与部署闭环：sim2real、遥操作、数据格式、OTA | 仿真不准就让它随机地不准；数据要记相机在哪、什么时候 | [GIF](https://wikieden.github.io/robot-architecture/animations/09-domain-randomization.gif) · [▶ 交互](https://wikieden.github.io/robot-architecture/animations/09-domain-randomization.html) | A25–A27 | 已写，待讲 |
| [10](https://wikieden.github.io/robot-architecture/lessons/10-safety-and-cases.html) | 安全、可靠性与五类整机案例 | 越往下越不靠软件；能减速就别断电 | [GIF](https://wikieden.github.io/robot-architecture/animations/10-safety-states.gif) · [▶ 交互](https://wikieden.github.io/robot-architecture/animations/10-safety-states.html) | A28–A30 | 已写，待讲 |
| [附录](https://wikieden.github.io/robot-architecture/lessons/appendix-glossary.html) | 缩写与术语表 | 全书缩写按领域分表 | — | — | 完成 |

---

本仓库由私有学习仓库里的脚本 `tools/publish-transformer/build.py` 自动生成，内容摘自讲义目录，不要直接修改这里的文件。
