# Step 4：无接触重定位的算法、代价与替换边界

## 1. 目标、范围与依赖

本阶段负责把真实 Pusher 从当前物理位置移到指定 ContactTarget(face,s)，分成退离、绕行、接近三个动作。它不选择推动接触、不优化物块目标轨迹、不实现 B/C，不使用准静态预测模型或 OSQP 求导。它只消费物理 geometry、WorldObservation 和 ContactTarget，输出世界速度，接入既有 supervisor。

当前物理场景是单个近似平面的矩形 Slider、圆柱 Pusher、无限桌面，没有额外障碍物。支持任意几何有效的初始中心位置和允许的面内 s；有限时间/重规划预算可能导致明确失败，不承诺无限远起点也能在30秒内到达。

## 2. 配置空间与碰撞几何

在当前物体坐标系，矩形半尺寸 h=(L/2,W/2)，Pusher 半径 rho。真实圆柱中心障碍是矩形与半径 rho 圆盘的 Minkowski 和（圆角矩形）。真实平面 clearance 计算为

\[
q=|p_c^{body}|-h,\quad
 d=\|\max(q,0)\|_2+\min(\max(q_x,q_y),0)-\rho.
\]

d>0 表示分离，d<0 表示几何重叠。这个距离用于执行诊断和退离安全检查，与所选面的 signed gap 不同。

默认路线规划使用更保守的轴对齐包络

\[
B_\delta=\{p:|p_i|<h_i+\rho+\delta\}.
\]

它包含实际圆角中心障碍及安全余量，代价是角点附近较保守。完整线段通过 analytic slab intersection 检查，不是只检查 waypoint 或用稀疏采样判碰撞。边界跟随允许，实际 routing vertex 再增加1mm padding，避免依赖浮点 epsilon。

所有规划计算先变换到物体 frame，再变换回世界坐标；旋转和平移只改变坐标，不改变路径长度和默认代价。

## 3. 路径生成

### 退离

如果起点位于保守 routing envelope 内，但没有超过允许的真实穿透，先沿最近的外侧坐标轴退出。对圆角附近的合法起点也适用：增加绝对坐标的一个分量，使到真实矩形的距离不减。起点深度互穿>1mm直接拒绝，不靠 reset 或大位移恢复。

起点已在包络外时不强制额外绕远；仍经过卸载/稳定检查。retreat_end_index 把退离前缀与真正无接触路线分开，不能把触碰起点当作普通 free-space 路径起点。

### 绕行

图节点：退离终点、目标预接触位置、4个 padding 后的矩形顶点。每对可见节点建立无碰撞线段。枚举起点到终点的不重复节点路径；只有4个角点，最多65条简单候选，不需要随机采样或大型路径规划依赖。碰撞过滤后评估代价，按 total、长度、坐标字典序确定性打破平局。

这保证的是在该有限可见图候选集合中找到最低评分路径；不声称对圆角精确障碍、动态场景或任意新 cost 都是连续空间全局最优。新 cost 不能让穿过障碍的路线重新变为可行。

### 预接触与接近

目标预接触中心为

\[
c_{pre}=p+R(\theta)[r(s)-(\rho+g_{pre})n],
\]

默认 gap=8mm，位于5mm膨胀余量+1mm routing padding 之外。无接触路径到此结束，length_m 不包含最后接近。

接近时跟踪最新物块位姿，沿 inward normal 低速运动，加入小的0.1mm preload 目标以建立真实载荷；接触前允许最多2mm/s的横向位置校正。有载荷时不通过主动切向拖动来对齐错误的接触位置，偏差过大直接失败。超时、穿透过大或面几何无效均输出零速度。

## 4. 默认代价：名义完成时间 + 转弯时间

历史 Jtravel 定义保持不变：

\[
J_{travel}=L_{retreat}+L_{transit}\quad[\mathrm m].
\]

新增独立选路评价为

\[
J_{route}=\frac{L_{retreat}}{v_r}+\frac{L_{transit}}{v_t}
+\frac{g_{pre}}{v_a}+\lambda_\phi\sum_i\phi_i+t_{settle}\quad[\mathrm s].
\]

phi_i 是连续非零线段方向之间的夹角，包含末段到 inward-normal approach 的转向；lambda_phi=0.05 s/rad。默认 vr=0.015、vt=0.03、va=0.003 m/s，t_settle=0.1s。

选择时间单位便于解释长路和转弯的开销，避免将米、秒、角度未经说明直接相加。不同候选都显式保存 length、retreat/transit/approach 距离、名义运动时间、转角、转弯项和固定项。转弯系数及稳定时间是首版工程启发式，不是拟合的时间保证；反馈减速、额外 waypoint 停顿、接触建立、重规划和 supervisor 确认会让真实时间不同。

这不包含 Jpush、目标位姿收益或 Jswitch。未来 B/C 可以读取原 length_m，或显式选择新时间代价作为其 travel 项；必须重新记录权重与单位，不能默默把旧的米数替换成秒数。

`RepositionCostEvaluator.evaluate(plan,observation)->MotionCost(total,unit,components)` 可替换为长度、能量近似、风险评分等；单位必须在同一轮候选中一致，结果必须有限。代价只决定有效路线之间的排序，不拥有仿真或碰撞判断。

## 5. 执行与安全状态

默认 FeedbackRepositionExecutor 使用实际世界中心误差的比例速度反馈，范数受各阶段速度上限限制，始终通过 env.step 执行。靠近 waypoint 时速度随距离减小，不按参考路径直接写物体或 Pusher 位姿。

- RETRACT：执行退离前缀，检查实际 clearance 不减少；退离结束后要求无载荷，Slider 线速度/角速度连续稳定100ms。超时则失败。
- TRANSIT：每步以当前真实物块位姿重新检查实际中心到下一控制步预计中心的完整线段；1.5mm执行 guard 与5mm规划余量、0.4mm waypoint容差分开。任何意外载荷立即失败。不能因为 cost 更低跳过检查。
- APPROACH：低速法向接近，检查实际 s、gap、载荷；连续40ms满足物理接触条件后返回 READY。
- 既有 supervisor 随后独立确认60ms，再激活 MPC、清空旧 warm start。READY 不直接等于完成了 task 层确认。

物块位姿相对 anchor 漂移超过1mm或0.02rad时，先返回零速度，再从新观测重规划，最多3次；执行器从不“修正”Slider位姿。路径开始必须对应实际中心，终点必须对应预接触目标；第三方 planner 返回穿越障碍的路线会在执行前被拒绝。

本执行器不依赖 MuJoCo handle，原 TaskSupervisor 也不持有 env。原本 `--with-reposition` 未启用的 Step 3 调用继续返回明确的请求。自定义 motion port 的 start/update ValueError/RuntimeError 也会变成 supervisor 的明确失败与零指令。

## 6. 可替换模块与兼容性

```python
cost = MyTravelCost()               # RepositionCostEvaluator
planner = MyPlanner(geometry, cost) # RepositionPlanner
executor = FeedbackRepositionExecutor(
    geometry, planner, monitor, MotionConfig(),
    collision_checker=MyCollisionChecker(geometry),
)
supervisor = TaskSupervisor(selector, push_controller, monitor,
                            reposition_executor=executor)
```

| 要改的设计 | 替换位置 | 不需改动 |
|---|---|---|
| 椭球模型换成新预测模型 | model + push adapter/必要的内部MPC | motion planner/cost/executor/世界观测接口 |
| OSQP换其他算法或状态维度变化 | push controller adapter | selector、Step 4几何与执行 |
| 可见图换A*/RRT/其他路线算法 | RepositionPlanner | supervisor、MPC、默认路径跟踪器（输出契约一致时） |
| time cost换长度/风险/能量近似 | RepositionCostEvaluator | 碰撞过滤、执行器、MPC |
| 速度反馈换轨迹跟踪控制 | RepositionExecutor | 选点、MPC及任务目标接口 |
| 需要不同的碰撞/保守几何 | CollisionChecker及planner相应几何 | supervisor与预测模型 |
| A换B/C接触选择 | ContactSelector | 同一Step 4和底层MPC |

当前 geometry/ContactTarget 仍明确描述单个矩形与face+s。新物体形状或完整3D环境需要几何/目标描述适配或接口版本升级，不能声称任意语义变化都免费兼容。算法替换与数学模型更换通过现有模块边界局部处理，不必推倒整个工程。

Contract v1保持，增量revision=1：RepositionPlan新字段具有默认值，PlanningResult添加可选cost，不破坏原构造/Protocol调用。历史字段step4_implemented作为step4_enabled的兼容别名；v0.4.0是否包含算法由包版本判断。

## 7. 参数与修改指引

| 配置 | 关键默认值 | 单位 |
|---|---|---|
| PlannerConfig | clearance=.005, vertex_padding=.001, precontact_gap=.008 | m |
| 同上 | max_recovery_penetration=.001 | m |
| MotionConfig | dt=.02, retreat/transit/approach=.015/.03/.003 | s；m/s |
| 同上 | position_gain=6, waypoint_tolerance=.0004 | 1/s；m |
| 同上 | execution_clearance=.0015 | m |
| 同上 | settle_seconds=.1, settle_timeout=2 | s |
| 同上 | slider_linear/angular_tolerance=.0015/.02 | m/s；rad/s |
| 同上 | pose_replan_distance/angle=.001/.02 | m；rad |
| 同上 | max_replans=3, max_seconds=30, approach_timeout=10 | 次；s；s |
| 同上 | contact_hold_seconds=.04, preload_depth=.0001 | s；m |
| 同上 | max_approach_penetration=.0008, max_recovery_penetration=.001 | m |
| TravelCostConfig | speeds=.015/.03/.003, turn_seconds_per_radian=.05, fixed_settle_seconds=.1 | m/s；s/rad；s |

`configs/step4_example.json` 可直接修改。rule_demo 的 JSON支持 planner/motion/travel_cost，环境、模型、MPC仍独立。未显式设置cost时，demo以执行速度初始化名义时间cost；显式cost配置可独立变化，所有展开值保存到日志。MPC、motion dt必须匹配env.dt_control；所有世界速度上限共同检查。

调安全余量时需满足 precontact_gap>clearance+vertex_padding，plan.clearance>execution_clearance+waypoint_tolerance。缩小margin不能代替修复错误模型。改变运动速度应检查 waypoint跟踪、离散步长、实际载荷与转弯耗时。时间预算和精度条件独立配置，不应靠不断放大阈值掩盖换面失败。

## 8. 限制、验证和日志

世界位置真值观测、单矩形、无其他障碍是当前假设；忽略roll/pitch/z变化，不支持大幅倾覆或感知误差。几何与每步检查是在当前观测下的安全措施，不是对任意外部扰动或未知动态障碍的形式保证。预算耗尽/未知载荷/不合法初态会明确失败。

退离起点和接近终点允许接触，TRANSIT必须无载荷且保留正间隙。接近会产生微小真实物块位移，报告与测试分开统计，不把整个周期声称为完全不接触。

验证包含线段穿越、随机合法起点、完整路径稠密独立距离检查、旋转等变性、cost分项和替换、错误第三方计划/载荷/位姿漂移/重规划上限/时间异常，以及MuJoCo中全部16个有向面组合、自由起点和运行中换面任务。当前实测值、耗时与未解决问题见 [step4_report.md](step4_report.md)。
