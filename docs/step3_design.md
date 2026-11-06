# Step 3: Rule-based Contact Selection and Stagnation Detection

本模块是 baseline Controller A。只使用目标误差、物理几何、当前接触与失败历史，不进行 MPC 候选 rollout、不使用 Jpush/Jtravel 搜索。完整系统接口和 Step 4 port 见 [component_contracts.md](component_contracts.md)。

## 选面与选点规则

令世界位置误差为 e_p=p_goal-p，物体坐标误差 e_b=R(theta)^T e_p。位置误差大于 translation_deadband 时，每个面的 normal alignment score 为

\[
a_f=n_f^T e_b/\|e_b\|.
\]

按 a_f 降序选择；明确的平分顺序为 LEFT、BOTTOM、RIGHT、TOP。当前面的 score 距最大值不超过 face_hysteresis 时，优先保留当前面。位置误差在 deadband 内时 scores=0，优先当前面，首次则按固定顺序。规则只定义局部启发式，不保证任务可达或全局最优。

采用所有面一致的 t=Jn、r=b+s t。矩形面中心 b 与 inward normal n 反向平行，因此纯法向推力矩为

\[
\tau=(b+s t)\times(f_n n)=-s f_n.
\]

yaw 误差 e_theta=atan2(sin(theta_goal-theta),cos(...))。选点为

\[
s^*=\begin{cases}0,&|e_\theta|\le\delta_\theta\\
\operatorname{clip}(-k_\theta e_\theta,-\eta s_{max},\eta s_{max}),&\text{otherwise}.
\end{cases}
\]

所以正 yaw 请求选择负 s，负 yaw 请求选择正 s；所有面符号相同。s_max 是扣除物理 corner_margin 后的可用半面长。这里的 normal-only 力矩解释用于规则方向，不替代 Local MPC 的完整 force cone。偏心位置幅度是工程启发式，没有使用模型预测误差拟合。

同一任务中，停滞、失去接触、越界或求解失败的目标加入 blocked 列表。同面且 s 相差在 blocked_s_tolerance 内视为重复失败；按同一 score/tie 规则尝试下一个面。改变当前误差后得到显著不同 s 仍是新目标，可在同面重定位。所有规则选项被阻止时返回明确失败，supervisor 另外限制总选择次数，避免无限重试。

## 停滞定义

使用无量纲尺度归一化误差

\[
V=\sqrt{(\|e_p\|/L_p)^2+(e_\theta/L_\theta)^2}.
\]

在固定仿真时间窗内，比较最早/最新最多3个样本的中位数，下降量为 DeltaV。下降不足

\[
\Delta V\le\max(\epsilon_{abs},\epsilon_{rel} V_{start})
\]

视为停滞。必须达到最少样本数和完整时间窗；时间不要求正好整除 dt，保留一个窗口前驱样本。只在实际 PUSH 阶段更新；暂停/换接触/换任务会清空窗口。尺度、阈值与成功条件独立，不把米和弧度直接相加。

当前设计把明显变差也判为缺乏有效进展。停止/选点只在最短保持时间之后触发；安全异常立即处理。更换停滞判据只影响 StagnationDetector / ProgressConfig 以及测试，不必改 QP。

## 物理接触与安全边界

monitor 从真实 pusher center 投影估计 measured_s，实际 contact samples 还需位于所选面的 supporting line 附近。载荷足够、s 匹配、gap 在容差内且位置未越界，连续满足 contact_confirm_seconds 后才调用 push.activate。desired_s 不会覆盖 measured_s。接触点误差按 measured_s 比较，累计 desired/measured 差异由漂移阈值检查。

实际载荷短暂为零仍完整记录；若无载荷且 gap>0.5 mm，则先输出零指令，持续60 ms后触发 contact_lost。微间隙不等于宏观脱离，也不证明理想 sticking；其间仍可能存在模型误差。signed gap 明显为负可能是物体另一侧/错误面，独立几何 guard 会拒绝。切向材料速度来自测得 pusher velocity 和 slider twist，超过2 mm/s的 slip 只标记记录，不声称新增 sliding mode。

QP infeasible、数值失败、越界、严重 s 漂移分别记录原因并重新选点。重新选点不调用 env.reset；需要移动时请求 Step 4，当前版本安全停止等待。更换同一面的接触位置也要重新确认并清空 MPC warm start。

## 默认参数（集中配置）

| 配置 | 参数 | 默认 | 单位/意义 |
|---|---|---|---|
| RuleSelectionConfig | rotation_gain | .16 | m/rad |
| 同上 | rotation_deadband | .02 | rad |
| 同上 | max_offset_fraction | .6 | usable half-face fraction |
| 同上 | translation_deadband | .005 | m |
| 同上 | face_hysteresis | .1 | normal alignment score |
| 同上 | blocked_s_tolerance | .003 | m |
| RectangleGeometry | corner_margin | .002 | m |
| ContactMonitorConfig | load_threshold | .01 | N |
| 同上 | face_distance_tolerance | .0015 | m |
| 同上 | match_s_tolerance | .003 | m |
| 同上 | match_gap_tolerance | .0005 | m |
| 同上 | slip_flag_speed | .002 | m/s |
| ProgressConfig | window_seconds/min_samples | 1.0/10 | s/samples |
| 同上 | position_scale/angle_scale | .04/.15 | m/rad |
| 同上 | min_absolute_decrease/min_relative_decrease | .01/.02 | normalized V/fraction |
| SupervisorConfig | position_tolerance/angle_tolerance | .005 / 3deg | m/rad (配置使用弧度) |
| 同上 | goal_hold_seconds | .2 | s |
| 同上 | contact_confirm_seconds/timeout | .06/.5 | s |
| 同上 | min_contact_hold_seconds | .5 | s |
| 同上 | loss_gap/loss_seconds | .0005/.06 | m；s（距离；持续时间） |
| 同上 | max_s_drift | .008 | m |
| 同上 | max_contact_penetration | .0015 | m；选定面的几何异常界限 |
| 同上 | max_selection_attempts | 4 | task-level calls |
| 同上 | max_task_seconds | 30 | s |
| 同上 | max_command_speed | .08 | m/s，demo还检查不超过env上限 |

这些值是可解释的首版工程选择，不是证明或标定。3cm/10deg 的初始计划成功阈值对厘米级局部任务过宽，当前使用5mm/3deg并保持0.2s。为将来 A/B/C 公平比较，成功、监测与绕行参数必须共用并完整记录。

## 演示和限制

`rule_demo` 提供 translation、rotation、right、rotated 四个物理闭环演示。每个任务仅在开始设置一次初始位姿，将 Pusher 放在规则选择的接触附近，再用真实低速指令建立载荷。选择时没有使用未来物块轨迹。rotation 的相对目标来自 Step 2 基线可达目标，作为固定任务常量保存；不会随模型 ell/friction 改变。之后每步只执行 supervisor 的速度。

`reposition` 故意把 Pusher 初始化在错误一侧，必须返回请求而不能成功；`stagnant` 保留同一物理旋转任务，只把成功阈值设为0.5mm/0.005rad，观察当前局部模型/执行失配造成的小残差停滞，触发重新选点。它是失败处理演示，没有改变 Slider 动力学或冻结物块。

本轮 Step 3 完成定义：规则、停滞与异常事件、shared supervisor、独立配置/适配器、真实简单任务闭环、明确的 Step 4 handoff。尚不能从任意初始 Pusher 位置执行绕行，也不声称已完成必须换面的任务；这些属于 Step 4。

本版本任务达到阈值就停止并保持，因此最终误差可能接近5mm；不能与 Step 2 连续推动6秒后的稳态误差混为同一指标。选点策略也没有完整可达性保证。精度、规则、检测与运动策略均可分别替换，具体位置见接口文档。
