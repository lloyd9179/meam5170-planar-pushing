# Step 0–4 的接口与修改边界（contract version 1, additive revision 1）

Step 3 与 Step 4 已分别实现：规则选点/任务状态机，以及可替换的绕行规划、代价评估和运动执行器。模块采用小型 dataclass + Python Protocol，不要求继承一个庞大的控制器基类。保留原 Step 0–2 的 API、代码与测试。早期 Step 3 的独立运行仍可不注入 Step 4；完整任务使用 `--with-reposition`。

## 所有权和依赖方向

```text
MuJoCo env --observe--> MuJoCoObservationAdapter --> WorldObservation
                                                      |
                         RuleContactSelector <--- TaskSupervisor
                            | ContactTarget            |
                            +--------------------------+
                                                      |
                     PUSH -----------------------------+-- REPOSITION
                      |                                       |
             LocalMPCPushAdapter                 RepositionExecutor (Step 4)
                      |                                       |
                existing LocalQPMPC                  RepositionPlanner (Step 4)
                      |                                       |
             world XY velocity <------------------------------+
                      |
                   env.step
```

- 仿真只接收世界坐标速度 `[vx, vy]`。task 模块的 selector、monitor、progress、supervisor 不持有 env，不读取 qpos，不调用 reset，也不直接推进仿真。
- `WorldObservation` 是脱离仿真内存的不可变值；只有 `task/adapters.py` 访问 MuJoCo 具名 body/contact。每个观察时间只调用一次 supervisor.tick；时间必须严格递增。
- `RectangleGeometry` 描述物理尺寸、Pusher 半径与安全边距，来自场景配置；不从 nominal friction 推导真实场景。底层 n/t 几何计算复用现有 contact_geometry，避免两套符号。
- `ModelConfig` 与 `EnvConfig` 分别注入，不允许 selector、supervisor 或 MPC adapter 构造/重置环境来“切换接触”。Step 3 runner 也不复用带 reset 的旧 `prepare_contact`。
- 外层只识别姿态、物理接触与世界速度。原 4 状态/2 输入模型数组、线性化和 OSQP 解保留在 LocalMPCPushAdapter 内部。

## 值类型和单位

所有定义位于 `src/planar_pushing/task/contracts.py`；`CONTRACT_VERSION=1` 随日志保存。

| 类型 | 关键字段与语义 |
|---|---|
| WorldObservation | slider_pose=(world x,y,yaw)，pusher_position=world center，slider_twist=(world vx,vy,wz)，pusher_velocity=actual world vx,vy，time=s，contacts=实际接触位置与载荷 |
| ContactTarget | face + **期望** body-face s，m；不包含 MuJoCo ID、模型状态数组、预测摩擦或世界接触坐标 |
| ContactEstimate | 当前 face 下的 **measured_s**、signed gap、physical in_bounds、loaded/normal force、相对材料切向速度、接触点误差 |
| SelectionContext | 当前已激活接触、任务内阻止重复失败的接触、触发原因 |
| SelectionResult | target 或 None，必须带可解释 reason |
| PushResult | 世界速度、可行性/status、总求解时间、迭代数/目标/残差；不向 supervisor 泄露优化变量索引 |
| TaskDecision | 唯一下一步世界速度、phase/reason、目标与接触诊断、可选重定位请求/求解日志 |
| RepositionRequest | 目标、旧接触、触发原因、请求时间、物块位姿快照；只是请求，不表示已执行 |
| RepositionPlan | start 到 precontact 的世界中心路径、target、物块位姿 anchor；length_m 由实际 waypoints 求和，避免路径和代价不一致 |
| PlanningResult | plan 或 None + reason，规划失败显式返回 |
| MotionUpdate | 世界速度、受限运动 phase、running/ready/failed、reason；ready **不等于** 物理接触已确认 |

长度 m，角 rad，速度 m/s，角速度 rad/s，力 N，时间使用仿真时间 s。yaw 观测允许 wrapped，角误差使用 atan2(sin,cos)；连续角度只在 LocalMPCPushAdapter 内建立。mocap 的测得速度不代表真实运动表面速度，观察将 pusher_velocity_valid=False，Step 3 demo 拒绝该执行模式，避免复现 Step 2 已诊断的问题。

signed gap 是相对于所选面的 supporting line 的几何量；Pusher 在物块另一侧时它可能很负，不能解释为全局碰撞穿透。激活前检查面、接触位置和载荷；激活后出现明显负 gap 会触发 selected_face_geometry_invalid。

## 可替换接口

```python
selector.reset(seed=0)
selection = selector.select(observation, goal_pose, context)

push_controller.reset()
push_controller.activate(target)  # 每个新接触任务，包括相同面不同 s
push = push_controller.compute(observation, contact_estimate, goal_pose)

decision = supervisor.tick(observation, goal_pose)
# runner 是唯一推进环境的位置：
env.step(decision.velocity_world)
```

高层未来 B/C 替换 ContactSelector 即可，沿用 supervisor 和 push controller。新 selector 可以独立持有模型/评估器，但不得修改真实环境。所有组共用 contact confirmation、异常处理、成功条件和 Step 4 的执行路径，避免比较时改变底层执行能力。

LocalMPCPushAdapter 在每次 activate 时无条件清除 warm start，即使 face 相同、只换了 s；每次 compute 使用最新 measured_s 而不是把 desired_s 塞回实际状态。不会逐步重置来强制维持“理想 sticking”。

## 状态与转移

```text
SELECT -> 已经在目标接触且有载荷 -> CONFIRM_CONTACT -> PUSH
   |
   +---- 需要移动到目标接触 -----> REPOSITION_REQUIRED
                                      |
                      有 Step 4 port：RETRACT -> TRANSIT -> APPROACH
                                      |
                                CONFIRM_CONTACT -> PUSH

PUSH -- 停滞/丢失/越界/严重漂移/QP失败 --> SELECT
任一非终止状态 -- 满足位置和角度阈值 --> GOAL_HOLD -> SUCCEEDED
任一非终止状态 -- 超时/重试耗尽/运动失败 --> FAILED
```

- 没有 Step 4 port 时，REPOSITION_REQUIRED 输出零速度与完整请求，既不伪造成功，也不运行 local MPC。runner 以 `reposition_required_step4_unavailable` 停止。
- 未来 port 返回 READY 后仍需真实载荷、位置匹配连续成立 60 ms，不能仅凭“到达 waypoint”启动 MPC。
- GOAL_HOLD 输出零输入，连续 200 ms 满足位置和角度阈值才成功；不需要为了成功保持正接触力。离开阈值重新选点。
- 只在 PUSH 更新进展窗口。绕行、接触确认、成功保持、宏观丢失待确认期间不计停滞；中断时清空旧窗口。
- 停滞需满足最短接触保持时间，安全异常不受该延迟限制。
- 所有失败优化结果、等待运动、确认接触和 terminal 状态都输出零速度。任何 port 的超限世界速度触发 FAILED，不静默限幅后假装执行了优化结果。
- reset() 只清理算法历史、selector、warm start 和运动状态；不重置仿真。更换 goal 自动建立新任务并清理历史，保留当前物理观测。

## Step 4 接入约定（已实现默认组件）

```python
planning = planner.plan(observation, target)
# planner 构造时注入物理 geometry、motion config；不得写入仿真。

executor.start(observation, request)  # 每个 request 只调用一次
update = executor.update(observation)
executor.cancel()                   # 必须幂等

supervisor = TaskSupervisor(selector, push, monitor,
                            reposition_executor=executor)
```

Step 4 应提供真实的退离、膨胀障碍绕行、低速接近。路径到 precontact 的 length_m 定义为初版 Jtravel（retreat+transit），最终 approach 距离/时间单列。几何规划和跟踪执行分开；执行时验证当前位姿相对于 anchor 的变化、整段扫掠碰撞、实际位置与速度，以及物块的意外运动。不能调用初始化 reset，不可瞬移，不可把自由运动阶段施加到“只推不拉”的 local MPC 输入上。

默认组件位于 `motion/`：VisibilityRepositionPlanner、NominalTimeCost、FeedbackRepositionExecutor，以及独立可注入的 CollisionChecker。真实 MuJoCo 验证覆盖所有16个有向面组合、旋转/同面换点/自由位置起步和任务运行中的换面。算法、成本定义、边界和修改方式见 [step4_design.md](step4_design.md)。

本次只做向后兼容的增量：RepositionPlan 添加默认值 retreat_end_index/precontact_gap/clearance；PlanningResult 添加可选 cost；CONTRACT_VERSION 仍为1，CONTRACT_REVISION=1。原有三参数 RepositionPlan、两参数 PlanningResult 和 selector/push/motion Protocol 的调用保持有效。用于实际 motion executor 的计划必须带正的安全余量、正确的实际起点和预接触终点，否则明确拒绝。

Jtravel 的原定义不变：RepositionPlan.length_m 为退离+绕行到 precontact 的米数。MotionCost 另提供 total、unit 与分项，默认是名义运动时间+转弯时间代价（s）；它不替代 Jpush 或 Jswitch。更换 cost_evaluator 不改变碰撞过滤和执行安全检查。自由运动规划从 WorldObservation 获取物理几何和位姿，不读取预测模型/OSQP变量。

## 配置和兼容策略

已有 EnvConfig / ModelConfig / MPCConfig 不动，新增 RuleSelectionConfig、ContactMonitorConfig、ProgressConfig、SupervisorConfig。demo 支持 JSON 部分覆盖和世界系 `--goal X Y YAW`，未知配置 section/field 报错。配置完整展开保存到每个独立运行目录；任务 goal 固定，不因修改 nominal ell/friction 偷偷改变。

`configs/step3_example.json` 是可编辑示例。env 与 model 的 friction 具有不同的配置项；真值的更改只改变执行，nominal 的更改只改变预测/可行性。dt 的关系是明确的兼容约束：MPC dt=env.dt_control，物理子步整除控制步长；不匹配报错。物理尺寸变更不保证原 nominal 模型还匹配，必须明确修改 ModelConfig 或记录模型失配。

| 希望修改 | 修改点 | 应保留的其他部分 |
|---|---|---|
| 仿真质量/真摩擦/执行器 | EnvConfig；若换仿真后端则替换 observation adapter | selector、supervisor、controller 的外层契约 |
| 极限面/nominal friction/Jacobian | 原 prediction model | task policies 和 Step 4 |
| Q/R/H/solver 或不同输入表示 | 原 LocalQPMPC + LocalMPCPushAdapter | 世界速度契约、monitor、selector、运动执行 |
| 选面/偏心规则 | RuleContactSelector / RuleSelectionConfig | supervisor、MPC、绕行 |
| 停滞或成功条件 | ProgressConfig / SupervisorConfig 与对应策略 | 模型、QP、物理执行 |
| 绕行算法 | Step 4 planner | selector、push、motion tracking（若路径输出不变） |
| B/C 采样 | 新 ContactSelector | 同一个 supervisor/MPC/Step 4 |

版本 1 约定 world XY 控制、矩形 face+s 描述、单物体。保持这些语义的局部设计变化不需要整项目重写；引入非矩形形状、多物体、不同维度的模型或 contact-implicit 控制仍可能需要对应适配/接口版本升级，不承诺任何变化零成本。发生破坏性变更应升级 CONTRACT_VERSION，保留迁移说明与旧结果元数据。

## 验证要求

原 Step 0–2 的 67 项测试作为回归。新增测试覆盖四面/旋转坐标选点、偏心力矩符号、hysteresis、失败点有限回退、时间窗与噪声、状态机 gates、goal hold、接触丢失/严重漂移、QP失败、Step 4 port handoff、同面换点清除 warm start、真值与 nominal 解耦，以及真实 MuJoCo 闭环和无显示 CLI。
