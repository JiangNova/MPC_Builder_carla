# Project Progress Tracker

## 总目标
在 `lvlm_mpc_carla` 中先复现论文里的 MPC Builder，再逐步接入 CARLA 场景、危险环境感知、视觉/LLM 风险理解，以及 primitives 扩展，最终形成“感知 -> 语义 -> primitives -> MPC -> CARLA 控车”的完整链路。

---

## 已完成
- [x] 梳理 `lvlm_mpc_carla` 目录结构与各部分作用
- [x] 明确当前优先目标：先复现 MPC Builder
- [x] 按论文结构补齐 `lvlm_mpc_carla/src/control_paper/mpc_builder/` 核心代码
- [x] 实现 primitives 组合与基础模型定义
- [x] 实现轻量版 MPPI 求解器
- [x] 增加 `MPCBuilder` 高层编排逻辑
- [x] 增加最小可运行 demo 入口 `run_mpc_builder_demo.py`
- [x] 增加结果可视化输出
- [x] 修复 `MPCBuilder` 导入问题
- [x] 修复 composed primitive 的状态维度问题
- [x] 创建项目进度记录文件 `PROJECT_PROGRESS.md`
- [x] 论文对齐：逐项对比论文 Section III-V、Table I、Algorithm 1-2
- [x] 论文对齐：补齐全部 6 个 primitive（KBM/LK/LC/CS/ACC/PV）
- [x] 论文对齐：实现 Primitive Assigner（论文 Section V-A 规则）
- [x] 论文对齐：恢复 MPPI 论文参数（σa=2.0, σδ=0.01）
- [x] 论文对齐：Composer 状态增广（PV primitive 时 8D 状态空间）

---

## 当前正在做的事情
- [x] 本地运行 `run_mpc_builder_demo.py`，demo 输出正常
- [x] MPPI 求解器修复：每时间步独立采样噪声序列（替代常量控制）
- [x] Demo 扩展为闭环滚动时域仿真（100 步 × 0.05s = 5s）
- [x] 论文对齐：补齐 6 个 primitive（KBM/LK/LC/CS/ACC/PV）对齐 Table I
- [x] 论文对齐：实现 Primitive Assigner（T→横向, 前车距离→CS/ACC, 周车→PV）
- [x] 论文对齐：MPPI 参数恢复论文值（σa=2.0, σδ=0.01）
- [x] 论文对齐：Composer 状态增广支持（max state_dim, PV 时自动 8D）
- [x] 验证输出：y=0.007（几乎完美车道保持），v=9.89→12，σa=2.0/σδ=0.01 下稳定
- [x] 实现公式 (10) 完整 feasibility check（horizon 模拟）
- [x] 实现 iOCP（Algorithm 1，惩罚项转化）
- [x] 实现 MPC Switcher（Algorithm 2 完整逻辑：feasible→accept / infeasible→iOCP / limit→reject）
- [x] 编写 `run_iocp_test.py`：任务切换场景测试（IDLE → LANE_RIGHT，前车阻挡触发 iOCP）
- [x] 修复 iOCP 嵌套膨胀 bug（previous_ocp 在 iOCP 模式下不更新，避免状态维度爆炸）
- [x] 修复同 OCP 误触发 iOCP bug（同名 OCP 跳过 feasibility check + 数值容差 1e-6）
- [x] iOCP 测试通过：16 步 iOCP 引导后 LC 被接受，pre-switch 无异常触发
- [x] **第一阶段完成：论文 MPC Builder 全部核心算法已验证通过**
- [ ] 接入 CARLA 最小闭环（第二阶段）

---

## 第一阶段：先把 MPC Builder 单独复现到可用
### 1. 核心结构
- [x] 建立 `MPCPrimitive` 数据结构
- [x] 建立 `ComposedPrimitive` 组合结构
- [x] 建立 `compose_primitives()` 组合接口
- [x] 实现 `lane_keep_primitive`
- [x] 实现 `constant_speed_primitive`
- [x] 实现 `adaptive_cruise_control_primitive`
- [x] 实现 `kinematic_bicycle_dynamics`
- [x] 实现 `MPPISolver`
- [x] 实现 `MPCBuilder`
- [x] 实现 `TaskCommand`
- [x] 实现 `MPCBuilderConfig`
- [x] 实现 `MPCBuilderResult`

### 2. 最小验证
- [x] 增加最小 demo 入口脚本
- [x] 增加 demo 的结果可视化
- [x] 确认 demo 生成的图像文件路径固定且易找
- [x] 检查轨迹图、状态图、控制输出符合预期
- [x] 补充横向误差、速度误差、cost 曲线等图（闭环 3 图：summary/states/diagnostics）
- [x] 增加结果自动保存为 JSON（closed_loop_data.json）

### 3. 论文对齐修正
- [x] 进一步对齐论文中的 primitive 设计（6 个 primitive → Table I）
- [x] 进一步对齐论文中的组合逻辑（状态增广，支持 PV）
- [x] 进一步对齐论文中的 feasibility check（公式 10，horizon 模拟）
- [x] 进一步对齐论文中的 iOCP（Algorithm 1，惩罚项转化）
- [x] 进一步对齐论文中的 MPC Switcher（Algorithm 2 完整实现）
- [x] 进一步对齐论文中的 Primitive Assigner（Section V-A 规则）
- [x] 补充更接近论文的参数设置与命名（σa=2.0, σδ=0.01, µ=100）

---

## 第二阶段：接入 CARLA 最小闭环
- [ ] 确认 CARLA Python API 可正常调用
- [ ] 编写 CARLA 连接与 world 初始化代码
- [ ] 获取 ego vehicle 的状态（位置、航向、速度）
- [ ] 将 CARLA 状态映射到 MPC Builder 的状态向量 `x0`
- [ ] 将 MPC 输出映射到 CARLA control 命令（throttle / brake / steer）
- [ ] 搭建最简单的直道闭环测试场景
- [ ] 验证车辆能在 CARLA 中稳定执行 MPC 输出
- [ ] 保存 CARLA 闭环轨迹和日志

---

## 第三阶段：把 MPC Builder 扩展成论文里的任务切换框架
- [x] 实现 `Primitive Assigner`（已在 Phase 1 完成）
- [x] 实现 `MPC Switcher`（已在 Phase 1 完成，含 iOCP + rejection）
- [x] 实现 `iOCP`（已在 Phase 1 完成，惩罚项转化 + 状态增广）
- [x] 实现任务切换时的 feasibility feedback（`check_feasibility` 公式 10）
- [x] 实现任务切换时的 rejection flag（`MPCBuilderResult.rejected`）
- [x] 建立任务命令到 primitives 的映射规则（`assign_primitives` Section V-A）
- [x] 支持多个 task command 的动态切换（`run_iocp_test.py` 验证通过）
- [x] 验证切换过程不会死锁（16 步 iOCP 后自动 accept，无死循环）
- [ ] 验证切换过程足够平滑（MPPI 参数可能需要针对大横向位移调优）

---

## 第四阶段：接入危险场景 primitives
- [ ] 设计油污区域 primitive
- [ ] 设计湿滑路面 primitive
- [ ] 设计施工区 primitive
- [ ] 设计车道封闭 primitive
- [ ] 设计低附着区域 primitive
- [ ] 设计“可通行 / 高风险 / 禁止进入”语义约束
- [ ] 将危险区域转换为 MPC 代价项或约束项
- [ ] 验证危险场景下的避让与减速行为

---

## 第五阶段：接入视觉模型 / LLM 风险语义
- [ ] 定义视觉模型输入与输出格式
- [ ] 让模型识别危险物体与危险路面区域
- [ ] 输出危险位置、范围、严重程度、可通行车道
- [ ] 让 LLM 将视觉结果转成结构化任务命令
- [ ] 建立从自然语言到 primitives 的中间表示
- [ ] 验证 LLM 输出不直接控制底层车辆
- [ ] 验证 LLM 只负责高层语义与任务规划

---

## 第六阶段：完整实验与论文材料整理
- [ ] 统一实验配置管理
- [ ] 统一日志与结果保存目录
- [ ] 统一绘图风格与图片输出路径
- [ ] 记录每个实验场景的参数和结果
- [ ] 生成适合论文的对比图、消融图、轨迹图
- [ ] 记录失败案例和边界情况
- [ ] 整理成实验报告或论文复现说明

---

> **维护约定**：每次完成一个任务后，必须在本文档中更新对应阶段的进度（勾选完成的项、在"当前正在做的事情"中反映最新状态）。

