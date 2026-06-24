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

---

## 当前正在做的事情
- [ ] 本地运行 `run_mpc_builder_demo.py`，确认 demo 输出和图片生成正常
- [ ] 检查 demo 输出数值是否合理（控制量、cost、feasible、trajectory）
- [ ] 根据第一次运行结果继续修正 MPC Builder 的实现细节

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
- [ ] 确认 demo 生成的图像文件路径固定且易找
- [ ] 检查轨迹图、状态图、控制输出是否符合预期
- [ ] 补充横向误差、速度误差、cost 曲线等图
- [ ] 增加结果自动保存为 JSON/CSV

### 3. 论文对齐修正
- [ ] 进一步对齐论文中的 primitive 设计
- [ ] 进一步对齐论文中的组合逻辑
- [ ] 进一步对齐论文中的 feasibility check
- [ ] 进一步对齐论文中的 iOCP
- [ ] 进一步对齐论文中的 MPC Switcher
- [ ] 进一步对齐论文中的 Primitive Assigner
- [ ] 补充更接近论文的参数设置与命名

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
- [ ] 实现 `Primitive Assigner`
- [ ] 实现 `MPC Switcher`
- [ ] 实现 `iOCP`
- [ ] 实现任务切换时的 feasibility feedback
- [ ] 实现任务切换时的 rejection flag
- [ ] 建立任务命令到 primitives 的映射规则
- [ ] 支持多个 task command 的动态切换
- [ ] 验证切换过程不会死锁
- [ ] 验证切换过程足够平滑

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

## 目前推荐执行顺序
1. 先确认 `run_mpc_builder_demo.py` 的输出和图片生成正常
2. 再补更多结果图和结果文件保存
3. 然后接 CARLA 的最小闭环接口
4. 再逐步补 `iOCP`、`MPC Switcher`、`Primitive Assigner`
5. 最后做危险场景和视觉/LLM 扩展

---

## 备注
- 当前策略：先让 MPC Builder 单独跑通，再接 CARLA。
- 后续所有新增任务建议按“先最小闭环，再逐步扩展”的顺序推进。
- 这个文件会作为整个项目的主进度表，后续每完成一步就更新一次。
