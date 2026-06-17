# Simulink 模型设计指南

## 📐 模型结构概览

本指南说明如何在Simulink中构建机械臂力-位置混合控制系统模型。

---

## 🏗️ 主要子系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                   混合控制系统顶层                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────────┐  ┌──────────────────┐              │
│  │  参考轨迹生成     │  │  位置控制器      │              │
│  │  (Reference Gen) │  │  (Pos PID)       │              │
│  └────────┬─────────┘  └────────┬─────────┘              │
│           │ x_d               │ u_p                      │
│           │                    │                         │
│           │                    ▼                         │
│           │                ┌──────┐                      │
│           │                │  +   │◄───────┐            │
│           │                └──┬───┘        │            │
│           │                   │           │u_f         │
│  ┌────────┴─────────┐         │      ┌────────────┐   │
│  │ 力参考生成       │         │      │力控制器    │   │
│  │(Force Ref)      │         │      │(Force PID) │   │
│  └────────┬─────────┘         │      └────────┬────┘   │
│           │ F_d               │             │        │
│           │                   │             │        │
│           │ ◄──────────────────┴─────────────┘        │
│           │      F_e (环境力反馈)                      │
│           │                                           │
│           ▼                                           │
│        ┌──────────────────┐                          │
│        │ 机械臂动力学模型 │ u_total                   │
│        │  (Arm Dynamics)  ├──────►x                  │
│        └────────┬─────────┘                          │
│                 │                                    │
│                 ▼                                    │
│        ┌──────────────────┐                          │
│        │  环境模型        │                          │
│        │(Environment)     ├──────►F_e               │
│        └──────────────────┘                          │
│                                                      │
└──────────────────────────────────────────────────────┘
```

---

## 🔧 关键子系统详细设计

### 1️⃣ 位置参考轨迹生成（Reference Generation）

**用途**：生成期望位置和速度轨迹

**实现方式**：
- 使用 Ramp 块生成线性上升轨迹
- 或使用 Signal Builder 自定义复杂轨迹
- 使用 From Workspace 导入预先定义的轨迹

**Simulink 配置**：
```
时间周期分段：
├─ 0-5s: 从0线性增加到x_d = 0.5m
├─ 5-15s: 保持x_d = 0.5m
└─ 15s: 终止
```

**关键参数**（从 arm_hybrid_control.m 传入）：
- `traj_params.x_d` - 期望位置
- `traj_params.ramp_time` - 上升时间

---

### 2️⃣ 位置 PID 控制器（Position PID Controller）

**控制律**：
$$u_p = K_p e_x + K_i \int e_x dt + K_d \frac{d e_x}{dt}$$

其中：$e_x = x_d - x$ （位置误差）

**Simulink 实现**：
```
位置误差 e_x
    │
    ├─► Proportional Gain (Kp=100) ─┐
    │                                │
    ├─► 积分器 (1/s) ──► Integral Gain (Ki=10) ──┤
    │                                │
    └─► 微分器 (Td/Ts) ──► Derivative Gain (Kd=5) ──┤
                                     │
                                     ▼
                                  Sum ──► Saturation ──► u_p
```

**关键配置**：
- **Integral Gain**: Ki/Kp = 10/100 = 0.1
- **Derivative Gain**: Kd = 5
- **Anti-windup**: 启用饱和限制器
- **Saturation Limits**: u_p ∈ [-50, 50]

---

### 3️⃣ 力 PID 控制器（Force PID Controller）

**控制律**：
$$u_f = K_{p,f} e_F + K_{i,f} \int e_F dt + K_{d,f} \frac{d e_F}{dt}$$

其中：$e_F = F_d - F_e$ （力误差）

**重要注意**：
- 力控制增益通常远小于位置控制增益
- $K_{p,f} = 1$ 相对于 $K_p = 100$
- 比例：约 1:100

**Simulink 实现**：
```
力误差 e_F
    │
    ├─► Proportional Gain (Kp=1) ──┐
    │                               │
    ├─► 积分器 ──► Integral Gain (Ki=0.1) ──┤
    │                               │
    └─► 微分器 ──► Derivative Gain (Kd=0.05) ──┤
                                    │
                                    ▼
                                  Sum ──► Saturation ──► u_f
```

**关键配置**：
- **Saturation Limits**: u_f ∈ [-20, 20]
- **启用积分饱和**防止 Windup

---

### 4️⃣ 混合控制合成（Hybrid Control Synthesis）

**合成方法**：
$$u_{total} = w_{pos} \cdot u_p + w_{force} \cdot u_f$$

其中权重满足：$w_{pos} + w_{force} = 1.0$

**建议权重**：
- $w_{pos} = 0.7$ （位置优先）
- $w_{force} = 0.3$ （力次之）

**Simulink 实现**：
```
      u_p (位置控制)
       │
       ├─► Gain (0.7) ──┐
       │                 │
       ▼                 ▼
    ┌─────┐
    │ Sum │ ──► u_total
    └─────┘
       ▲                 ▲
       │                 │
       ├─ Gain (0.3) ──┘
       │
      u_f (力控制)
```

**可选**：使用切换逻辑根据接触情况调整权重
```matlab
if (F_e < 0.5)  % 未接触
    w_pos = 1.0; w_force = 0.0;
else             % 已接触
    w_pos = 0.7; w_force = 0.3;
end
```

---

### 5️⃣ 机械臂动力学模型（Arm Dynamics）

**运动方程**：
$$m\ddot{x} + b\dot{x} = u_{total} - F_e$$

其中：
- $m = 2$ kg （臂杆质量）
- $b = 0.5$ （阻尼系数）
- $u_{total}$ （总控制力）
- $F_e$ （环境反馈力）

**Simulink 实现**（使用积分器级联）：

```
u_total - F_e
      │
      ▼
   [1/m] ──► 积分器1 ──► ẋ
             │            │
             │            ├─► Gain(b) ──┐
             │            │             │
             │            └─► 积分器2 ──► x
             │                         
             └─────── Gain(-b/m) ◄──────┘
```

**关键参数传入**：
- `arm_params.m` - 质量
- `arm_params.b` - 阻尼系数

---

### 6️⃣ 环境接触模型（Environment Model）

**线性弹簧-阻尼模型**：
$$F_e = K_{env}(x - x_{contact}) + B_{env}\dot{x}$$

激活条件：$x \geq x_{contact}$

**Simulink 实现**：

```
       ┌─ If 语句 ──► [K_env, B_env, friction]
   x   │
    ├──┤ 
    │  └─ Else ──► [0]
    │
    ▼
[K_env * max(0, x - x_contact) + B_env * dx - friction_force]
```

**使用 MATLAB Function 块实现**：
```matlab
function F_e = environment_model(x, dx, K_env, B_env, x_contact)
    if x >= x_contact
        F_e = K_env * (x - x_contact) + B_env * dx;
        if F_e > 0
            F_e = F_e - 2;  % 库伦摩擦
        end
    else
        F_e = 0;
    end
end
```

**关键参数**：
- `env_params.K_env` = 5000 N/m （环境刚度）
- `env_params.B_env` = 50 N·s/m （环境阻尼）
- `env_params.contact_pos` = 0.5 m （接触位置）

---

## 📊 信号流总结

| 信号名 | 类型 | 值范围 | 来源 → 去向 |
|-------|------|-------|----------|
| x_d | 参考 | [0, 0.5] m | 轨迹生成 → 位置控制 |
| x | 反馈 | [0, 0.5] m | 臂杆 → 位置控制、力控制 |
| e_x | 误差 | [-0.5, 0.5] m | 位置控制 |
| u_p | 控制 | [-50, 50] N | 位置PID → 合成 |
| F_d | 参考 | [0, 10] N | 力参考生成 → 力控制 |
| F_e | 反馈 | [0, 20] N | 环境模型 → 力控制 |
| e_F | 误差 | [-10, 10] N | 力控制 |
| u_f | 控制 | [-20, 20] N | 力PID → 合成 |
| u_total | 合成 | [-50, 50] N | 合成 → 臂杆动力学 |

---

## 🎮 Simulink 配置建议

### 求解器设置
```
求解器: ode45 (Dormand-Prince)
相对容差: 1e-4
绝对容差: 1e-5
最大步长: auto
```

### 数据日志
```
• 记录所有关键信号
• 采样时间: 0.001 s
• 仿真时长: 15 s
```

### 预处理参数
```matlab
% 在 arm_hybrid_control.m 中设置所有参数
% Simulink 模型将自动加载工作区变量
% 使用 From Workspace 或 Constant 块传入参数
```

---

## 🐛 常见问题排查

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 位置超调过大 | Kp 过高 | 减小Kp或增加Kd |
| 力控制发散 | Ki 过高 | 启用anti-windup限制 |
| 振荡 | 采样时间过大 | 减小Ts到0.001s |
| 位力冲突 | 权重设置不合理 | 调整w_pos和w_force |
| 阶跃响应缓慢 | 增益太小 | 增加Kp |

---

## 📈 仿真验证清单

- [ ] 参数初始化完成（arm_hybrid_control.m）
- [ ] 所有块正确连接
- [ ] 信号维数匹配
- [ ] 无代数环（Algebraic loops）
- [ ] 采样时间设置合理
- [ ] 数据日志已启用
- [ ] 运行仿真成功
- [ ] 分析脚本正确执行（analyze_results.m）

---

## 📞 模型验证步骤

1. **逐步运行仿真**
   - 先运行不带力控制（w_force=0）
   - 验证位置控制性能
   - 再开启力控制

2. **信号观察**
   - 在Scope中实时监测关键信号
   - 检查无异常跳变

3. **参数敏感性分析**
   - 逐一改变控制参数
   - 观察系统响应变化

4. **结果评估**
   - 运行 analyze_results.m
   - 对比性能指标

---

更多信息请参考主 README.md 文件！

