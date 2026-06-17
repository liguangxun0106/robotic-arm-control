%% ========================================================================
%  机械臂力-位置混合控制仿真 - 参数初始化脚本
%  ========================================================================
%  本脚本初始化所有仿真参数，包括：
%  - 机械臂物理参数
%  - 位置控制器参数
%  - 力控制器参数
%  - 期望轨迹参数
%  - 环境接触参数
%  ========================================================================

clear all; close all; clc;

fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════╗\n');
fprintf('║     机械臂力-位置混合控制仿真 - 参数初始化            ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');

%% ========================================================================
%  1. 机械臂物理参数设置
%  ========================================================================
fprintf('▶ 设置机械臂物理参数...\n');

arm_params.m = 2;           % 臂杆质量 [kg]
arm_params.L = 0.5;         % 臂杆长度 [m]
arm_params.J = 0.1;         % 转动惯量 [kg·m²]
arm_params.b = 0.5;         % 阻尼系数 [N·s/m]
arm_params.g = 9.8;         % 重力加速度 [m/s²]

fprintf('  ✓ 质量: %.2f kg\n', arm_params.m);
fprintf('  ✓ 长度: %.2f m\n', arm_params.L);
fprintf('  ✓ 转动惯量: %.2f kg·m²\n', arm_params.J);
fprintf('  ✓ 阻尼系数: %.2f N·s/m\n', arm_params.b);

%% ========================================================================
%  2. 位置控制器参数（PID）
%  ========================================================================
fprintf('\n▶ 设置位置控制器参数 (PID)...\n');

pos_control.Kp = 100;       % 比例增益
pos_control.Ki = 10;        % 积分增益
pos_control.Kd = 5;         % 微分增益
pos_control.tau_i = 1;      % 积分时间常数 [s]
pos_control.tau_d = 0.1;    % 微分时间常数 [s]
pos_control.sat_max = 50;   % 输出饱和上限
pos_control.sat_min = -50;  % 输出饱和下限

fprintf('  ✓ 比例增益 Kp: %.2f\n', pos_control.Kp);
fprintf('  ✓ 积分增益 Ki: %.2f\n', pos_control.Ki);
fprintf('  ✓ 微分增益 Kd: %.2f\n', pos_control.Kd);
fprintf('  ✓ 输出范围: [%.2f, %.2f]\n', pos_control.sat_min, pos_control.sat_max);

%% ========================================================================
%  3. 力控制器参数（PID）
%  ========================================================================
fprintf('\n▶ 设置力控制器参数 (PID)...\n');

force_control.Kp = 1;       % 比例增益 (通常远小于位置控制增益)
force_control.Ki = 0.1;     % 积分增益
force_control.Kd = 0.05;    % 微分增益
force_control.tau_i = 1;    % 积分时间常数 [s]
force_control.tau_d = 0.1;  % 微分时间常数 [s]
force_control.sat_max = 20; % 输出饱和上限
force_control.sat_min = -20;% 输出饱和下限

fprintf('  ✓ 比例增益 Kp: %.2f\n', force_control.Kp);
fprintf('  ✓ 积分增益 Ki: %.2f\n', force_control.Ki);
fprintf('  ✓ 微分增益 Kd: %.2f\n', force_control.Kd);
fprintf('  ✓ 输出范围: [%.2f, %.2f]\n', force_control.sat_min, force_control.sat_max);

%% ========================================================================
%  4. 环境和接触参数
%  ========================================================================
fprintf('\n▶ 设置环境接触参数...\n');

env_params.K_env = 5000;    % 环境刚度 [N/m]
env_params.B_env = 50;      % 环境阻尼 [N·s/m]
env_params.F_friction = 2;  % 摩擦力 [N]
env_params.contact_pos = 0.5; % 接触位置 [m]

fprintf('  ✓ 环境刚度: %.2f N/m\n', env_params.K_env);
fprintf('  ✓ 环境阻尼: %.2f N·s/m\n', env_params.B_env);
fprintf('  ✓ 摩擦力: %.2f N\n', env_params.F_friction);
fprintf('  ✓ 接触位置: %.2f m\n', env_params.contact_pos);

%% ========================================================================
%  5. 期望轨迹参数
%  ========================================================================
fprintf('\n▶ 设置期望轨迹...\n');

traj_params.x_d = 0.5;      % 期望位置 [m]
traj_params.F_d = 10;       % 期望接触力 [N]
traj_params.ramp_time = 5;  % 位置上升时间 [s]
traj_params.force_ramp_time = 2; % 力上升时间 [s]
traj_params.force_delay = 3;     % 力控制延迟 [s]

fprintf('  ✓ 期望位置: %.2f m\n', traj_params.x_d);
fprintf('  ✓ 期望接触力: %.2f N\n', traj_params.F_d);
fprintf('  ✓ 位置上升时间: %.2f s\n', traj_params.ramp_time);
fprintf('  ✓ 力上升时间: %.2f s\n', traj_params.force_ramp_time);

%% ========================================================================
%  6. 仿真配置参数
%  ========================================================================
fprintf('\n▶ 设置仿真参数...\n');

sim_params.Ts = 0.001;      % 采样时间 [s]
sim_params.t_final = 15;    % 仿真终止时间 [s]
sim_params.solver = 'ode45'; % 求解器类型

% 初始条件
sim_params.x_init = 0;      % 初始位置 [m]
sim_params.v_init = 0;      % 初始速度 [m/s]
sim_params.F_init = 0;      % 初始力 [N]

fprintf('  ✓ 采样时间: %.4f s\n', sim_params.Ts);
fprintf('  ✓ 仿真时长: %.2f s\n', sim_params.t_final);
fprintf('  ✓ 初始位置: %.2f m\n', sim_params.x_init);
fprintf('  ✓ 初始速度: %.2f m/s\n', sim_params.v_init);

%% ========================================================================
%  7. 混合控制权重参数
%  ========================================================================
fprintf('\n▶ 设置混合控制权重...\n');

hybrid_params.w_pos = 0.7;  % 位置控制权重 (0-1)
hybrid_params.w_force = 0.3;% 力控制权重 (0-1)

% 验证权重和为1
if abs(hybrid_params.w_pos + hybrid_params.w_force - 1.0) > 1e-6
    warning('警告: 位置和力控制权重之和不为1！');
end

fprintf('  ✓ 位置控制权重: %.2f\n', hybrid_params.w_pos);
fprintf('  ✓ 力控制权重: %.2f\n', hybrid_params.w_force);

%% ========================================================================
%  8. 保存参数到MATLAB工作区
%  ========================================================================
fprintf('\n▶ 保存参数到工作区...\n');

assignin('base', 'arm_params', arm_params);
assignin('base', 'env_params', env_params);
assignin('base', 'pos_control', pos_control);
assignin('base', 'force_control', force_control);
assignin('base', 'traj_params', traj_params);
assignin('base', 'sim_params', sim_params);
assignin('base', 'hybrid_params', hybrid_params);

fprintf('  ✓ 所有参数已保存\n');

%% ========================================================================
%  9. 参数汇总表
%  ========================================================================
fprintf('\n╔═══════════════════════════════════════════════════════╗\n');
fprintf('║                    参数汇总                            ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');

fprintf('【机械臂参数】\n');
fprintf('  质量: %.2f kg | 长度: %.2f m | 转动惯量: %.3f kg·m²\n\n', ...
    arm_params.m, arm_params.L, arm_params.J);

fprintf('【控制参数】\n');
fprintf('  位置控制: Kp=%.1f, Ki=%.1f, Kd=%.2f\n', ...
    pos_control.Kp, pos_control.Ki, pos_control.Kd);
fprintf('  力控制:   Kp=%.2f, Ki=%.2f, Kd=%.3f\n\n', ...
    force_control.Kp, force_control.Ki, force_control.Kd);

fprintf('【环境参数】\n');
fprintf('  刚度: %.0f N/m | 阻尼: %.1f N·s/m\n\n', ...
    env_params.K_env, env_params.B_env);

fprintf('【期望轨迹】\n');
fprintf('  位置: %.2f m | 接触力: %.1f N\n\n', ...
    traj_params.x_d, traj_params.F_d);

fprintf('【仿真配置】\n');
fprintf('  时长: %.1f s | 步长: %.4f s | 总步数: %.0f\n\n', ...
    sim_params.t_final, sim_params.Ts, sim_params.t_final/sim_params.Ts);

fprintf('╔═══════════════════════════════════════════════════════╗\n');
fprintf('║              参数初始化完成！                          ║\n');
fprintf('║     下一步：打开 hybrid_control_simulink.slx         ║\n');
fprintf('║            并运行仿真                                ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');

%% ========================================================================
%  10. 可选：参数导出为JSON格式（便于其他工具使用）
%  ========================================================================
function export_params_json(arm_params, pos_control, force_control, env_params)
    % 可选功能：导出参数为JSON格式
    
    params_struct.arm_params = arm_params;
    params_struct.pos_control = pos_control;
    params_struct.force_control = force_control;
    params_struct.env_params = env_params;
    
    json_str = jsonencode(params_struct);
    fid = fopen('simulation_params.json', 'w');
    fprintf(fid, '%s', json_str);
    fclose(fid);
    
    fprintf('  ✓ 参数已导出到 simulation_params.json\n');
end

% 取消注释以启用JSON导出
% export_params_json(arm_params, pos_control, force_control, env_params);

%% ========================================================================
% END OF arm_hybrid_control.m
%% ========================================================================
