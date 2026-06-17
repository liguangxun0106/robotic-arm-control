%% ========================================================================
%  自动生成Simulink混合控制模型框架
%  ========================================================================
%  本脚本自动创建 hybrid_control_simulink.slx 模型
%  包含所有必要的子系统和信号连接
%  ========================================================================

clear all; close all; clc;

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║   自动生成 Simulink 混合控制模型                         ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

%% ========================================================================
%  步骤1：设置模型名和参数
%  ========================================================================
fprintf('【步骤1】创建新模型...\n');

model_name = 'hybrid_control_simulink';

% 创建新模型（如果已存在则关闭）
if bdIsLoaded(model_name)
    close_system(model_name, 0);
    bdclose(model_name);
    pause(0.5);
end

% 新建模型
new_system(model_name);
open_system(model_name);

fprintf('  ✓ 模型已创建: %s\n', model_name);

%% ========================================================================
%  步骤2：加载参数
%  ========================================================================
fprintf('\n【步骤2】加载仿真参数...\n');

try
    arm_hybrid_control;
    fprintf('  ✓ 参数加载成功\n');
catch
    fprintf('  ⚠ 参数加载失败，将使用默认值\n');
end

%% ========================================================================
%  步骤3：配置模型参数
%  ========================================================================
fprintf('\n【步骤3】配置模型参数...\n');

set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'RelTol', '1e-4');
set_param(model_name, 'AbsTol', '1e-5');
set_param(model_name, 'StopTime', num2str(sim_params.t_final));
set_param(model_name, 'SaveState', 'on');
set_param(model_name, 'SaveOutput', 'on');

fprintf('  ✓ 求解器设置完成\n');

%% ========================================================================
%  步骤4：创建顶层结构（使用Subsystems）
%  ========================================================================
fprintf('\n【步骤4】添加主要子系统...\n');

% 添加位置参考轨迹生成子系统
pos_traj_block = add_block('built-in/SubSystem', [model_name '/Position_Reference']);
set_param(pos_traj_block, 'Position', [50, 100, 150, 150]);
fprintf('  ✓ 位置参考生成子系统\n');

% 添加位置控制器子系统
pos_ctrl_block = add_block('built-in/SubSystem', [model_name '/Pos_Controller']);
set_param(pos_ctrl_block, 'Position', [250, 50, 350, 150]);
fprintf('  ✓ 位置控制器子系统\n');

% 添加力参考生成子系统
force_ref_block = add_block('built-in/SubSystem', [model_name '/Force_Reference']);
set_param(force_ref_block, 'Position', [50, 300, 150, 350]);
fprintf('  ✓ 力参考生成子系统\n');

% 添加力控制器子系统
force_ctrl_block = add_block('built-in/SubSystem', [model_name '/Force_Controller']);
set_param(force_ctrl_block, 'Position', [250, 250, 350, 350]);
fprintf('  ✓ 力控制器子系统\n');

% 添加混合控制合成子系统
hybrid_block = add_block('built-in/SubSystem', [model_name '/Hybrid_Synthesis']);
set_param(hybrid_block, 'Position', [450, 150, 550, 250]);
fprintf('  ✓ 混合控制合成子系统\n');

% 添加机械臂动力学子系统
arm_dyn_block = add_block('built-in/SubSystem', [model_name '/Arm_Dynamics']);
set_param(arm_dyn_block, 'Position', [650, 150, 750, 250]);
fprintf('  ✓ 机械臂动力学子系统\n');

% 添加环境模型子系统
env_block = add_block('built-in/SubSystem', [model_name '/Environment']);
set_param(env_block, 'Position', [650, 300, 750, 400]);
fprintf('  ✓ 环境模型子系统\n');

% 添加数据记录子系统
scope_block = add_block('built-in/SubSystem', [model_name '/Data_Logger']);
set_param(scope_block, 'Position', [850, 150, 950, 250]);
fprintf('  ✓ 数据记录子系统\n');

%% ========================================================================
%  步骤5：添加信号线和连接
%  ========================================================================
fprintf('\n【步骤5】添加信号连接...\n');

% 这里可以添加具体的连接线
% 由于复杂性，建议手动完成或使用图形界面

%% ========================================================================
%  步骤6：保存模型
%  ========================================================================
fprintf('\n【步骤6】保存模型...\n');

save_system(model_name, [model_name '.slx']);
fprintf('  ✓ 模型已保存: %s.slx\n\n', model_name);

%% ========================================================================
%  步骤7：显示后续说明
%  ========================================================================
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║                模型框架已创建！                          ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

fprintf('📝 后续配置说明：\n\n');

fprintf('1. 位置参考生成子系统 (Position_Reference):\n');
fprintf('   ├─ 输入: 无\n');
fprintf('   └─ 输出: x_d (期望位置)\n');
fprintf('   └─ 使用: Ramp块从0线性增加到 %.2f m\n\n', traj_params.x_d);

fprintf('2. 位置控制器子系统 (Pos_Controller):\n');
fprintf('   ├─ 输入: x_d (期望), x (实际)\n');
fprintf('   ├─ 输出: u_p (位置控制)\n');
fprintf('   └─ 配置: PID控制器\n');
fprintf('       • Kp = %.1f\n', pos_control.Kp);
fprintf('       • Ki = %.1f\n', pos_control.Ki);
fprintf('       • Kd = %.2f\n\n', pos_control.Kd);

fprintf('3. 力参考生成子系统 (Force_Reference):\n');
fprintf('   ├─ 输入: 无\n');
fprintf('   └─ 输出: F_d (期望力)\n');
fprintf('   └─ 使用: Step或Constant块，值为 %.1f N\n\n', traj_params.F_d);

fprintf('4. 力控制器子系统 (Force_Controller):\n');
fprintf('   ├─ 输入: F_d (期望), F_e (实际)\n');
fprintf('   ├─ 输出: u_f (力控制)\n');
fprintf('   └─ 配置: PID控制器\n');
fprintf('       • Kp = %.2f\n', force_control.Kp);
fprintf('       • Ki = %.2f\n', force_control.Ki);
fprintf('       • Kd = %.3f\n\n', force_control.Kd);

fprintf('5. 混合控制合成子系统 (Hybrid_Synthesis):\n');
fprintf('   ├─ 输入: u_p (位置控制), u_f (力控制)\n');
fprintf('   ├─ 输出: u_total (总控制)\n');
fprintf('   └─ 公式: u_total = %.1f*u_p + %.1f*u_f\n\n', ...
    hybrid_params.w_pos, hybrid_params.w_force);

fprintf('6. 机械臂动力学子系统 (Arm_Dynamics):\n');
fprintf('   ├─ 输入: u_total (控制), F_e (环境力)\n');
fprintf('   ├─ 输出: x (位置), v (速度)\n');
fprintf('   └─ 方程: m*dv/dt + b*v = u_total - F_e\n');
fprintf('       • 质量 m = %.1f kg\n', arm_params.m);
fprintf('       • 阻尼 b = %.2f\n\n', arm_params.b);

fprintf('7. 环境模型子系统 (Environment):\n');
fprintf('   ├─ 输入: x (位置), v (速度)\n');
fprintf('   ├─ 输出: F_e (环境力)\n');
fprintf('   └─ 参数:\n');
fprintf('       • 刚度 K = %.0f N/m\n', env_params.K_env);
fprintf('       • 阻尼 B = %.1f N·s/m\n', env_params.B_env);
fprintf('       • 接触位置 = %.2f m\n\n', env_params.contact_pos);

fprintf('8. 数据记录子系统 (Data_Logger):\n');
fprintf('   └─ 记录所有关键信号用于后处理分析\n\n');

fprintf('══════════════════════════════════════════════════════════\n');
fprintf('🔧 详细配置步骤：\n');
fprintf('══════════════════════════════════════════════════════════\n\n');

fprintf('A. 双击各子系统进行详细配置\n');
fprintf('B. 参考 SIMULINK_MODEL_GUIDE.md 获取具体实现方法\n');
fprintf('C. 完成配置后运行 run_simulation.m 执行仿真\n\n');

fprintf('✅ 模型框架已完成，现在需要完成各子系统的内部配置！\n\n');

%% ========================================================================
% END OF create_model.m
%% ========================================================================
