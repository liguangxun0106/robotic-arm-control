%% ========================================================================
%  运行机械臂混合控制仿真的完整脚本
%  ========================================================================
%  此脚本完整执行以下步骤：
%  1. 初始化所有参数
%  2. 打开Simulink模型
%  3. 运行仿真
%  4. 分析结果
%  5. 保存结果
%  ========================================================================

clear all; close all; clc;

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║     机械臂力-位置混合控制仿真 - 完整执行脚本              ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

%% ========================================================================
%  步骤1: 初始化参数
%  ========================================================================
fprintf('【步骤1】初始化仿真参数...\n');

try
    arm_hybrid_control;
    fprintf('✓ 参数初始化成功\n\n');
catch ME
    fprintf('✗ 参数初始化失败: %s\n', ME.message);
    return;
end

%% ========================================================================
%  步骤2: 检查Simulink模型
%  ========================================================================
fprintf('【步骤2】加载Simulink模型...\n');

model_name = 'hybrid_control_simulink';

% 检查模型文件是否存在
if isfile([model_name '.slx'])
    fprintf('✓ 找到模型文件: %s.slx\n', model_name);
    
    try
        % 加载模型
        load_system(model_name);
        fprintf('✓ 模型加载成功\n\n');
    catch ME
        fprintf('✗ 模型加载失败: %s\n', ME.message);
        return;
    end
else
    fprintf('✗ 未找到模型文件: %s.slx\n', model_name);
    fprintf('   请先创建Simulink模型（参考 SIMULINK_MODEL_GUIDE.md）\n');
    return;
end

%% ========================================================================
%  步骤3: 运行仿真
%  ========================================================================
fprintf('【步骤3】运行仿真...\n');
fprintf('仿真进度: ');

try
    % 获取仿真配置
    sim_time = sim_params.t_final;
    
    % 显示仿真配置
    fprintf('\n  • 仿真时长: %.2f s\n', sim_time);
    fprintf('  • 采样时间: %.4f s\n', sim_params.Ts);
    fprintf('  • 预期样本数: %.0f\n\n', sim_time / sim_params.Ts);
    
    % 运行仿真
    tic;
    out = sim(model_name, 'StopTime', num2str(sim_time));
    elapsed_time = toc;
    
    fprintf('✓ 仿真完成！耗时: %.2f 秒\n\n', elapsed_time);
    
    % 验证输出
    if ~isfield(out, 'time') && ~isfield(out, 'tout')
        error('仿真输出格式错误');
    end
    
    % 显示输出信息
    if isfield(out, 'time')
        t = out.time;
    else
        t = out.tout;
    end
    
    fprintf('  • 实际样本数: %d\n', length(t));
    fprintf('  • 输出信号数: %d\n', length(fieldnames(out)));
    
catch ME
    fprintf('✗ 仿真执行失败:\n');
    fprintf('  错误信息: %s\n', ME.message);
    fprintf('  建议: 检查SIMULINK_MODEL_GUIDE.md中的模型设计\n');
    return;
end

%% ========================================================================
%  步骤4: 数据验证
%  ========================================================================
fprintf('\n【步骤4】验证仿真数据...\n');

try
    % 提取关键信号
    x = out.x.Data;
    x_d = out.x_d.Data;
    F_e = out.F_e.Data;
    F_d = out.F_d.Data;
    
    % 基本统计
    fprintf('  • 位置范围: [%.4f, %.4f] m\n', min(x), max(x));
    fprintf('  • 期望位置: %.4f m\n', mean(x_d(end-100:end)));
    fprintf('  • 力范围: [%.4f, %.4f] N\n', min(F_e), max(F_e));
    fprintf('  • 期望力: %.4f N\n', mean(F_d(end-100:end)));
    
    fprintf('✓ 数据验证成功\n\n');
    
catch ME
    fprintf('✗ 数据验证失败: %s\n', ME.message);
    return;
end

%% ========================================================================
%  步骤5: 结果分析
%  ========================================================================
fprintf('【步骤5】分析仿真结果...\n');

try
    % 调用分析脚本
    analyze_results;
    
    fprintf('\n✓ 结果分析完成\n');
    
catch ME
    fprintf('⚠ 结果分析出现问题: %s\n', ME.message);
    fprintf('  但仿真数据已在工作区中，可手动分析\n');
end

%% ========================================================================
%  步骤6: 保存结果
%  ========================================================================
fprintf('\n【步骤6】保存仿真结果...\n');

try
    % 创建结果目录
    result_dir = 'simulation_results';
    if ~isfolder(result_dir)
        mkdir(result_dir);
        fprintf('  • 创建结果目录: %s\n', result_dir);
    end
    
    % 保存MAT文件（包含所有仿真数据）
    timestamp = datetime('now', 'Format', 'yyyyMMdd_HHmmss');
    mat_filename = fullfile(result_dir, sprintf('sim_result_%s.mat', timestamp));
    
    save(mat_filename, 'out', 'arm_params', 'pos_control', 'force_control', ...
         'env_params', 'traj_params', 'sim_params', 'hybrid_params');
    fprintf('  ✓ 数据已保存: %s\n', mat_filename);
    
    % 保存工作区变量
    workspace_filename = fullfile(result_dir, sprintf('workspace_%s.mat', timestamp));
    save(workspace_filename);
    fprintf('  ✓ 工作区已保存: %s\n', workspace_filename);
    
    % 生成报告摘要
    report_file = fullfile(result_dir, sprintf('report_%s.txt', timestamp));
    fid = fopen(report_file, 'w');
    
    fprintf(fid, '========================================\n');
    fprintf(fid, '机械臂混合控制仿真报告\n');
    fprintf(fid, '========================================\n\n');
    fprintf(fid, '执行时间: %s\n\n', datetime('now'));
    fprintf(fid, '【系统参数】\n');
    fprintf(fid, '质量: %.2f kg\n', arm_params.m);
    fprintf(fid, '刚度: %.2f N/m\n', env_params.K_env);
    fprintf(fid, '期望位置: %.2f m\n', traj_params.x_d);
    fprintf(fid, '期望力: %.2f N\n\n', traj_params.F_d);
    
    fprintf(fid, '【控制参数】\n');
    fprintf(fid, '位置控制 - Kp: %.2f, Ki: %.2f, Kd: %.2f\n', ...
        pos_control.Kp, pos_control.Ki, pos_control.Kd);
    fprintf(fid, '力控制 - Kp: %.2f, Ki: %.2f, Kd: %.2f\n\n', ...
        force_control.Kp, force_control.Ki, force_control.Kd);
    
    fprintf(fid, '【仿真配置】\n');
    fprintf(fid, '仿真时长: %.2f s\n', sim_params.t_final);
    fprintf(fid, '采样时间: %.4f s\n', sim_params.Ts);
    fprintf(fid, '实际耗时: %.2f s\n\n', elapsed_time);
    
    fprintf(fid, '【关键结果】\n');
    e_pos = x_d - x;
    fprintf(fid, '位置RMSE: %.6f m\n', sqrt(mean(e_pos.^2)));
    fprintf(fid, '最大位置误差: %.6f m\n', max(abs(e_pos)));
    fprintf(fid, '最终位置: %.6f m (期望: %.2f m)\n', x(end), x_d(end));
    fprintf(fid, '最终接触力: %.4f N (期望: %.2f N)\n', F_e(end), F_d(end));
    
    fclose(fid);
    fprintf('  ✓ 报告已保存: %s\n', report_file);
    
catch ME
    fprintf('⚠ 保存结果时出错: %s\n', ME.message);
end

%% ========================================================================
%  最终总结
%  ========================================================================
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║              仿真执行完全成功！✓                          ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

fprintf('📊 输出文件：\n');
fprintf('  • Simulink仿真结果: simulation_results/\n');
fprintf('  • 分析图表: hybrid_control_results.png\n');
fprintf('  • MAT数据: simulation_results/sim_result_*.mat\n\n');

fprintf('📈 后续操作建议：\n');
fprintf('  1. 查看生成的图表理解控制效果\n');
fprintf('  2. 根据分析报告调整控制参数\n');
fprintf('  3. 修改环境参数进行对比仿真\n');
fprintf('  4. 优化混合权重(w_pos, w_force)\n\n');

fprintf('💡 参数调优建议：\n');
fprintf('  • 若位置超调过大: 减小Kp_pos或增加Kd_pos\n');
fprintf('  • 若力控制不稳定: 减小Kp_force或Kd_force\n');
fprintf('  • 若响应太慢: 适当增加Kp\n');
fprintf('  • 若出现振荡: 增加阻尼相关参数\n\n');

fprintf('更多信息请参考: README.md 和 SIMULINK_MODEL_GUIDE.md\n\n');

%% ========================================================================
% END OF run_simulation.m
%% ========================================================================
