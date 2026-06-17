%% ========================================================================
%  机械臂力-位置混合控制 - 仿真结果分析脚本
%  ========================================================================
%  本脚本用于分析和可视化混合控制系统的仿真结果
%  包括：位置跟踪、力控制、控制输出、性能指标评估
%  ========================================================================

clear variables -except ans out;
close all;
clc;

fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════╗\n');
fprintf('║          仿真结果分析与可视化                        ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');

%% ========================================================================
%  1. 检查仿真数据是否存在
%  ========================================================================
fprintf('▶ 检查仿真数据...\n');

% 检查是否存在输出数据
if ~exist('out', 'var')
    warning('未找到仿真输出！请先运行 hybrid_control_simulink.slx');
    return;
end

% 从Simulink输出中提取数据
if isfield(out, 'time')
    t = out.time;
else
    t = out.tout;
end

% 提取关键信号
try
    x = out.x.Data;           % 位置
    x_d = out.x_d.Data;       % 期望位置
    F_e = out.F_e.Data;       % 环境力反馈
    F_d = out.F_d.Data;       % 期望力
    u_p = out.u_p.Data;       % 位置控制输出
    u_f = out.u_f.Data;       % 力控制输出
    u_total = out.u_total.Data; % 总控制输出
catch
    fprintf('✗ 数据提取失败，请检查Simulink输出信号名称\n');
    return;
end

fprintf('  ✓ 时间范围: [%.3f, %.3f] s\n', t(1), t(end));
fprintf('  ✓ 数据点数: %d\n', length(t));

%% ========================================================================
%  2. 计算性能指标
%  ========================================================================
fprintf('\n▶ 计算控制性能指标...\n');

% 位置跟踪误差
e_pos = x_d - x;
e_pos_mse = sqrt(mean(e_pos.^2));  % 均方根误差
e_pos_max = max(abs(e_pos));        % 最大误差
e_pos_ss = abs(e_pos(end));         % 稳态误差

fprintf('\n  【位置控制性能】\n');
fprintf('    ✓ 位置RMSE: %.6f m\n', e_pos_mse);
fprintf('    ✓ 最大误差: %.6f m\n', e_pos_max);
fprintf('    ✓ 稳态误差: %.6f m\n', e_pos_ss);

% 力控制性能（仅在接触后）
contact_idx = find(F_e > 0.1, 1);  % 接触起始点
if ~isempty(contact_idx)
    e_force = F_d(contact_idx:end) - F_e(contact_idx:end);
    e_force_mse = sqrt(mean(e_force.^2));
    e_force_max = max(abs(e_force));
    
    fprintf('\n  【力控制性能】\n');
    fprintf('    ✓ 力RMSE: %.4f N\n', e_force_mse);
    fprintf('    ✓ 最大误差: %.4f N\n', e_force_max);
else
    fprintf('\n  【力控制性能】\n');
    fprintf('    ⚠ 仿真过程中未产生显著接触\n');
end

% 控制能量
energy_pos = trapz(t, u_p.^2);      % 位置控制能量
energy_force = trapz(t, u_f.^2);    % 力控制能量

fprintf('\n  【控制能量】\n');
fprintf('    ✓ 位置控制能耗: %.2f J\n', energy_pos);
fprintf('    ✓ 力控制能耗: %.2f J\n', energy_force);

%% ========================================================================
%  3. 绘制位置跟踪曲线
%  ========================================================================
fprintf('\n▶ 绘制位置跟踪曲线...\n');

figure('Position', [100, 100, 1400, 900], 'Name', '机械臂混合控制仿真结果');

% 子图1：位置跟踪
subplot(3,3,1);
plot(t, x_d, 'r-', 'LineWidth', 2.5, 'DisplayName', '期望位置 $x_d$');
hold on;
plot(t, x, 'b-', 'LineWidth', 2, 'DisplayName', '实际位置 $x$');
grid on; grid minor;
xlabel('时间 (s)', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('位置 (m)', 'Interpreter', 'latex', 'FontSize', 11);
title('【位置跟踪性能】', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
legend('Interpreter', 'latex', 'FontSize', 10, 'Location', 'best');
set(gca, 'FontSize', 10);

% 子图2：位置误差
subplot(3,3,2);
plot(t, e_pos*1000, 'g-', 'LineWidth', 2);
grid on; grid minor;
xlabel('时间 (s)', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('误差 (mm)', 'Interpreter', 'latex', 'FontSize', 11);
title('【位置跟踪误差】', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 10);
yline(0, 'k--', 'LineWidth', 1);

% 子图3：位置误差直方图
subplot(3,3,3);
histogram(e_pos*1000, 'FaceColor', '#7E57C2', 'EdgeColor', 'black', 'FaceAlpha', 0.7);
xlabel('误差 (mm)', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('频数', 'Interpreter', 'latex', 'FontSize', 11);
title('【位置误差分布】', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 10);

%% ========================================================================
%  4. 绘制力控制曲线
%  ========================================================================
fprintf('  ✓ 绘制力控制曲线...\n');

% 子图4：力控制
subplot(3,3,4);
plot(t, F_d, 'r-', 'LineWidth', 2.5, 'DisplayName', '期望力 $F_d$');
hold on;
plot(t, F_e, 'b-', 'LineWidth', 2, 'DisplayName', '环境反馈力 $F_e$');
grid on; grid minor;
xlabel('时间 (s)', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('力 (N)', 'Interpreter', 'latex', 'FontSize', 11);
title('【力控制性能】', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
legend('Interpreter', 'latex', 'FontSize', 10, 'Location', 'best');
set(gca, 'FontSize', 10);

% 子图5：力误差
subplot(3,3,5);
e_force_all = F_d - F_e;
plot(t, e_force_all, 'g-', 'LineWidth', 2);
grid on; grid minor;
xlabel('时间 (s)', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('误差 (N)', 'Interpreter', 'latex', 'FontSize', 11);
title('【力跟踪误差】', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
yline(0, 'k--', 'LineWidth', 1);
set(gca, 'FontSize', 10);

% 子图6：力响应相位图
subplot(3,3,6);
scatter(F_d, F_e, 20, t, 'filled', 'o');
hold on;
plot([0, max(F_d)], [0, max(F_d)], 'r--', 'LineWidth', 2, 'DisplayName', '理想跟踪');
colorbar; caxis([t(1), t(end)]);
xlabel('期望力 $F_d$ (N)', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('实际力 $F_e$ (N)', 'Interpreter', 'latex', 'FontSize', 11);
title('【力-期望相位轨迹】', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
legend('Interpreter', 'latex', 'FontSize', 10);
set(gca, 'FontSize', 10);

%% ========================================================================
%  5. 绘制控制输出
%  ========================================================================
fprintf('  ✓ 绘制控制输出曲线...\n');

% 子图7：位置控制输出
subplot(3,3,7);
plot(t, u_p, 'b-', 'LineWidth', 2, 'DisplayName', '位置控制 $u_p$');
hold on;
plot(t, u_f, 'r-', 'LineWidth', 2, 'DisplayName', '力控制 $u_f$');
grid on; grid minor;
xlabel('时间 (s)', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('控制输出 (N)', 'Interpreter', 'latex', 'FontSize', 11);
title('【分路控制输出】', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
legend('Interpreter', 'latex', 'FontSize', 10, 'Location', 'best');
set(gca, 'FontSize', 10);

% 子图8：总控制输出
subplot(3,3,8);
plot(t, u_total, 'purple', 'LineWidth', 2.5);
grid on; grid minor;
xlabel('时间 (s)', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('控制输出 (N)', 'Interpreter', 'latex', 'FontSize', 11);
title('【综合控制输出】', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 10);

% 子图9：控制贡献度
subplot(3,3,9);
pos_contribution = (abs(u_p) ./ (abs(u_p) + abs(u_f) + 1e-6)) * 100;
force_contribution = (abs(u_f) ./ (abs(u_p) + abs(u_f) + 1e-6)) * 100;
area(t, [pos_contribution, force_contribution]);
ylabel('贡献度 (%)', 'Interpreter', 'latex', 'FontSize', 11);
xlabel('时间 (s)', 'Interpreter', 'latex', 'FontSize', 11);
title('【控制权重分析】', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
legend({'位置控制', '力控制'}, 'Interpreter', 'latex', 'FontSize', 10);
set(gca, 'FontSize', 10);

% 调整整体布局
sgtitle('机械臂力-位置混合控制仿真分析结果', ...
    'Interpreter', 'latex', 'FontSize', 14, 'FontWeight', 'bold');

% 保存图片
saveas(gcf, 'hybrid_control_results.png');
fprintf('  ✓ 结果图已保存: hybrid_control_results.png\n');

%% ========================================================================
%  6. 绘制额外的分析图表
%  ========================================================================
fprintf('\n▶ 绘制补充分析图表...\n');

% 新建图表2：相图分析
figure('Position', [150, 150, 1000, 700], 'Name', '相图分析');

% 位置相图
subplot(2,2,1);
v = [0; diff(x) / (t(2)-t(1))];  % 速度（数值微分）
plot(x, v, 'b-', 'LineWidth', 2);
hold on;
plot(x(1), v(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '初始状态');
plot(x(end), v(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', '末态');
grid on;
xlabel('位置 $x$ (m)', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('速度 $\dot{x}$ (m/s)', 'Interpreter', 'latex', 'FontSize', 11);
title('【位置相图】', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
legend('Interpreter', 'latex', 'FontSize', 10);

% 力响应曲线
subplot(2,2,2);
semilogy(t, abs(e_force_all) + 1e-6, 'b-', 'LineWidth', 2);
grid on; grid minor;
xlabel('时间 (s)', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('力误差 |$e_F$| (N)', 'Interpreter', 'latex', 'FontSize', 11);
title('【力误差对数曲线】', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 10);

% 控制输出频谱（可选）
subplot(2,2,3);
if length(t) > 100
    fs = 1/(t(2)-t(1));  % 采样频率
    [Pxx, f] = pwelch(u_total, [], [], [], fs);
    loglog(f, Pxx, 'b-', 'LineWidth', 1.5);
    grid on; grid minor;
    xlabel('频率 (Hz)', 'Interpreter', 'latex', 'FontSize', 11);
    ylabel('功率谱密度', 'Interpreter', 'latex', 'FontSize', 11);
    title('【控制输出频谱】', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
end

% 累积误差
subplot(2,2,4);
cumulative_pos_error = cumsum(abs(e_pos)) * (t(2)-t(1));
plot(t, cumulative_pos_error, 'b-', 'LineWidth', 2);
grid on;
xlabel('时间 (s)', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('累积误差 (m·s)', 'Interpreter', 'latex', 'FontSize', 11);
title('【累积位置误差】', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 10);

sgtitle('补充分析图表', 'Interpreter', 'latex', 'FontSize', 14, 'FontWeight', 'bold');

%% ========================================================================
%  7. 生成性能报告
%  ========================================================================
fprintf('\n╔═══════════════════════════════════════════════════════╗\n');
fprintf('║                  性能评估报告                        ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');

fprintf('【位置跟踪性能评估】\n');
fprintf('  • 均方根误差 (RMSE): %.6f m\n', e_pos_mse);
fprintf('  • 最大误差: %.6f m\n', e_pos_max);
fprintf('  • 稳态误差: %.6f m\n', e_pos_ss);
fprintf('  • 性能等级: ');
if e_pos_mse < 1e-4
    fprintf('优秀 ★★★★★\n');
elseif e_pos_mse < 1e-3
    fprintf('良好 ★★★★☆\n');
elseif e_pos_mse < 1e-2
    fprintf('一般 ★★★☆☆\n');
else
    fprintf('需改进 ★★☆☆☆\n');
end

if ~isempty(contact_idx)
    fprintf('\n【力控制性能评估】\n');
    fprintf('  • 均方根误差 (RMSE): %.4f N\n', e_force_mse);
    fprintf('  • 最大误差: %.4f N\n', e_force_max);
    fprintf('  • 性能等级: ');
    if e_force_mse < 0.1
        fprintf('优秀 ★★★★★\n');
    elseif e_force_mse < 0.5
        fprintf('良好 ★★★★☆\n');
    elseif e_force_mse < 1
        fprintf('一般 ★★★☆☆\n');
    else
        fprintf('需改进 ★★☆☆☆\n');
    end
end

fprintf('\n【控制效率评估】\n');
fprintf('  • 位置控制能耗: %.2f J (%.1f%%)\n', ...
    energy_pos, energy_pos/(energy_pos+energy_force+1e-6)*100);
fprintf('  • 力控制能耗: %.2f J (%.1f%%)\n', ...
    energy_force, energy_force/(energy_pos+energy_force+1e-6)*100);
fprintf('  • 总能耗: %.2f J\n', energy_pos + energy_force);

fprintf('\n╔═══════════════════════════════════════════════════════╗\n');
fprintf('║                分析完成！                            ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');

%% ========================================================================
% END OF analyze_results.m
%% ========================================================================
