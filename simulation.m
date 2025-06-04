% --- 系统参数 ---
v_r = 0.4; % 参考线速度 (m/s)
omega_r = 0.5; % 参考角速度 (rad/s)

% --- (1) 状态空间模型 ---
A = [0, omega_r, 0;
     -omega_r, 0, v_r;
     0, 0, 0];
B = [-1, 0;
     0, 0;
     0, -1];

disp('系统矩阵 A:'); disp(A);
disp('输入矩阵 B:'); disp(B);

% 检查系统可控性
Co = ctrb(A, B);
rank_Co = rank(Co);
if rank_Co < size(A,1)
    disp('系统不可控，极点配置可能失败或存在问题。');
    % 注意：对于这个特定的A, B矩阵，由于B(2,1)和B(2,2)为0，
    % y_e 的动态特性主要由A矩阵决定，直接控制能力较弱，
    % 但通过x_e和theta_e的耦合仍可间接影响。
    % 对于这个系统，rank(ctrb(A,B)) = 3，所以是可控的。
else
    disp('系统可控。');
end

% --- (2) 极点配置控制器设计 ---
% 性能指标
Mp_spec = 0.04; % 超调量 <= 4%
ts_spec = 5;    % 调节时间 (2%准则) <= 5s

% 根据性能指标计算期望的阻尼比 zeta 和自然频率 omega_n
zeta = sqrt(log(Mp_spec)^2 / (pi^2 + log(Mp_spec)^2));
% zeta = 0.7156，取 zeta = 0.72
if zeta < 0.7156 % 确保满足超调量
    zeta = 0.72;
end
omega_n_min = 4 / (zeta * ts_spec);
% omega_n_min = 4 / (0.72 * 5) = 4 / 3.6 = 1.111
% 取 omega_n = 1.2 rad/s
omega_n = 1.2; 
if omega_n < omega_n_min
    omega_n = omega_n_min * 1.1; % 留些余量
end

fprintf('设计参数: zeta = %f, omega_n = %f rad/s\n', zeta, omega_n);

P_dominant_1 = -zeta*omega_n + 1j*omega_n*sqrt(1-zeta^2);
P_dominant_2 = -zeta*omega_n - 1j*omega_n*sqrt(1-zeta^2);
P_third = -5 * zeta * omega_n; % 第三极点，比主导极点快5倍

P_desired = [P_dominant_1, P_dominant_2, P_third];
disp('期望配置的极点:'); disp(P_desired);

K_pp = place(A, B, P_desired);
disp('极点配置控制器增益 K_pp:'); disp(K_pp);

% 闭环系统 (极点配置)
A_cl_pp = A - B * K_pp;
sys_cl_pp = ss(A_cl_pp, zeros(3,1), eye(3), 0); % B_dummy=zeros(3,1) 用于观察初始响应

disp('极点配置后闭环系统 A-BK_pp 的特征值:'); disp(eig(A_cl_pp));

% --- (3) LQR 控制器设计 ---
% 权重矩阵 Q 和 R (可调参数)
Q = diag([10, 10, 1]); % 惩罚 x_e, y_e 误差较大, theta_e 误差较小
% 或者 Q = diag([1/(0.1^2), 1/(0.1^2), 1/(0.1^2)]); % 假设允许误差幅度
R = diag([0.1, 0.1]);   % 允许一定的控制输入量
% 或者 R = diag([1/(0.5^2), 1/(0.5^2)]); % 假设允许控制输入幅度

[K_lqr, S_lqr, E_lqr] = lqr(A, B, Q, R);
disp('LQR 控制器增益 K_lqr:'); disp(K_lqr);

% 闭环系统 (LQR)
A_cl_lqr = A - B * K_lqr;
sys_cl_lqr = ss(A_cl_lqr, zeros(3,1), eye(3), 0);

disp('LQR 控制器闭环系统 A-BK_lqr 的特征值 (E_lqr):'); disp(E_lqr);

% --- 仿真 ---
t_sim = 0:0.05:10; % 仿真时间
p_e0 = [0.5; 0.3; 0.2]; % 初始误差 [x_e(0); y_e(0); theta_e(0)] (例如: 0.5m, 0.3m, 0.2rad)

% 初始响应
[y_pp, t_pp, x_pp] = initial(sys_cl_pp, p_e0, t_sim); % x_pp 是状态轨迹
[y_lqr, t_lqr, x_lqr] = initial(sys_cl_lqr, p_e0, t_sim); % x_lqr 是状态轨迹

% --- 绘制响应曲线 ---
figure;
sgtitle('跟踪误差响应曲线 (初始误差: x_e=0.5, y_e=0.3, \theta_e=0.2)');

subplot(3,1,1);
plot(t_pp, x_pp(:,1), 'r-', 'LineWidth', 1.5); hold on;
plot(t_lqr, x_lqr(:,1), 'b--', 'LineWidth', 1.5);
ylabel('x_e (m)');
legend('极点配置', 'LQR');
title('纵向误差 x_e');
grid on;

subplot(3,1,2);
plot(t_pp, x_pp(:,2), 'r-', 'LineWidth', 1.5); hold on;
plot(t_lqr, x_lqr(:,2), 'b--', 'LineWidth', 1.5);
ylabel('y_e (m)');
legend('极点配置', 'LQR');
title('横向误差 y_e');
grid on;

subplot(3,1,3);
plot(t_pp, x_pp(:,3), 'r-', 'LineWidth', 1.5); hold on;
plot(t_lqr, x_lqr(:,3), 'b--', 'LineWidth', 1.5);
ylabel('\theta_e (rad)');
xlabel('时间 (s)');
legend('极点配置', 'LQR');
title('方向误差 \theta_e');
grid on;

% --- 性能分析 (主要针对极点配置，因为其设计目标明确) ---
disp('--- 极点配置控制器性能分析 ---');
% 对于 x_e
info_xe_pp = stepinfo(x_pp(:,1), t_pp, 0, 'SettlingTimeThreshold', 0.02); % Target final value is 0
% stepinfo 的超调量是针对从0到某个值的阶跃响应，或者从某个值到0的衰减。
% 对于初始条件响应，如果从 p_e0(1) 衰减到 0:
% 超调量定义为响应穿过0点后在反向达到的峰值 与 |p_e0(1)| 的比率。
% 如果不穿过0点，则超调量为0。

% x_e 性能
initial_xe = p_e0(1);
ts_xe_pp = NaN;
overshoot_xe_pp = 0;
% 调节时间 (2% 准则)
settling_threshold_xe = 0.02 * abs(initial_xe);
for k = 1:length(t_pp)
    if all(abs(x_pp(k:end,1)) <= settling_threshold_xe)
        ts_xe_pp = t_pp(k);
        break;
    end
end
% 超调量
if initial_xe > 0 && min(x_pp(:,1)) < -eps % 从正值开始，穿过0到负值
    overshoot_xe_pp = abs(min(x_pp(:,1))) / initial_xe * 100;
elseif initial_xe < 0 && max(x_pp(:,1)) > eps % 从负值开始，穿过0到正值
    overshoot_xe_pp = abs(max(x_pp(:,1))) / abs(initial_xe) * 100;
end
fprintf('x_e (极点配置): 超调量 ≈ %.2f%%, 调节时间 (2%%) ≈ %.2f s\n', overshoot_xe_pp, ts_xe_pp);
if overshoot_xe_pp <= Mp_spec*100 && ts_xe_pp <= ts_spec
    fprintf('x_e 响应满足设计指标。\n');
else
    fprintf('x_e 响应未能完全满足设计指标。可能需要调整期望极点。\n');
end

% 对 y_e 和 theta_e 进行类似分析，但通常性能指标是针对系统整体主导行为。
% 这里主要关注x_e的指标，因为它是直接被u_v控制的，而y_e主要通过耦合影响。
% 实际应用中，可能需要综合评估所有状态的响应。
