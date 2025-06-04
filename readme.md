
# pole assignment & lqr tracking simulation for modern control theory

参见[博客](https://symc.wang/2025/06/04/pole-assignment-lqr-tracking-simulation-for-modern-control-theory/)。
---

## 0 题目要求

假设在惯性坐标系下移动机器人的坐标为$O=(x,y,θ)^T$，参考机器人的坐标为$q_r=(x_r,y_r,θ_r)^T$，跟踪误差坐标在载体坐标系下的坐标为$p_e=(x_e,y_e,θ_e)^T$。  

（1）通过载体坐标系旋转变换以及图1的几何关系试建立移动机器人跟踪误差线性状态空间模型。  

（2）如果参考机器人轨迹是一个半径$2$米的圆，参考线速度为$0.4$米/秒，参考角速度为$0.5$弧度/秒，试设计一个稳定化的状态反馈控制器，使得系统的超调量$\le 4 \%$，调节时间$\le5$秒，并用MATLAB给出响应曲线(按$2\%$准则)。  

（3）在相同的参数下设计LQR控制器，并用MATLAB给出响应曲线。  

![图1](/img/model.png)

## 1 推导

### 1.1 定义坐标与误差

实际机器人在世界坐标系下的状态：  

$$
q = \begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
$$

参考轨迹的状态：  

$$
q_r = \begin{bmatrix}
x_r \\
y_r \\
\theta_r
\end{bmatrix}
$$

跟踪误差坐标在参考机器人坐标系下定义为：  

$$
p_e = \begin{bmatrix}
x_e \\
y_e \\
\theta_e
\end{bmatrix} = \begin{bmatrix}
x - x_r \\
y - y_r \\
\theta - \theta_r
\end{bmatrix}
$$

### 1.2 坐标变换推导

利用旋转矩阵：
$$
R_r = \begin{bmatrix}
\cos\theta_r & \sin\theta_r & 0 \\
-\sin\theta_r & \cos\theta_r & 0 \\
0 & 0 & 1
\end{bmatrix}
$$
可以将实际机器人坐标系下的状态转换到参考机器人坐标系下的跟踪误差坐标：  

$$
p_e = \begin{bmatrix}
x_e \\
y_e \\
\theta_e
\end{bmatrix} = \begin{bmatrix}
cos\theta_r(x - x_r) + sin\theta_r(y - y_r)\\
-sin\theta_r(x - x_r) + cos\theta_r(y - y_r) \\
\theta - \theta_r
\end{bmatrix}
$$

### 1.3 机器人运动学模型

定义机器人人动力学模型。  

定义实际机器人状态：  

$$
\left\{
\begin{aligned}
\dot{x} &= v \cos\theta \\
\dot{y} &= v \sin\theta \\
\dot{\theta} &= \omega
\end{aligned}
\right.
$$

定义参考机器人状态：  

$$
\left\{
\begin{aligned}
\dot{x}_r &= v_r \cos\theta_r \\
\dot{y}_r &= v_r \sin\theta_r \\
\dot{\theta}_r &= \omega_r
\end{aligned}
\right.
$$

### 1.4 误差状态空间模型

对$x_e$，$y_e$，$\theta_e$分别求导。  

对于$x_e$：  

$$
\begin{aligned}
\dot{x}_e &= \frac{d}{dt}[cos\theta_r(x - x_r) + sin\theta_r(y - y_r)] \\
&= -sin\theta_r \dot{\theta}_r (x - x_r) + cos\theta_r (\dot{x} - \dot{x}_r) + cos\theta_r \dot{\theta}_r (y - y_r) + sin\theta_r (\dot{y} - \dot{y}_r) \\
\end{aligned}
$$

将导数带入得到：  

$$
\dot{x}_e = -sin\theta_r \omega_r (x - x_r) + cos\theta_r (v \cos\theta - v_r \cos\theta_r) + cos\theta_r \omega_r (y - y_r) + sin\theta_r (v \sin\theta - v_r \sin\theta_r)
$$

我们做小误差角度近似：假设$\theta = \theta_r + \theta_e$，并且在$\theta_e$较小时展开：  

$$
\begin{aligned}
cos\theta &\approx cos\theta_r - \theta_e \sin\theta_r \\
sin\theta &\approx sin\theta_r + \theta_e cos\theta_r
\end{aligned}
$$

带入$x_e$的导数中，省略高阶项并化简，得到：  

$$
\dot{x}_e = -v_r + v cos\theta_r + \omega_r y_e
$$

同理，对于$y_e$：  

$$
\begin{aligned}
\dot{y}_e &= \frac{d}{dt}[-sin\theta_r(x - x_r) + cos\theta_r(y - y_r)] \\
&= -cos\theta_r \dot{\theta}_r (x - x_r) - sin\theta_r (\dot{x} - \dot{x}_r) + sin\theta_r \dot{\theta}_r (y - y_r) + cos\theta_r (\dot{y} - \dot{y}_r)
\end{aligned}
$$

带入并化简，得到：  

$$
\dot{y}_e = v sin\theta_e - \omega_r x_e
$$

对于$\theta_e$，显然：  

$$
\dot{\theta}_e = \dot{\theta} - \dot{\theta}_r = \omega - \omega_r
$$

最终得到非线性误差模型：  

$$
\left\{
\begin{aligned}
\dot{x}_e &= -v_r + v cos\theta_r + \omega_r y_e \\
\dot{y}_e &= v sin\theta_e - \omega_r x_e \\
\dot{\theta}_e &= \omega - \omega_r
\end{aligned}
\right.
$$

### 1.5 线性化误差模型

当误差很小时，使用小角度近似：  

$$
\begin{aligned}
sin\theta_e &\approx \theta_e \\
cos\theta_e &\approx 1
\end{aligned}
$$

将其代入上式，得到线性化的误差状态空间模型：  
$$
\left\{
\begin{aligned}
\dot{x}_e &= -v_r + v + \omega_r y_e \\
\dot{y}_e &= v \theta_e - \omega_r x_e \\
\dot{\theta}_e &= \omega - \omega_r
\end{aligned}
\right.
$$

设误差状态：  

$$
X_e = \begin{bmatrix}
x_e \\
y_e \\
\theta_e
\end{bmatrix}
$$

控制输入误差：  

$$
U_e = \begin{bmatrix}
v - v_r \\
\omega - \omega_r
\end{bmatrix}
$$

得到线性化的误差状态空间模型：  

$$
\dot{X}_e = A X_e + B U_e
$$

其中，状态矩阵$A$和输入矩阵$B$为：  

$$
A = \begin{bmatrix}
0 & \omega_r & 0 \\
-\omega_r & 0 & v_r \\
0 & 0 & 0
\end{bmatrix}, \quad B = \begin{bmatrix}
1 & 0 \\
0 & 0 \\
0 & 1
\end{bmatrix}
$$

## 2 控制器设计

### 2.1 极点配置

极点配置方法是通过选择合适的状态反馈增益矩阵$K$来将系统的极点移动到期望的位置。  

#### 2.1.1 可控性检验

首先检查系统的可控性。构造可控性矩阵$C$：
$$
C = \begin{bmatrix}
B & AB & A^2B
\end{bmatrix} \in \mathbb{R}^{3 \times 6}
\Rightarrow rank(C) = 3 \Rightarrow 可控
$$

在mATLAB中可以使用`ctrb`函数验证可控性：  

```matlab
% 定义状态矩阵A和输入矩阵B
A = [0 0.5 0; -0.5 0 0.4; 0 0 0];
B = [1 0; 0 0; 0 1];
% 计算可控性矩阵
C = ctrb(A, B);
% 检查可控性
if rank(C) == size(A, 1)
    disp('系统可控');
else
    disp('系统不可控');
end
```

#### 2.1.2 性能指标转化

设计要求：  
- 超调量$M_p \leq 4\%$  
- 调节时间$t_s \leq 5$秒  

$$
M_p = e^{-\frac{\zeta \pi}{\sqrt{1 - \zeta^2}}} = 0.04 \Rightarrow \zeta \approx 0.7156 \Rightarrow \zeta = 0.72
$$

调节时间满足$2\%$准则：  

$$
t_s = \frac{4}{\zeta \omega_n} \leq 5 \Rightarrow \omega_n \geq \frac{4}{\zeta t_s} = \frac{4}{0.72 \times 5} \approx 1.1111 \Rightarrow \omega_n = 1.2
$$

计算目标极点：  

$$
s_{1,2} = -\zeta \omega_n \pm j \omega_n \sqrt{1 - \zeta^2} = -0.864 + j 0.847
$$

设第3个极点为实数且收敛更快：  

$$
s_3 = -5 \zeta \omega_n = -4.32
$$

所以期望极点为：  

$$
s = \{-0.864 + j 0.847, -0.864 - j 0.847, -4.32\}
$$

#### 2.1.3 求解增益矩阵

给定参考速度$v_r = 0.4$m/s，参考角速度$\omega_r = 0.5$rad/s，得到状态矩阵$A$和输入矩阵$B$：  

$$
A = \begin{bmatrix}
0 & 0.5 & 0 \\
-0.5 & 0 & 0.4 \\
0 & 0 & 0
\end{bmatrix}, \quad B = \begin{bmatrix}
1 & 0 \\
0 & 0 \\
0 & 1
\end{bmatrix}
$$

使用MATLAB的`place`函数求解增益矩阵$K$：  

```matlab
% 定义状态矩阵A和输入矩阵B
A = [0 0.5 0; -0.5 0 0.4; 0 0 0];
B = [1 0; 0 0; 0 1];
% 期望极点
desired_poles = [-0.864 + 0.847i, -0.864 - 0.847i, -4.32];
% 使用place函数计算增益矩阵K
K = place(A, B, desired_poles);
% 输出增益矩阵K
disp('增益矩阵K:');
disp(K);
```

### 2.2 LQR控制器设计

LQR控制器设计是通过最小化状态和控制输入的加权二次型来获得最优控制增益矩阵$K_{lqr}$。  

LQR求解最优控制律：  

$$
u = -K_{lqr} X_e
$$

通过代价函数：  

$$
J = \int_0^\infty (X_e^T Q X_e + U_e^T R U_e) dt
$$

设计目标：  
- 惩罚$x_e$，$y_e$偏差 $\rightarrow$ Q权重大  
- 控制输入不宜过大 $\rightarrow$ R不能过小  


#### 2.2.1 定义权重矩阵

选择常用状态权重矩阵$Q$和控制输入权重矩阵$R$：
$$
Q = \begin{bmatrix}
10 & 0 & 0 \\
0 & 10 & 0 \\
0 & 0 & 1
\end{bmatrix}, \quad R = \begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix}
$$

#### 2.2.2 求解LQR增益矩阵
使用MATLAB的`lqr`函数求解LQR增益矩阵$K_{LQR}$：  

```matlab
% 定义状态矩阵A和输入矩阵B
A = [0 0.5 0; -0.5 0 0.4; 0 0 0];
B = [1 0; 0 0; 0 1];
% 定义权重矩阵Q和R
Q = [10 0 0; 0 10 0; 0 0 1];
R = [1 0; 0 1];
% 使用lqr函数计算LQR增益矩阵K_LQR
K_LQR = lqr(A, B, Q, R);
% 输出LQR增益矩阵K_LQR
disp('LQR增益矩阵K_LQR:');
disp(K_LQR);
```
而在c++中，需要求解Algebraic Riccati Equation (ARE)来得到LQR增益矩阵$K_{LQR}$：  

```cpp
MatrixXd SolveLQR(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R){
    // solve Algebraic Riccati equation: A'P + PA - PBR^(-1)B'P + Q = 0
    MatrixXd P = Q;
    for(int i = 0; i < 100; i++){
        MatrixXd P_next = Q + A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
        if((P_next - P).norm() < 1e-6){
            break;
        }
        cout<<"Iteration " << i << ": P norm = " << (P_next - P).norm() << endl;
        P = P_next;
    }
    // optimal gain matrix K = R^(-1)B'P
    return (R + B.transpose() * P * B).inverse() * B.transpose() * P;
}
```

得到控制增益矩阵$K_{lqr}$后，可以将其应用于控制输入：  

$$
A_{lqr} = A - B K_{lqr}
$$

## 3 仿真

### 3.1 matlab仿真

基于上述推导和设计，使用MATLAB进行仿真。  

```matlab
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
```

### 3.2 c++仿真

基于opencv和Eigen库，使用c++简单实现了LQR控制器的仿真。  

```cpp
#include <iostream>
#include "Eigen/Dense"
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;
using namespace Eigen;

constexpr  double v_r = 0.4; // reference linear velocity
constexpr  double w_r = 0.5; // reference angular velocity
constexpr  double radius = 2.0; // radius of the circle
constexpr double dt = 0.01; // time step
constexpr double sim_time = 10.0; // total simulation time
constexpr int steps = static_cast<int>(sim_time / dt); // number of steps

MatrixXd SolveLQR(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R){
    // solve Algebraic Riccati equation: A'P + PA - PBR^(-1)B'P + Q = 0
    MatrixXd P = Q;
    for(int i = 0; i < 100; i++){
        MatrixXd P_next = Q + A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
        if((P_next - P).norm() < 1e-6){
            break;
        }
        cout<<"Iteration " << i << ": P norm = " << (P_next - P).norm() << endl;
        P = P_next;
    }
    // optimal gain matrix K = R^(-1)B'P
    return (R + B.transpose() * P * B).inverse() * B.transpose() * P;
}

int main(){
    // state space model
    Matrix3d A;
    A << 0, w_r, 0,
         -w_r, 0, v_r,
         0, 0, 0;
    MatrixXd B(3, 2);
    B << 1, 0,
         0, 0,
         0, 1;

    // LQR parameters
    Matrix3d Q;
    Q << 10, 0, 0,
         0, 10, 0,
         0, 0, 5; // state cost
    Matrix2d R;
    R << 1, 0,
         0, 1; // control cost
    Matrix<double, 2, 3> K = SolveLQR(A, B, Q, R);
    cout<<"K matrix:\n" << K << endl;
    // state initialization
    Vector3d x_e(0.5, 0.2, 0.1); // initial error state
    vector<cv::Point> trajectory, reference_trajectory;

    // graphics setup
    int size = 600;
    int origin = size >> 1;
    double scale = 100.0; // 1m = 100 pixels
    Mat canvas(size, size, CV_8UC3, Scalar(255, 255, 255));

    // simulation loop
    for(int i = 0; i < steps; i++) {
        double t = i * dt;
        double x_r = radius * cos(w_r * t);
        double y_r = radius * sin(w_r * t);
        double theta_r = w_r * t;

        // control law
        Vector2d u = -K * x_e;

        // update error state
        Vector3d dx_e = A * x_e + B * u;
        x_e += dx_e * dt;

        // convert error to global coordinates
        Matrix2d R_theta;
        R_theta << cos(theta_r), -sin(theta_r),
                sin(theta_r), cos(theta_r);
        Vector2d pe = R_theta * x_e.head<2>();

        double x = x_r + pe(0);
        double y = y_r + pe(1);

        // store trajectory points
        cv::Point p(static_cast<int>(origin + x * scale), static_cast<int>(origin - y * scale));
        trajectory.push_back(p);
        reference_trajectory.emplace_back(static_cast<int>(origin + x_r * scale),
                                          static_cast<int>(origin - y_r * scale));
    }
    // draw trajectory
    canvas = Scalar(255, 255, 255); // clear canvas
    for(int i = 0; i < trajectory.size(); i++){
        // draw start point
        if(i == 0) {
            circle(canvas, trajectory[i], 5, Scalar(0, 255, 0), -1); // start point in green
        }
        if(i == trajectory.size() - 1) {
            circle(canvas, trajectory[i], 5, Scalar(0, 0, 255), -1); // end point in red
        }
        circle(canvas, trajectory[i], 1, Scalar(211, 85, 186), -1);
        circle(canvas, reference_trajectory[i], 1, Scalar(255, 0, 0), -1);

        if(i != 0 && i != trajectory.size() - 1 && i % 20 == 0)
            circle(canvas, trajectory[i], 5, Scalar(128, 128, 240), 1); // draw current position

        // display the canvas
        imshow("LQR Trajectory", canvas);
        char key = (char)waitKey(1);
        if(key == 27) { // ESC key to exit
            break;
        }
    }

    // wait for a key press before closing
    waitKey(0);
    return 0;
}
```

### 3.3 仿真结果

![matlab仿真结果](/img/matlab.png)  
![matlab输出结果](/img/matlab_output.png)  
![c++仿真结果](/img/cpp.png)  

## 4 总结

本文通过推导移动机器人跟踪误差的线性状态空间模型，设计了极点配置和LQR控制器，并在MATLAB和C++中进行了仿真。结果表明，所设计的控制器能够有效地跟踪参考轨迹，满足超调量和调节时间的要求。极点配置方法和LQR方法都能实现稳定的跟踪控制，但LQR方法在处理多变量系统时更为灵活和高效。  

代码见[github](https://github.com/symcreg/lqr_simulation)，仿真视频见[bilibili](https://www.bilibili.com/video/BV1stTjz9EXX)。
