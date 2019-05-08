## 引言

对于一个状态估计问题，我们通常需要提供运动模型，观测模型，估计误差模型：
$$
\dot {\mathbf x_t}=f(\mathbf x_t) \\
\mathbf z = g(\mathbf x_t) + \mathbf n \\
\delta \mathbf x = e(\mathbf x, \mathbf x_t)
$$
其中$\mathbf x_t$表示真实状态量（True state, 我们永远得不到），$\mathbf x$表示估计状态量（Nominal state），$\delta \mathbf x$表示误差状态量（Error state）， $\mathbf z$表示观测值，$\mathbf n$表示观测噪声。

对于轮式编码器而言，其状态可以表示为：$\mathbf x=[\mathbf p,\theta]$， 其中$\mathbf p = [x,y]$为机器人在世界参考坐标系中的二维位置，$\theta$为机器人的参考坐标系中的绝对朝向。

## 轮式编码器运动模型

### 真实状态量和估计状态量运动模型

一般地面机器人都会配备有轮式编码器。轮式编码器的输出测量值为机器人的瞬时线速度$v_m$，瞬时角速度$\omega_m$，需要注意的是这两者都是相对于轮式编码器的Frenet-Serret坐标系而言的。轮式编码器的测量值包含了真实测量值和测量噪声：
$$
v_m = v_t + n_v \\
\omega_m = \omega_t + n_\omega
$$
其中$v_t$和$\omega_t$为真实测量值，$n$为高斯白噪声, $n_v\sim \mathcal N(0,\sigma_v^2), n_\omega \sim \mathcal N(0, \sigma_\omega^2)$。

因此机器人在载体坐标系下的真实速度为$\mathbf v_t=[v_t, 0]^T$, $\mathbf v_m = \mathbf v_t + \mathbf n_v$. 

我们可以通过旋转矩阵$\mathbf R_t$将其变换到世界参考坐标系中:
$$
\mathbf R_t = 
\begin{bmatrix}
\cos(\theta_t) & -\sin(\theta_t) \\
\sin(\theta_t) & \cos(\theta_t)
\end{bmatrix}
$$
因此，我们可以得到
$$
\dot {\mathbf p_t} = \mathbf R_t \mathbf v_t =
\begin{bmatrix}
\cos(\theta_t) & -\sin(\theta_t) \\
\sin(\theta_t) & \cos(\theta_t)
\end{bmatrix} 
\begin{bmatrix}
v \\ 0
\end{bmatrix}
= 
\begin{bmatrix}
\cos(\theta_t)v_t \\
\sin(\theta_t)v_t
\end{bmatrix}
$$
另外显而易见，对于平面旋转，
$$
\dot \theta_t = \omega_t
$$
因此，我们得到轮式编码器的真实状态量运动模型为：
$$
\begin{align}
\dot x_t &= \cos(\theta_t)v_t \\
\dot y_t &= \sin(\theta_t)v_t \\
\dot \theta_t &= \omega_t
\end{align}
$$
显然，估计状态量的运动模型为：
$$
\begin{align}
\dot x &= \cos(\theta)v_m \\
\dot y &= \sin(\theta)v_m \\
\dot \theta &= \omega_m
\end{align}
$$

### 误差状态量运动模型

我们的目标是得到误差状态量运动模型。

对于旋转矩阵，我们首先有以下近似：
$$
\mathbf R_t = \mathbf R(\mathbf I + [\delta \theta]_\times)+O(\Vert \delta \theta\Vert^2)
$$
其中负对称运算符$[\cdot ]_\times$产生负对称矩阵:
$$
[\mathbf a]_\times = 
\begin{bmatrix}
0 & -a_z & a_y \\
a_z & 0 & -a_x \\
-a_y & a_x & 0
\end{bmatrix}
$$
且有性质：
$$
[\mathbf a]_\times\mathbf b = -[\mathbf b]_\times\mathbf a 
$$


对于位置而言，有：$\dot {\delta \mathbf p} = \dot {\mathbf p_t} - \dot {\mathbf p}$,

我们有
$$
\begin{align}
\dot {\mathbf p_t}&=\mathbf R_t \mathbf v_t  \\
&= \mathbf R(\mathbf I + [\delta \theta]_\times)(\mathbf v_m - \mathbf n_v) \\
&= \mathbf R\mathbf v_m - \mathbf R \mathbf n_v + \mathbf R [\delta \theta]_\times \mathbf v_m - \mathbf R[\delta \theta]_\times \mathbf n_v \\
&= \mathbf R\mathbf v_m + \mathbf R [\delta \theta]_\times \mathbf v_m - \mathbf R\mathbf n_v
\end{align}
$$
其中，$\mathbf R[\delta \theta]_\times \mathbf n_v $是二阶小项，略去.

所以，
$$
\begin{align}
\dot {\delta \mathbf p} 
&= \dot {\mathbf p_t} - \dot {\mathbf p} \\
&= \mathbf R\mathbf v_m + \mathbf R [\delta \theta]_\times \mathbf v_m - \mathbf R\mathbf n_v - \mathbf R\mathbf v_m \\
&= -\mathbf R [\mathbf v_m]_\times \delta\theta - \mathbf R\mathbf n_v \\
&= 
-\begin{bmatrix}
\cos(\theta) & -\sin(\theta) & 0 \\
\sin(\theta) & \cos(\theta) & 0 \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & -v_m \\
0 & v_m & 0
\end{bmatrix}
\begin{bmatrix}
0 \\ 0 \\ \delta\theta
\end{bmatrix} - 
\mathbf R\mathbf n_v\\
&= \begin{bmatrix}
-\sin(\theta)v_m \delta \theta\\
\cos(\theta)v_m \delta \theta\\
0
\end{bmatrix}  
- 
\begin{bmatrix}
\cos(\theta)n_v \\ \sin(\theta)n_v \\ 0
\end{bmatrix} 
\end{align}
$$
对于朝向角，有$\dot {\delta \mathbf \theta} = \dot {\mathbf \theta_t} - \dot {\mathbf \theta}=\omega_t - \omega_m = -n_\omega$ .

所以，二维情况下的误差状态量运动模型为：
$$
\begin{align}
\dot {\delta x} &= -\sin(\theta)v_m\delta \theta + \cos(\theta)n_v \\
\dot {\delta y} &= \cos(\theta)v_m\delta \theta + \sin(\theta)n_v \\
\dot {\delta \theta} &= n_\omega
\end{align}
$$
（-变为＋是因为n为零均值白噪声）

## 离散时间运动模型

上一节我们得到了轮式编码器连续时间下的运动模型，上述微分方程需要在离散时间区间$\Delta t>0$内进行积分来得到对应的差分方程。对于误差状态卡尔曼滤波器来说，我们要做两个方面的积分：1. 估计状态量的积分； 2. 误差状态量的积分，包括确定性变量和随机变量。

### 估计状态量运动模型

估计状态量差分方程如下：
$$
\begin{align}
x &\leftarrow x + \cos(\theta)v_m\Delta t \\
y &\leftarrow y + \sin(\theta)v_m \Delta t \\
\theta &\leftarrow \theta + \omega_m\Delta t
\end{align}
$$
其中$x\leftarrow f(x,\cdot)$ 表示对状态量的更新：$x_{k+1}=f(x_k,\cdot)$.

上述我们用到了一阶欧拉积分，更精细的积分方法可以采用中点积分法等。

### 误差状态量运动模型

误差状态量中的确定性变量正常积分，随机变量积分产生随机脉冲，因此差分方程如下:
$$
\begin{align}
\delta x &\leftarrow \delta x - \sin(\theta)v_m\delta\theta \Delta t + v_{x,i}\\
\delta y &\leftarrow \delta y + \cos(\theta)v_m\delta\theta \Delta t + v_{y,i}\\
\delta \theta &\leftarrow \delta \theta + \omega_i
\end{align}
$$
其中$v_i,\omega_i$是随机脉冲，为高斯白噪声，其均值为0，方差为$\cos(\theta)^2\sigma_v^2\Delta t^2,\sin(\theta)^2\sigma_v^2\Delta t^2, \sigma_\omega^2\Delta t^2$ .

### 雅可比矩阵

由上述差分方程，我们可以写出雅可比矩阵。估计误差模型可以写为：
$$
\delta \mathbf x \leftarrow f(\mathbf x, \delta \mathbf x, \mathbf u_m, \mathbf i)
= \mathbf F_{\mathbf x}(\mathbf x, \mathbf u_m)\cdot \delta \mathbf x + \mathbf F_i \cdot \mathbf i
$$
其中$\mathbf u_m$为输入控制量，$\mathbf i$为扰动量，
$$
\mathbf u_m=
\begin{bmatrix}
v_m \\ 0 \\ \omega_m
\end{bmatrix}，
\mathbf i = 
\begin{bmatrix}
v_{x,i} \\ v_{y,i} \\ \omega_i
\end{bmatrix}
$$
因此，ESKF的更新方程可以写为：
$$
\begin{align}
\hat {\delta \mathbf x} &\leftarrow \mathbf F_{\mathbf x}(\mathbf x,\mathbf u_m)\cdot \hat{\delta \mathbf x} \\
\mathbf P &\leftarrow \mathbf F_{\mathbf x}\mathbf P\mathbf F_{\mathbf x}^T + \mathbf F_{\mathbf i}\mathbf Q_{\mathbf i}\mathbf F_{\mathbf i}^T
\end{align}
$$
其中，$\delta \mathbf x \sim \mathcal N(\hat{\delta \mathbf x}, \mathbf P)$, $\mathbf F_{\mathbf x}$和$\mathbf F_{\mathbf i}$为$f()$相对于误差状态量和扰动量的雅可比矩阵。

那么如何得到雅可比的具体形式呢？

根据定义，
$$
\mathbf F_{\mathbf x} = \frac {\partial f} {\partial \delta \mathbf x}\vert_{\mathbf x,\mathbf u_m} = 
\begin{bmatrix}
1 & 0 &-\sin(\theta)v_m\Delta t \\
0 & 1 & \cos(\theta)v_m\Delta t \\
0 & 0 & 1
\end{bmatrix}
$$

$$
\mathbf F_{\mathbf i} =\frac {\partial f} {\partial \mathbf i}\vert_{\mathbf x,\mathbf u_m} 
=
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}，
\mathbf Q_{\mathbf i}
=
\begin{bmatrix}
v_{x,i} & 0 & 0 \\
0 & v_{y,i} & 0 \\
0 & 0 & \omega_i
\end{bmatrix}
$$

## 多机器人UWB测距值校正

多机器人协同定位中，每个机器人配备有轮式编码器，机器人之间通过UWB测距，得到测距值后，我们可以根据测距值校正卡尔曼滤波器状态。以两个机器人为例，状态为$\mathbf x = [\mathbf p_1, \theta_1, \mathbf p_2, \theta_2]^T$ ,我们有观测模型：
$$
\mathbf y = h(\mathbf x_t) + \mathbf v
$$
$h()$是一个关于系统真实状态量的非线性方程，$\mathbf v \sim \mathcal N(0, \mathbf V)$是高斯白噪声。

误差状态卡尔曼滤波器估计的是误差状态量，因此更新方程为：
$$
\mathbf K = \mathbf P \mathbf H^T (\mathbf H \mathbf P \mathbf H^T + \mathbf V)^{-1} \\
\hat{\delta \mathbf x} \leftarrow \mathbf K(\mathbf y - h(\hat{\mathbf x_t})) \\
\mathbf P \leftarrow (\mathbf I - \mathbf K \mathbf H)\mathbf P
$$
其中$\mathbf H$是相对于误差状态量$\delta \mathbf x$的雅可比矩阵，在此时的最佳真实状态估计值$\hat {\mathbf x_t}=\mathbf x \oplus \hat{\delta \mathbf x}$处取值，由于此时我们并没有得到$\hat {\delta \mathbf x}$, 但是我们知道$\delta \mathbf x \sim \mathcal N(\hat{\delta \mathbf x}, \mathbf P)$, 即误差状态量的均值为0，所以我们有$\hat {\mathbf x_t}=\mathbf x$, 所以：
$$
\mathbf H = \frac{\partial h}{\partial \delta \mathbf x} \vert_{\mathbf x}
$$
由链式法则，有：
$$
\mathbf H = \frac{\partial h}{\partial \delta \mathbf x} \vert_{\mathbf x} = 
\frac{\partial h}{\partial \mathbf x_t} \vert_{\mathbf x}
\frac{\partial \mathbf x_t}{\partial \delta \mathbf x} \vert_{\mathbf x} 
= \mathbf H_{\mathbf x}\mathbf X_{\delta \mathbf x}
$$
其中，$\mathbf H_{\mathbf x}$是$h()$的相对于其参数的标准雅可比矩阵（在普通EKF中用到）。

对于机器人1和2之间的UWB测距来说，观测模型为：
$$
y = \Vert \mathbf p_1 - \mathbf p_2\Vert + v
$$
$\mathbf H_{\mathbf x}$ 计算为：
$$
\mathbf H_{\mathbf x} = \begin{bmatrix}
\mathbf e & 0 & \mathbf -\mathbf e & 0
\end{bmatrix}
$$
其中
$$
\mathbf e = \frac{\mathbf p_1^T - \mathbf p_2^T}{\Vert \mathbf p_1 - \mathbf p_2\Vert}
$$


 对于雅可比矩阵第二部分,$\mathbf H_{\delta \mathbf x}$是真实状态量相对于误差状态量的雅可比矩阵，
$$
\mathbf H_{\delta \mathbf x}=\frac{\partial \mathbf x_t}{\partial \delta \mathbf x} \vert{\mathbf x}
= \mathbf I_6
$$
所以，$\mathbf H = \mathbf H_{\mathbf x}\mathbf X_{\delta \mathbf x} = \mathbf H_{\mathbf x}$.

在得到误差状态量的估计值后，我们需要将其注入到估计状态量中：
$$
\mathbf x \leftarrow \mathbf x \oplus \hat {\delta \mathbf x}
$$


## 参考

[1] Sola, Joan. "Quaternion kinematics for the error-state Kalman filter." *arXiv preprint arXiv:1711.02508* (2017).

