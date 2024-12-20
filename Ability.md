<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->

<!-- code_chunk_output -->

- [机器人学(Robotics)](#机器人学robotics)
  - [1. 数学理论(Mathmatics)](#1-数学理论mathmatics)
    - [1.1 姿态表述方式，四元数与轴角的关系](#11-姿态表述方式四元数与轴角的关系)
    - [1.2 最优化算法：梯度下降法，牛顿法，高斯-牛顿法，LM算法](#12-最优化算法梯度下降法牛顿法高斯-牛顿法lm算法123)
      - [1.2.1 梯度下降](#121-梯度下降)
      - [1.2.2 牛顿法](#122-牛顿法)
      - [1.2.3 高斯-牛顿法](#123-高斯-牛顿法)
      - [1.2.4 LM算法](#124-lm算法)
      - [1.2.5 总结](#125-总结)
    - [1.3 矩阵/向量求导等运算规则](#13-矩阵向量求导等运算规则)
  - [2. 运动学(Kinematics)](#2-运动学kinematics)
    - [2.1 标准/改进DH法，正逆运动学：解析解/数值解](#21-标准改进dh法正逆运动学解析解数值解)
    - [2.2 雅可比矩阵：Jaco_rpy,Jaco_euler,Jaco之间的关系](#22-雅可比矩阵jaco_rpyjaco_eulerjaco之间的关系)
    - [2.3 误差模型，运动学标定：DH参数标定，绕针尖标定，手眼标定](#23-误差模型运动学标定dh参数标定绕针尖标定手眼标定)
  - [3. 动力学(Dynamics)](#3-动力学dynamics)
    - [3.1 牛顿-欧拉法建模，拉格朗日法建模，动力学方程](#31-牛顿-欧拉法建模拉格朗日法建模动力学方程)
    - [3.2 动力学方程线性化](#32-动力学方程线性化)
    - [3.3 动力学参数辨识，激励轨迹的求解](#33-动力学参数辨识激励轨迹的求解)
    - [3.4 柔顺控制：导纳/阻抗控制，力控制](#34-柔顺控制导纳阻抗控制力控制)
  - [4. 运动规划(Motion Planning)](#4-运动规划motion-planning)
    - [4.1 路径规划：RRT及其变种，PRM及其变种](#41-路径规划rrt及其变种prm及其变种)
    - [4.2 路径插补：贝塞尔曲线/样条曲线，多点连续位姿插补(squad算法)](#42-路径插补贝塞尔曲线样条曲线多点连续位姿插补squad算法)
    - [4.3 轨迹插补](#43-轨迹插补)
      - [4.3.1 梯形速度规划，多项式速度规划，DoubleS规划](#431-梯形速度规划多项式速度规划doubles规划)
      - [4.3.2 时间最优(路径-->轨迹)：IPTP，ISP，TOTP，Toppra等算法](#432-时间最优路径-轨迹iptpisptotptoppra等算法)
      - [4.3.3 ros_controllers中的轨迹插补](#433-ros_controllers中的轨迹插补)
  - [5. 遥操作(Teleoperation)](#5-遥操作teleoperation)
    - [5.1 遥操作的稳定性和透明性之间的关系](#51-遥操作的稳定性和透明性之间的关系)
    - [5.2 建立位姿映射关系：基于速度，以位置闭环](#52-建立位姿映射关系基于速度以位置闭环)
    - [5.3 增加从端虚拟位置约束，速度约束，增加主端力约束](#53-增加从端虚拟位置约束速度约束增加主端力约束)
  - [6. 机器人学习(Robot Learning)](#6-机器人学习robot-learning)
    - [6.1 轨迹泛化：DMP算法原理，KMP算法原理](#61-轨迹泛化dmp算法原理kmp算法原理)
    - [6.2 基于强化学习](#62-基于强化学习)
    - [6.3 基于模仿学习](#63-基于模仿学习)
  - [7. 机器人操作系统ROS](#7-机器人操作系统ros)
    - [7.1 ROS的架构与原理](#71-ros的架构与原理)
    - [7.2 MoveIt的架构与原理](#72-moveit的架构与原理)

<!-- /code_chunk_output -->

<!-- pagebreak -->



---
# 机器人学(Robotics)
---
<!-- pagebreak -->


## 1. 数学理论(Mathmatics)
### 1.1 姿态表述方式，四元数与轴角的关系
### 1.2 最优化算法：梯度下降法，牛顿法，高斯-牛顿法，LM算法[^1][^2][^3]
最优化问题的一般形式为：
> $min f(x)$
> $s.t.x \in X$
> 其中$x$为决策变量，$f(x)$为目标函数，$X$为约束集或可行域。特别地，如果$X=R^n$，则变成了无约束最优化问题

最优化方法通过采用迭代法求它的最优解，基本思路为$x_{k+1} = x_k+\alpha d_k$
上式中，$\alpha$为步长因子，$d_k$为搜索方向


#### 1.2.1 梯度下降
**介绍**：以所处的位置为基准，寻找这个位置最陡峭的地方，然后朝着下降方向走一步，然后又继续以当前位置为基准，再找最陡峭的地方，直到最后到达最低处。

**梯度**：在单变量函数中，梯度就是函数的微分，代表着函数在给定点的切线斜率，在多变量函数中，梯度是一个向量，其方向表示函数在给定点上升最快的方向。

**数学表示**：
$\mathbf{\Theta}_1 = \mathbf{\Theta}_0-\alpha\nabla J(\mathbf{\Theta})$


#### 1.2.2 牛顿法
**介绍**：牛顿法是求解无约束优化问题最早使用的经典方法之一：用迭代点$x_k$出的一阶导数(梯度)和二阶导数(海森矩阵)对目标函数进行二次函数近似，然后把二次模型的极小值点作为新的迭代点，并不断重复这个过程，直至得到满足精度要求得近似极小值点。

**数学推导**：
将函数$f(x)$在$x_k$处进行泰勒展开，那么前三项为
$$
q_k(x) = f(x_k)+\mathbf{g}_k^T(x-x_k)+\frac{1}{2}(x-x_k)^T\mathbf{H}_k(x-x_k)
$$
其中$\mathbf{g}_k=\nabla f(x_k)$，$\mathbf{H}_k=\nabla^2f(x_k)$
求二次函数$q_k(x)$得稳定点(求极值点，也就是令一阶导数为0)
$$
\nabla q_k(x) = \mathbf{g}_k+\mathbf{H}_k^{-1}(x-x_k) = 0
$$
若$\mathbf{H}_k$非奇异，那么上述方程的解(记其解为$x_{k+1}$)即为牛顿法的迭代公式
$$
x_{k+1} = x_k - \mathbf{H}_k^{-1}\mathbf{g}_k
$$
上述迭代公式中每步迭代都需要求解海森矩阵的逆$\mathbf{H}_k^{-1}$，在实际计算过程中，一般先通过解$\mathbf{H}_k^{-1}d=-\mathbf{g}_k$得到$d_k$，然后令$x_{k+1}=x_k+d_k$进行迭代来避免求逆。


#### 1.2.3 高斯-牛顿法
对于多变量单输出的函数，对变量求微分即得到梯度，对于多变量多输出的函数，对变量求微分即得到雅可比矩阵。
对于多变量多输出函数，其一阶泰勒展开表达式为
$$
\mathbf{f}(x) = \mathbf{f}(x_0)+\mathbf{J}(\mathbf{x}-\mathbf{x_0})+o(||\mathbf{x}-\mathbf{x_0}||)
$$
海森矩阵就是梯度的雅可比矩阵$\mathbf{H}(f_k(\mathbf{x})) = \mathbf{J}(\nabla f_k(x))$
设模型函数为$$f(x_1,x_2,...,x_p;\beta_1,\beta_2,...,\beta_n)$$，优化目标函数$$min S = \sum_{i=1}^m (f(\mathbf{x_i};\mathbf{\beta_i})-y_i)^2$$，那么第i次观测点的预测偏差为$$r_i = f(\mathbf{x_i};\mathbf{\beta_i})-y_i$$为标量，那么m次观测误差可以组成向量$\mathbf{r} = [r_1,r_2,...,r_m]^T$，那么目标函数便可以简化为$$S = \sum_{i=1}^m r_i^2 = \mathbf{r}^T\mathbf{r}$$

那么目标函数的梯度$$\nabla S(\mathbf{\beta}) = [\frac{\partial S}{\partial \beta_1},...,\frac{\partial S}{\beta_n}]^T$$
其中$$\frac{\partial S}{\beta_j} = 2\sum_{i=1}^m r_i\frac{\partial r_i}{\partial \beta_j}$$
最终梯度可以表示为$\nabla S = 2\mathbf{J}^T\mathbf{r}$，海森矩阵也可以近似表示为$\mathbf{H} = 2(\mathbf{J}^T\mathbf{J}+\mathbf{O})$

牛顿法可以进一步化简
$$\begin{aligned}
\mathbf{\beta_{k+1}} &= \mathbf{\beta_k} - \mathbf{H}^{-1}\mathbf{g}_k \\
&= \mathbf{\beta_k} - (2(\mathbf{J}^T\mathbf{J}+\mathbf{O}))^{-1}2\mathbf{J}^T\mathbf{r} \\
&= \mathbf{\beta_k} - (\mathbf{J}^T\mathbf{J})^{-1}\mathbf{J}^T\mathbf{r}
\end{aligned}$$


#### 1.2.4 LM算法
牛顿法虽然收敛速度快，但是需要计算海森矩阵，对于高维问题，计算二阶导数会很复杂，因此有了高斯-牛顿法。
高斯-牛顿法不直接计算海森矩阵，而是通过雅可比矩阵对海森矩阵进行近似：$\mathbf{H} \approx \mathbf{J}^T\mathbf{J}$

但是用雅可比矩阵近似海森矩阵，如果雅可比矩阵不正定，那么近似后的海森矩阵也不正定，此时牛顿法将不再收敛，所以要引入一个对角阵与之相加$\mathbf{H} \approx \mathbf{J}^T\mathbf{J}+\mu\mathbf{I}$。
这就得到了LM算法$$x_{k+1} = x_k - (\mathbf{J}^T\mathbf{J}+\mu\mathbf{I})^{-1}\mathbf{g}_k$$

当$\mu$接近0时，LM算法就近似于高斯-牛顿法，当$\mu$很大时，LM算法近似于梯度下降法，因此LM算法也称为高斯-牛顿法和梯度下降法的结合


#### 1.2.5 总结
|Algorithm      |Update Rules   |Convergence   |Computation Complexity   |
|---|---|---|---|
|EBP algorithm  |$\mathbf{w}_{k+1} = \mathbf{w}_k-\alpha \mathbf{g}_k$                  |Stable,slow|Gradient|
|Newton algorithm   |$\mathbf{w}_{k+1} = \mathbf{w}_k-\mathbf{H}_k^{-1}\mathbf{g}_k$    |Unstable,fast |Gradient,Hessian |
|Gauss-Newton algorithm |$\mathbf{w}_{k+1} = \mathbf{w}_k-(\mathbf{J}_k^T\mathbf{J}_k)^{-1}\mathbf{J}_k\mathbf{e}_k$ |Unstable,fast |Jacobian |
|Levenberg-Marquardt algorithm |$\mathbf{w}_{k+1} = \mathbf{w}_k-(\mathbf{J}_k^T\mathbf{J}_k+\mu\mathbf{I})^{-1}\mathbf{J}_k\mathbf{e}_k$ |Stable,fast |Jacobian |


[^1]: [最优化方法——梯度下降法、牛顿法、LM算法](https://blog.csdn.net/ha_lee/article/details/122363325)
[^2]: [最优化方法总结——梯度下降法、最速下降法、牛顿法、高斯牛顿法、LM法、拟牛顿法](https://blog.csdn.net/dongke1991/article/details/127981561)
[^3]: [Levenberg-Marquardt算法(LM)的前世今生](https://blog.csdn.net/qq_43349296/article/details/143635708)

### 1.3 矩阵/向量求导等运算规则

---
<!-- pagebreak -->

## 2. 运动学(Kinematics)
### 2.1 标准/改进DH法，正逆运动学：解析解/数值解
### 2.2 雅可比矩阵：Jaco_rpy,Jaco_euler,Jaco之间的关系
### 2.3 误差模型，运动学标定：DH参数标定，绕针尖标定，手眼标定

---
<!-- pagebreak -->

## 3. 动力学(Dynamics)
### 3.1 牛顿-欧拉法建模，拉格朗日法建模，动力学方程
### 3.2 动力学方程线性化
### 3.3 动力学参数辨识，激励轨迹的求解
### 3.4 柔顺控制：导纳/阻抗控制，力控制
<!-- pagebreak -->

## 4. 运动规划(Motion Planning)
### 4.1 路径规划：RRT及其变种，PRM及其变种
### 4.2 路径插补：贝塞尔曲线/样条曲线，多点连续位姿插补(squad算法)
### 4.3 轨迹插补
#### 4.3.1 梯形速度规划，多项式速度规划，DoubleS规划
#### 4.3.2 时间最优(路径-->轨迹)：IPTP，ISP，TOTP，Toppra等算法
#### 4.3.3 ros_controllers中的轨迹插补
---
## 5. 遥操作(Teleoperation)
### 5.1 遥操作的稳定性和透明性之间的关系
### 5.2 建立位姿映射关系：基于速度，以位置闭环
### 5.3 增加从端虚拟位置约束，速度约束，增加主端力约束
---
## 6. 机器人学习(Robot Learning)
### 6.1 轨迹泛化：DMP算法原理，KMP算法原理
### 6.2 基于强化学习
### 6.3 基于模仿学习
---
## 7. 机器人操作系统ROS
### 7.1 ROS的架构与原理
### 7.2 MoveIt的架构与原理





