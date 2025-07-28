import numpy as np
from scipy.optimize import minimize

# 1. 定义目标函数：Rosenbrock函数
def rosenbrock(x):
    return (1 - x[0])**2 + 100 * (x[1] - x[0]**2)**2

# 2. 定义非线性不等式约束（圆内约束）
def circle_constraint(x):
    center = np.array([1/3, 1/3])
    radius = 1/3
    # 约束需返回 >=0 的值（满足约束时：圆心到x的距离 <= 半径 ==> 返回值 >=0）
    return radius**2 - np.sum((x - center)**2)  # 若返回值 >=0，则点在圆内

# 3. 设置约束条件
nonlinear_constraint = {
    'type': 'ineq',  # 不等式约束（>=0）
    'fun': circle_constraint
}

# 4. 设置变量边界
bounds = [(0.0, 0.5), (0.2, 0.8)]

# 5. 选择满足约束的初始点（例如 [0.4, 0.4]）
initial_guess = np.array([1, 1/4])
print("初始点约束检查：", circle_constraint(initial_guess) >= 0)  # 应为 True

# 6. 调用 minimize 函数
result = minimize(
    fun=rosenbrock,                 # 目标函数
    x0=initial_guess,               # 初始点
    method='SLSQP',                 # 支持非线性约束的算法
    bounds=bounds,                  # 边界约束
    constraints=[nonlinear_constraint], # 非线性约束列表
    options={'disp': True}          # 显示优化过程信息
)

# 7. 输出结果
print("\n优化结果：")
print(f"最优解: x = {result.x}")
print(f"函数最小值: f(x) = {result.fun:.6f}")
print(f"是否成功: {result.success}")
print(f"终止状态: {result.message}")
