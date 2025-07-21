# 处理一个将有限范围随机采样点投影至约束流形上的问题：具体的例子是在[-1,1]范围内均匀采样，
# 并投影至约束子空间g(x)=x1^2-x2=0上面
# 方法1：零空间投影迭代，x_{k+1}=x_k-J^{+}g(x_k)+\beta P(x_rand-x_k)
# 方法2：预先在约束流形上生成足量点位，然后无需迭代再直接用增量投影x_p=x_n+ZZ^T(x_s-x_n)
# 方法3：迭代生成约束流形上的点位，然后无需迭代再直接用增量投影x_p=x_n+ZZ^T(x_s-x_n)，效果很差
# 方法4：迭代生成约束流形上的点位，然后用迭代投影得到x_p，再根据约束误差迭代生成约束流形上的点位x_pnew，效果很好
# 方法4应为最终方案，无论线性约束还是非线性约束，都可以得到约束流形上的点位，且约束误差不断缩小，且约束点位不断生成，效果很好

import numpy as np
import matplotlib.pyplot as plt

# 约束函数
def g(x):
    return x[0]**2 - x[1]

# 约束雅可比矩阵
def jacobian(x):
    return np.array([[2*x[0]], [-1]]).T  # 1×2 雅可比矩阵

# 零空间投影算子
def null_space_projector(J):
    """计算零空间投影矩阵 P = I - J⁺J"""
    # 计算伪逆
    J_pinv = np.linalg.pinv(J)
    # 计算投影矩阵
    return np.eye(2) - J_pinv @ J

# 流形投影函数
def project_to_manifold(x_rand, max_iter=10, tol=1e-6):
    """将点投影到约束流形 g(x)=0 上"""
    x = x_rand.copy()
    
    for i in range(max_iter):
        # 计算约束值和雅可比
        g_val = g(x)
        J = jacobian(x)
        
        # 检查收敛
        if abs(g_val) < tol:
            return x
        
        # 计算伪逆
        J_pinv = np.linalg.pinv(J)
        
        # 计算零空间投影矩阵
        P = null_space_projector(J)
        
        # 约束空间修正
        delta_constraint = -J_pinv * g_val
        
        # 零空间修正 (保持接近原始点)
        delta_null = P @ (x_rand - x)
        
        # 组合更新
        x = x + delta_constraint.flatten() + 0.5 * delta_null
        
        # 确保在区域内
        x = np.clip(x, -1, 1)
    
    return x


def project_method1():
    # 采样和投影
    np.random.seed(42)
    num_samples = 1000
    samples = np.random.uniform(-1, 1, (num_samples, 2))
    projected = np.array([project_to_manifold(x) for x in samples])

    # 验证投影精度
    constraint_errors = [abs(g(x)) for x in projected]
    print(f"最大约束误差: {max(constraint_errors):.2e}")
    print(f"平均约束误差: {np.mean(constraint_errors):.2e}")

    # 可视化
    plt.figure(figsize=(10, 8))

    # 绘制原始采样点
    plt.scatter(samples[:, 0], samples[:, 1], c='blue', alpha=0.3, label='raw sample points')

    # 绘制投影点
    plt.scatter(projected[:, 0], projected[:, 1], c='red', s=20, label='projection points')

    # 绘制约束流形 (x1^2 = x2)
    x1_vals = np.linspace(-1, 1, 100)
    x2_vals = x1_vals**2
    plt.plot(x1_vals, x2_vals, 'g-', linewidth=2, label='constrained manifold $x_1^2 = x_2$')

    # 绘制投影线
    for i in range(min(50, num_samples)):
        plt.plot([samples[i, 0], projected[i, 0]], 
                [samples[i, 1], projected[i, 1]], 
                'k--', alpha=0.2)

    plt.xlabel('$x_1$')
    plt.ylabel('$x_2$')
    plt.title('constrained manifold projection: $x_1^2 - x_2 = 0$')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# ====================================================================================

from scipy.linalg import svd
from scipy.spatial.distance import cdist

class ConstrainedProjector:
    def __init__(self, step_size=0.1, revolute_weight=1.0, prismatic_weight=1.0):
        self.step_size = step_size
        self.revolute_weight = revolute_weight
        self.prismatic_weight = prismatic_weight
    
    def jacobian(self, x):
        """计算约束雅可比矩阵"""
        return np.array([[2*x[0], -1]])  # g(x) = x1² - x2
    
    def null_space(self, x):
        """计算约束雅可比矩阵的零空间 (C++ 代码的 Python 实现)"""
        J = self.jacobian(x)
        
        # SVD 分解
        U, s, Vh = svd(J, full_matrices=True)
        
        # 计算零空间维度
        nullity = J.shape[1] - np.sum(s > 1e-7)
        
        # 获取零空间基
        Z = Vh[-nullity:].T if nullity > 0 else np.zeros((J.shape[1], 0))
        
        return Z
    
    def project(self, x_s, x_n):
        """
        使用自适应步长将采样点投影到约束流形
        参数:
            x_s: 随机采样点 (numpy array)
            x_n: 流形上的参考点 (numpy array)
        返回:
            投影点 (numpy array)
        """
        # 计算位移向量
        vec_new = x_s - x_n
        
        # 计算零空间矩阵
        Z = self.null_space(x_n)
        
        if Z.size == 0:
            return x_n  # 没有零空间，直接返回参考点
        
        # 零空间投影
        vec_proj = Z @ (Z.T @ vec_new)
        
        # 自适应步长计算
        k = 1.0
        for i in range(len(vec_proj)):
            vi = vec_proj[i]
            
            # 简化假设: 所有关节都是旋转关节
            max_step = self.step_size * self.revolute_weight
            clipped_vi = np.clip(vi, -max_step, max_step)
            
            # 避免除以零
            if abs(vi) > 1e-6:
                k_i = clipped_vi / vi
                k = min(k, k_i)
        
        # 应用自适应步长
        vec_proj = k * vec_proj
        
        # 计算投影点
        return x_n + vec_proj
    
def find_nearst_point(x_s, manifold_points):
    points_array = np.array(manifold_points)
    distances = cdist([x_s], points_array).flatten()
    nearest_idx = np.argmin(distances)
    return points_array[nearest_idx]

# 在流形上生成参考点
def generate_manifold_points(num_points=100):
    x1 = np.linspace(-1, 1, num_points)
    return np.column_stack((x1, x1**2))

# 查找最近流形点 (使用 KD 树加速)
from scipy.spatial import cKDTree

def create_manifold_kdtree(manifold_points):
    return cKDTree(manifold_points)


def project_method2():
    # 初始化投影器
    projector = ConstrainedProjector(
        step_size=0.2,
        revolute_weight=1.0,
        prismatic_weight=1.0
    )
    
    # 生成流形参考点并创建 KD 树
    manifold_points = generate_manifold_points(100)
    kdtree = create_manifold_kdtree(manifold_points)
    # manifold_points = [np.array([0.5, 0.5**2])]
    
    # 随机采样点
    np.random.seed(42)
    samples = np.random.uniform(-1, 1, (1000, 2))
    
    # 存储投影点
    projected_points = []
    
    # 对每个采样点进行投影
    for x_s in samples:
        # 找到最近的流形点作为参考点
        # _, idx = kdtree.query(x_s)
        # x_n = manifold_points[idx]
        x_n = find_nearst_point(x_s, manifold_points)
        
        # 计算投影点
        x_p = projector.project(x_s, x_n)
        projected_points.append(x_p)
        # manifold_points.append(x_p)
    
    projected_points = np.array(projected_points)
    
    # 计算约束误差
    errors = np.abs([g(x) for x in projected_points])
    print(f"最大约束误差: {np.max(errors):.2e}")
    print(f"平均约束误差: {np.mean(errors):.2e}")
    
    # 可视化
    import matplotlib.pyplot as plt
    plt.figure(figsize=(10, 8))
    
    # 绘制流形曲线
    x1_vals = np.linspace(-1, 1, 100)
    plt.plot(x1_vals, x1_vals**2, 'g-', linewidth=3, label='constrained manifold $x_1^2 = x_2$')
    
    # 绘制原始采样点
    plt.scatter(samples[:, 0], samples[:, 1], c='blue', alpha=0.6, label='raw sample points')
    
    # 绘制投影点
    plt.scatter(projected_points[:, 0], projected_points[:, 1], c='red', s=40, label='projection points')
    
    # 绘制投影线
    for i in range(100):
        plt.plot([samples[i, 0], projected_points[i, 0]], 
                 [samples[i, 1], projected_points[i, 1]], 
                 'k--', alpha=0.3)
    
    plt.xlabel('$x_1$')
    plt.ylabel('$x_2$')
    plt.title('nbull-space projection')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()


# ============================================================================
# 零空间计算函数
def compute_null_space(J):
    """
    计算雅可比矩阵的零空间基
    :param J: 约束雅可比矩阵 (m×n)
    :return Z: 零空间矩阵 (n×k), k=nullity
    """
    # SVD分解
    U, s, Vh = np.linalg.svd(J, full_matrices=True)
    
    # 计算零空间维度 (奇异值小于阈值的数量)
    tolerance = 1e-7
    rank = np.sum(s > tolerance)
    nullity = J.shape[1] - rank  # 零空间维度
    
    # 提取零空间基 (V的右部分列向量)
    V = Vh.T
    Z = V[:, rank:] if nullity > 0 else np.zeros((J.shape[1], 0))
    
    return Z

# 直接投影函数
def direct_projection(x_s, manifold_points, alpha=0.1):
    """
    将随机点投影到约束流形
    :param x_s: 随机采样点 (2,)
    :param manifold_points: 流形点集列表 [N×(2,)]
    :param alpha: 步长缩放因子
    :return x_p: 投影点 (2,)
    """
    # 1. 在点集中找最近点x_n
    points_array = np.array(manifold_points)
    distances = cdist([x_s], points_array).flatten()
    nearest_idx = np.argmin(distances)
    x_n = points_array[nearest_idx]
    
    # 2. 计算x_n处的零空间矩阵
    J = jacobian(x_n)
    Z = compute_null_space(J)
    
    # 3. 计算投影点 (当零空间存在时)
    if Z.size > 0:
        displacement = x_s - x_n
        # 投影到切空间: Z@Zᵀ@displacement
        tangent_projection = Z @ Z.T @ displacement
        x_p = x_n + alpha * tangent_projection
    else:
        x_p = x_n.copy()  # 零空间为空时返回最近点
    
    return x_p
def project_method3():
    # 初始化流形点集 (包含至少一个点)
    manifold_points = [np.array([0.5, 0.5**2])]  # [0.5, 0.125]
    
    # 生成随机采样点
    np.random.seed(42)
    num_samples = 1000
    samples = np.random.uniform(-1, 1, (num_samples, 2))
    
    # 执行投影并动态更新点集
    projected_points = []
    for x_s in samples:
        x_p = direct_projection(x_s, manifold_points, alpha=1.0)
        projected_points.append(x_p)
        manifold_points.append(x_p)  # 将新投影点加入点集
    
    projected = np.array(projected_points)
    
    # 验证投影精度
    constraint_errors = [abs(g(x)) for x in projected]
    print(f"最大约束误差: {max(constraint_errors):.2e}")
    print(f"平均约束误差: {np.mean(constraint_errors):.2e}")
    
    # 可视化
    plt.figure(figsize=(12, 10))
    
    # 绘制原始采样点
    plt.scatter(samples[:, 0], samples[:, 1], 
                c='blue', alpha=0.2, label='raw sample points')
    
    # 绘制投影点
    plt.scatter(projected[:, 0], projected[:, 1], 
                c='red', s=15, alpha=0.7, label='projection points')
    
    # 绘制约束流形 (x₀³ = x₁)
    x_vals = np.linspace(-1, 1, 500)
    y_vals = x_vals**2
    plt.plot(x_vals, y_vals, 'g-', linewidth=3, 
             label='constrained manifold: $x_0^3 - x_1 = 0$')
    
    # 绘制投影线 (前50个样本)
    for i in range(min(50, num_samples)):
        plt.plot([samples[i, 0], projected[i, 0]],
                 [samples[i, 1], projected[i, 1]],
                 'k--', alpha=0.3, linewidth=0.8)
    
    # 绘制初始点 (特殊标记)
    plt.scatter(manifold_points[0][0], manifold_points[0][1], 
                s=120, c='purple', marker='*', label='initial point')
    
    plt.xlabel('$x_0$')
    plt.ylabel('$x_1$')
    plt.title('null-space projection')
    plt.legend(loc='upper left')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axis('equal')
    plt.tight_layout()
    # plt.savefig('direct_projection.png', dpi=300)
    plt.show()


# ==============================================================================
def g_line(x):
    return 2*x[0] - x[1]

def g_nonlinear(x):
    return x[0]**2 - x[1]

class IncrementalManifoldProjector:
    def __init__(self, initial_point, step_size=0.2, growth_threshold=0.1, constraint_line=True):
        """
        参数:
            initial_point: 流形上的初始点 (numpy array)
            step_size: 投影步长
            growth_threshold: 新增点距离阈值
        """
        # 初始化流形点集
        self.manifold_points = [initial_point]
        self.kdtree = cKDTree([initial_point])
        self.step_size = step_size
        self.growth_threshold = growth_threshold
        self.projection_count = 0
        self.growth_count = 0
        self.constraint_line = constraint_line
        
    def jacobian(self, x):
        """线性约束雅可比矩阵 (g(x) = 2*x1 - x2)"""
        """非线性约束雅可比矩阵 (g(x) = x1² - x2)"""
        if self.constraint_line:
            return np.array([[2, -1]])
        else:
            return np.array([[2*x[0], -1]])
    
    def null_space(self, x):
        """计算零空间"""
        J = self.jacobian(x)
        U, s, Vh = np.linalg.svd(J, full_matrices=True)
        nullity = J.shape[1] - np.sum(s > 1e-7)
        return Vh[-nullity:].T if nullity > 0 else np.zeros((J.shape[1], 0))
    
    def project(self, x_s):
        # x_s = np.array([0.6,0.3])
        """将采样点投影到约束流形"""
        self.projection_count += 1
        
        # 在流形点集中找到最近点
        distances, indices = self.kdtree.query(x_s, k=1)
        x_n = self.manifold_points[indices]
        
        # 计算零空间投影
        vec_new = x_s - x_n
        Z = self.null_space(x_n)
        
        if Z.size > 0:
            # 零空间投影
            vec_proj = Z @ (Z.T @ vec_new)
            
            # 自适应步长
            k = 1.0
            for i in range(len(vec_proj)):
                vi = vec_proj[i]
                max_step = self.step_size
                clipped_vi = np.clip(vi, -max_step, max_step)
                if abs(vi) > 1e-6:
                    k_i = clipped_vi / vi
                    k = min(k, k_i)
            
            vec_proj = k * vec_proj
            x_p = x_n + vec_proj
        else:
            x_p = x_n
        
        # 检查是否添加新点
        # min_dist = np.min(np.linalg.norm(np.array(self.manifold_points) - x_p, axis=1))
        # if min_dist > self.growth_threshold:
        #     self.manifold_points.append(x_p)
        #     self.kdtree = cKDTree(self.manifold_points)
        #     self.growth_count += 1
        
        return x_p

    def null_space_project_with_correction(self, x_s, max_iter=10, tol=1e-5):
        # 原始零空间投影
        x_p = self.project(x_s)  # 你的现有函数

        # 在流形点集中找到最近点
        distances, indices = self.kdtree.query(x_s, k=1)
        x_n = self.manifold_points[indices]
        
        # 牛顿迭代校正约束违反
        for it_times in range(max_iter):
            g_val = g_line(x_p) if self.constraint_line else g_nonlinear(x_p) # 约束函数值 g(x)=x1^2-x2
            if abs(g_val) < tol:
                break
            J = self.jacobian(x_p)       # 计算当前雅可比
            # 求解欠定方程 J * dx = -g_val (最小范数解)
            dx = -J.T * g_val / (J @ J.T)
            x_p = x_p + dx[:,0]

        min_dist = np.min(np.linalg.norm(np.array(self.manifold_points) - x_p, axis=1))
        if min_dist > self.growth_threshold:
            self.manifold_points.append(x_p)
            self.kdtree = cKDTree(self.manifold_points)
            self.growth_count += 1

        return x_p

    def visualize(self, samples=None, projections=None):
        """可视化当前状态"""
        plt.figure(figsize=(12, 10))
        
        # 绘制流形曲线
        x1_vals = np.linspace(-1, 1, 100)
        if self.constraint_line:
            x2_vals = 2*x1_vals
        else:
            x2_vals = x1_vals**2
        plt.plot(x1_vals, x2_vals, 'g-', linewidth=2, label='real manifold', alpha=0.7)
        
        # 绘制流形点集
        points = np.array(self.manifold_points)
        plt.scatter(points[:, 0], points[:, 1], c='blue', s=50, 
                    label=f'manifold ({len(points)}points)')
        
        # 绘制采样点和投影点
        if samples is not None and projections is not None:
            plt.scatter(samples[:, 0], samples[:, 1], c='gray', alpha=0.4, 
                        label=f'sample ({len(samples)}points)')
            plt.scatter(projections[:, 0], projections[:, 1], c='red', s=30, 
                        label=f'projection ({len(projections)}points)')
            
            # 绘制投影线
            for i in range(min(50, len(samples))):
                plt.plot([samples[i, 0], projections[i, 0]], 
                         [samples[i, 1], projections[i, 1]], 
                         'k--', alpha=0.2)
        
        plt.xlabel('$x_1$')
        plt.ylabel('$x_2$')
        plt.title(f'incremental manifold projection (size: {len(points)} | increment ratio: {self.growth_count/self.projection_count:.1%})')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()
        
        # 打印统计信息
        print(f"投影操作次数: {self.projection_count}")
        print(f"新增点数量: {self.growth_count}")
        print(f"点集增长率: {self.growth_count/self.projection_count:.1%}")
        print(f"当前点集大小: {len(self.manifold_points)}")
        
        return points

def project_method4():
    constraint_line = False
    # 初始化投影器 (从[0,0]点开始)
    projector = IncrementalManifoldProjector(
        initial_point=np.array([0.5, 0.25]),
        step_size=0.3,
        growth_threshold=0.02,
        constraint_line=constraint_line
    )
    
    # 随机采样点
    np.random.seed(42)
    samples = np.random.uniform(-1, 1, (200, 2))
    
    # 存储投影点
    projections = []
    constraint_errors = []
    
    # 分批处理采样点
    for i, x_s in enumerate(samples):
        # x_p = projector.project(x_s)
        x_p = projector.null_space_project_with_correction(x_s)
        projections.append(x_p)
        if constraint_line:
            constraint_errors.append(abs(g_line(x_p)))
        else:
            constraint_errors.append(abs(g_nonlinear(x_p)))
        
        # 每50个点可视化一次
        if (i+1) % 50 == 0:
            projector.visualize(samples[:i+1], np.array(projections))
    
    # 最终评估
    projections = np.array(projections)
    print("\n最终评估结果:")
    print(f"最大约束误差: {np.max(constraint_errors):.2e}")
    print(f"平均约束误差: {np.mean(constraint_errors):.2e}")
    
    # 可视化点集分布
    plt.figure(figsize=(10, 8))
    points = np.array(projector.manifold_points)
    plt.scatter(points[:, 0], points[:, 1], c=np.arange(len(points)), 
                cmap='viridis', s=50)
    plt.colorbar(label='添加顺序')
    # plt.plot(x1_vals, x1_vals**2, 'r-', alpha=0.5)
    plt.title("流形点集分布 (按添加顺序着色)")
    plt.xlabel('$x_1$')
    plt.ylabel('$x_2$')
    plt.grid(True)
    plt.show()

# ===================================================================================
if __name__ == "__main__":
    # project_method1()
    # project_method2()
    # project_method3()
    project_method4()