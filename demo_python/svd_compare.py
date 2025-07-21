import numpy as np
from scipy.linalg import svd
import timeit

# 示例雅可比矩阵
J = np.array([[2*0.5, -1]])  # 在x_n=[0.5,0.25]处的雅可比

# 方法1：SVD计算零空间
def method1():
    U, s, Vh = svd(J, full_matrices=True)
    nullity = J.shape[1] - np.sum(s > 1e-7)
    return Vh[-nullity:].T if nullity > 0 else np.zeros((J.shape[1], 0))

# 方法2：伪逆计算投影矩阵
def method2():
    J_pinv = np.linalg.pinv(J)
    return np.eye(J.shape[1]) - J_pinv @ J

# 性能测试
t1 = timeit.timeit(method1, number=1000)
t2 = timeit.timeit(method2, number=1000)

print(f"SVD方法耗时: {t1*1000:.4f} ms (1000次调用)")
print(f"伪逆方法耗时: {t2*1000:.4f} ms (1000次调用)")

# 验证等价性
Z = method1()
P1 = Z @ Z.T if Z.size > 0 else np.zeros((2,2))
P2 = method2()

print("两种方法差异:", np.max(np.abs(P1 - P2)))