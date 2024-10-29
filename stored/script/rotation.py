import numpy as np

def is_rotation_matrix(matrix, tol=1e-6):
    """
    检查给定的矩阵是否为旋转矩阵。

    参数:
    - matrix (list或numpy.ndarray): 输入的3x3或4x4矩阵。
    - tol (float): 容差，用于数值比较，默认为1e-6。

    返回:
    - bool: 如果是旋转矩阵则返回True，否则返回False。
    """

    # 将输入转换为numpy数组
    matrix = np.array(matrix)

    # 检查矩阵的形状是否为3x3或4x4
    if matrix.shape == (4, 4):
        # 提取左上角的3x3子矩阵
        R = matrix[:3, :3]
    elif matrix.shape == (3, 3):
        R = matrix
    else:
        print("矩阵必须是3x3或4x4的。")
        return False

    # 检查正交性：R * R^T 应该等于单位矩阵
    should_be_identity = np.dot(R, R.T)
    print(should_be_identity)
    identity = np.identity(3)
    orthogonal = np.allclose(should_be_identity, identity, atol=tol)
    if not orthogonal:
        print("矩阵的3x3部分不满足正交性。")

    # 检查行列式是否为+1
    det = np.linalg.det(R)
    print(det)
    if not np.isclose(det, 1.0, atol=tol):
        print(f"矩阵的行列式为 {det}，不等于 +1。")
        return False

    # 如果所有检查都通过，则是旋转矩阵
    return True

# 示例用法
if __name__ == "__main__":
    matrix = [
        [-0.16810, 0.88225, 0.43975, 0.00000],
        [0.86211, -0.08476, 0.49959, 0.00000],
        [-0.47804, -0.46309, 0.74634, 0.00000],
        [0.00000, 0.00000, 0.00000, 1.00000]
    ]

    if is_rotation_matrix(matrix):
        print("该矩阵是一个有效的旋转矩阵。")
    else:
        print("该矩阵不是一个有效的旋转矩阵。")
