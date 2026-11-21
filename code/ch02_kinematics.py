"""第 2 章 · 本体与运动学：关节角怎么变成手的位置。

讲义：robot-system-architecture/notes/lessons/02-kinematics.md
运行：uv run python ch02_kinematics.py
连杆长度和角度都是编的（L1 = 0.5 m，L2 = 0.4 m），不是练习题里的数。
"""

import numpy as np

L1, L2 = 0.5, 0.4  # 两节连杆长度（m），编的


def fk(theta: np.ndarray) -> np.ndarray:
    """两连杆正运动学：x = L1 cos θ1 + L2 cos(θ1+θ2)，y = L1 sin θ1 + L2 sin(θ1+θ2)。"""
    t1, t2 = theta
    return np.array([L1 * np.cos(t1) + L2 * np.cos(t1 + t2), L1 * np.sin(t1) + L2 * np.sin(t1 + t2)])


def jacobian(theta: np.ndarray) -> np.ndarray:
    """两连杆雅可比 J(θ)：第 j 列 = 只转第 j 个关节时手的速度方向。"""
    t1, t2 = theta
    s1, c1, s12, c12 = np.sin(t1), np.cos(t1), np.sin(t1 + t2), np.cos(t1 + t2)
    return np.array([[-L1 * s1 - L2 * s12, -L2 * s12], [L1 * c1 + L2 * c12, L2 * c12]])


def numeric_jacobian(theta: np.ndarray, h: float = 1e-6) -> np.ndarray:
    """用差分近似 J：每个关节挪 h，看手挪了多少，除以 h。"""
    cols = []
    for j in range(2):
        d = np.zeros(2)
        d[j] = h
        cols.append((fk(theta + d) - fk(theta - d)) / (2 * h))
    return np.stack(cols, axis=1)


def dls_ik(target: np.ndarray, theta0: np.ndarray, lam: float = 0.05, iters: int = 100) -> np.ndarray:
    """阻尼最小二乘 IK：Δθ = Jᵀ (J Jᵀ + λ² I)⁻¹ e，重复 iters 次。"""
    theta = theta0.astype(float).copy()
    for _ in range(iters):
        e = target - fk(theta)
        J = jacobian(theta)
        theta = theta + J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(2), e)
    return theta


def manipulability(theta: np.ndarray) -> float:
    """可操作度 w = sqrt(det(J Jᵀ))，越接近 0 越靠近奇异。"""
    J = jacobian(theta)
    return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))


def main() -> None:
    theta = np.radians([30.0, 60.0])
    print("\n第一步：自由度——平面两连杆有 2 个关节，手的位置 (x, y) 也是 2 个数")
    print("  三维空间放任意位姿要 6 个数：x, y, z + 3 个转角")

    print("\n第三步：正运动学 FK，θ1 = 30°，θ2 = 60°")
    p = fk(theta)
    print(f"  手在 x = {p[0]:.4f} m, y = {p[1]:.4f} m")
    print(f"  工作空间：内圈半径 |L1 − L2| = {abs(L1 - L2):.1f} m，外圈 L1 + L2 = {L1 + L2:.1f} m")

    print("\n第四步：逆运动学 IK——同一个点有肘朝上、肘朝下两组解")
    up = dls_ik(p, np.radians([0.0, 90.0]))
    down = dls_ik(p, np.radians([90.0, -60.0]))
    for name, th in (("解 1", up), ("解 2", down)):
        print(f"  {name}: θ = {np.round(np.degrees(th), 2).tolist()}°  → FK 回去 {np.round(fk(th), 4).tolist()}")

    print("\n第五步：雅可比 ẋ = J θ̇，解析式和差分对一下")
    J = jacobian(theta)
    print("  解析 J =\n", np.round(J, 4))
    print("  差分 J =\n", np.round(numeric_jacobian(theta), 4))
    F = np.array([0.0, -10.0])  # 手上挂 10 N 的重物（编的）
    print(f"  静力学 τ = Jᵀ F，F = {F.tolist()} N → τ = {np.round(J.T @ F, 4).tolist()} N·m")

    print("\n第六步：奇异——det J = L1 L2 sin θ2，θ2 → 0 时手臂伸直")
    for deg in (60.0, 20.0, 5.0, 0.0):
        th = np.radians([30.0, deg])
        print(f"  θ2 = {deg:>4}°: det J = {np.linalg.det(jacobian(th)):+.4f}  可操作度 w = {manipulability(th):.4f}")


if __name__ == "__main__":
    main()
