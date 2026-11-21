"""第 6 章 · 底层控制：电流环 → 速度环 → 位置环，再到阻抗与全身控制。

讲义：robot-system-architecture/notes/lessons/06-low-level-control.md
运行：uv run python ch06_low_level_control.py
PD 增益、FOC 的电流和角度、阻抗仿真里的质量都是编的；刚度只用了动画里的 300 N/m 和另一个编的 2000 N/m。
"""

import numpy as np


def pd_ff(q_ref: float, q: float, dq_ref: float, dq: float, kp: float, kd: float, tau_ff: float) -> float:
    """关节 PD + 前馈：τ = K_p (q* − q) + K_d (q̇* − q̇) + τ_ff。"""
    return kp * (q_ref - q) + kd * (dq_ref - dq) + tau_ff


def clarke(ia: float, ib: float, ic: float) -> tuple[float, float]:
    """Clarke 变换：三相 (a, b, c) → 两轴 (α, β)，幅值不变的写法。"""
    i_alpha = (2 / 3) * (ia - 0.5 * ib - 0.5 * ic)
    i_beta = (2 / 3) * (np.sqrt(3) / 2) * (ib - ic)
    return i_alpha, i_beta


def park(i_alpha: float, i_beta: float, theta_e: float) -> tuple[float, float]:
    """Park 变换：按转子电角度 θ_e 旋转到 (d, q)；就是一个 2×2 旋转矩阵。"""
    c, s = np.cos(theta_e), np.sin(theta_e)
    return c * i_alpha + s * i_beta, -s * i_alpha + c * i_beta


def three_phase(i_d: float, i_q: float, theta_e: float) -> tuple[float, float, float]:
    """反过来：给定 i_d、i_q 和角度，合成三相电流（反 Park + 反 Clarke）。"""
    phases = (0.0, -2 * np.pi / 3, 2 * np.pi / 3)
    return tuple(i_d * np.cos(theta_e + p) - i_q * np.sin(theta_e + p) for p in phases)  # type: ignore[return-value]


def critical_damping(k: float, m: float) -> float:
    """临界阻尼 D = 2 √(K m)。"""
    return 2 * np.sqrt(k * m)


def impedance_wall(k: float, m: float = 1.0, depth: float = 0.02, t_end: float = 2.0, dt: float = 1e-4) -> float:
    """一维阻抗控制撞墙：墙在 x = 0（刚度 1e5 N/m 的硬弹簧），目标在墙后 depth。返回最后的接触力（N）。"""
    d = critical_damping(k, m)
    k_wall = 1e5
    x, v = -0.10, 0.0  # 从墙前 10 cm 出发
    for _ in range(int(t_end / dt)):
        f_ctrl = k * (depth - x) + d * (0.0 - v)  # F = K (x* − x) + D (ẋ* − ẋ)
        f_wall = -k_wall * x if x > 0 else 0.0
        v += (f_ctrl + f_wall) / m * dt
        x += v * dt
    return k_wall * x if x > 0 else 0.0


def main() -> None:
    print("\n第一步：PD + 前馈（编的：q* = 1.0，q = 0.9，q̇ = 0.5，Kp = 50，Kd = 2，τ_ff = 3）")
    tau = pd_ff(1.0, 0.9, 0.0, 0.5, kp=50, kd=2, tau_ff=3)
    print(f"  τ = 50 × 0.1 + 2 × (0 − 0.5) + 3 = {tau:.1f} N·m")

    print("\n第二步：FOC——三相电流经过 Clarke、Park 变成两个直流量")
    theta_e = np.radians(40.0)
    ia, ib, ic = three_phase(0.0, 5.0, theta_e)
    print(f"  三相 a, b, c = {ia:+.3f}, {ib:+.3f}, {ic:+.3f} A（加起来 {ia + ib + ic:+.1e}）")
    i_al, i_be = clarke(ia, ib, ic)
    i_d, i_q = park(i_al, i_be, theta_e)
    print(f"  Clarke → α, β = {i_al:+.3f}, {i_be:+.3f}；Park → d, q = {i_d:+.3f}, {i_q:+.3f}")
    print(f"  τ = K_t i_q：K_t = 0.1 N·m/A → τ = {0.1 * i_q:.2f} N·m")

    print("\n第四步：阻抗控制撞墙，目标在墙后 2 cm，稳态力 ≈ K × 0.02")
    for k in (300.0, 2000.0):
        f = impedance_wall(k)
        print(f"  K = {k:>6.0f} N/m：仿真稳态力 {f:6.2f} N，估算 K × 0.02 = {k * 0.02:.1f} N")

    print("\n公式表：临界阻尼 D = 2 √(K m)，m = 1 kg")
    for k in (300.0, 2000.0):
        print(f"  K = {k:.0f} → D = {critical_damping(k, 1.0):.2f} N·s/m")


if __name__ == "__main__":
    main()
