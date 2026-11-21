"""第 3 章 · 执行器：电机 + 减速器 = 一个关节。

讲义：robot-system-architecture/notes/lessons/03-actuators.md
运行：uv run python ch03_actuators.py
电机参数是编的（K_t = 0.1 N·m/A、转子惯量 1e-4 kg·m²、4000 rpm），不是练习题里那台电机。
"""

import numpy as np

KT = 0.1  # 力矩常数 K_t（N·m/A），编的
J_ROTOR = 1e-4  # 转子惯量 J_m（kg·m²），编的
MOTOR_RPM = 4000.0  # 电机最高转速（rpm），编的
ETA = 0.9  # 减速器效率 η，编的


def motor_torque(i_amp: float) -> float:
    """电机力矩 τ_m = K_t · i。"""
    return KT * i_amp


def gear(n: float, tau_m: float, rpm_m: float, eta: float = ETA) -> dict[str, float]:
    """减速比 N 的四个后果：力矩 ×ηN、转速 ÷N、反射惯量 ×N²、反驱力矩大致 ×N。"""
    return {
        "tau_out": eta * n * tau_m,
        "rpm_out": rpm_m / n,
        "j_refl": n**2 * J_ROTOR,
        "backdrive_rel": n,  # 相对 N = 1 的倍数（摩擦 × N 的粗略模型）
    }


def power_balance(n: float, tau_m: float, omega_m: float, eta: float = ETA) -> tuple[float, float]:
    """能量守恒：输入功率 × η = 输出功率，返回 (τ_m ω_m η, τ_out ω_out)。"""
    g = gear(n, tau_m, 0.0, eta)
    return tau_m * omega_m * eta, g["tau_out"] * (omega_m / n)


def sea_torque(k: float, theta_motor_side: float, theta_joint: float) -> float:
    """串联弹性执行器：τ = k (θ_电机侧 − θ_关节侧)，θ_电机侧 已经换算到减速器输出端。"""
    return k * (theta_motor_side - theta_joint)


def peak_torque(m: float, r: float, inertia: float, alpha: float, g: float = 9.81) -> float:
    """选型峰值力矩 τ_peak ≈ m g r + J α。"""
    return m * g * r + inertia * alpha


def heat(i_amp: float, r_ohm: float) -> float:
    """绕组发热 P = i² R。"""
    return i_amp**2 * r_ohm


def main() -> None:
    print("\n第一步：电机力矩和电流成正比 τ_m = K_t i")
    for i in (5.0, 10.0):
        print(f"  i = {i:>4} A → τ_m = {motor_torque(i):.2f} N·m")

    print("\n第二步：减速比 N 的四个后果（电机 τ_m = 1 N·m、4000 rpm）")
    print(f"  {'N':>5} {'输出力矩 N·m':>12} {'输出转速 rpm':>12} {'反射惯量 kg·m²':>16} {'反驱 ×':>7}")
    for n in (1, 6, 50, 160):
        g = gear(n, 1.0, MOTOR_RPM)
        print(f"  {n:>5} {g['tau_out']:>12.1f} {g['rpm_out']:>12.1f} {g['j_refl']:>16.4f} {g['backdrive_rel']:>7.0f}")
    print(f"  N = 100 时反射惯量放大 N² = {100**2} 倍")

    print("\n第二步：能量守恒 τ_m ω_m η = τ_out ω_out（N = 50）")
    omega_m = MOTOR_RPM * 2 * np.pi / 60
    pin, pout = power_balance(50, 1.0, omega_m)
    print(f"  输入 × η = {pin:.2f} W，输出 = {pout:.2f} W")

    print("\n第四步：SEA 用弹簧测力 τ = k (θ_电机侧 − θ_关节侧)")
    print(f"  k = 300 N·m/rad，两边差 0.01 rad → τ = {sea_torque(300.0, 0.51, 0.50):.2f} N·m")

    print("\n第五步：选型三个数（编的：2 kg 小臂、质心 0.3 m、J = 0.2 kg·m²、α = 10 rad/s²）")
    print(f"  峰值力矩 τ_peak ≈ m g r + J α = {peak_torque(2.0, 0.3, 0.2, 10.0):.3f} N·m")
    print(f"  发热 i² R：10 A × 0.5 Ω → {heat(10, 0.5):.0f} W；电流翻倍 → {heat(20, 0.5):.0f} W（4 倍）")
    need_rpm = 30.0
    for n in (50, 160):
        ok = need_rpm * n <= MOTOR_RPM
        print(f"  关节要 {need_rpm} rpm，N = {n} → 电机要 {need_rpm * n:.0f} rpm，{'够' if ok else '不够'}（额定 {MOTOR_RPM:.0f}）")


if __name__ == "__main__":
    main()
