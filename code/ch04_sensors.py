"""第 4 章 · 传感器：知道自己在哪，看见外面的世界。

讲义：robot-system-architecture/notes/lessons/04-sensors.md
运行：uv run python ch04_sensors.py
互补滤波的信号（真实角度、陀螺零偏、加速度计噪声）和相机参数都是编的。
"""

import numpy as np


def delay_error(v: float, dt: float) -> float:
    """延迟误差 e = v · Δt。"""
    return v * dt


def compensate(x_obs: float, v: float, dt: float) -> float:
    """延迟补偿 x̂ = x_obs + v · Δt。"""
    return x_obs + v * dt


def project(point: np.ndarray, fx: float, fy: float, cx: float, cy: float) -> tuple[float, float]:
    """针孔相机投影 u = f_x X / Z + c_x，v = f_y Y / Z + c_y。"""
    X, Y, Z = point
    return fx * X / Z + cx, fy * Y / Z + cy


def complementary(gyro: np.ndarray, acc_angle: np.ndarray, dt: float, alpha: float) -> np.ndarray:
    """互补滤波 θ̂_k = α (θ̂_{k−1} + ω_k Δt) + (1 − α) θ_acc,k。"""
    est = np.zeros_like(gyro)
    est[0] = acc_angle[0]
    for k in range(1, len(gyro)):
        est[k] = alpha * (est[k - 1] + gyro[k] * dt) + (1 - alpha) * acc_angle[k]
    return est


def fake_imu(n: int = 2000, dt: float = 0.005, seed: int = 0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """编一段 10 秒的一维姿态：真实角度慢慢摆，陀螺有零偏 0.02 rad/s，加速度计噪声 0.05 rad。"""
    rng = np.random.default_rng(seed)
    t = np.arange(n) * dt
    truth = 0.3 * np.sin(0.5 * t)
    gyro = np.gradient(truth, dt) + 0.02  # 角速度 + 零偏
    acc = truth + rng.normal(0, 0.05, n)  # 角度 + 白噪声
    return truth, gyro, acc


def main() -> None:
    print("\n第三步：时间同步——延迟误差 e = v Δt")
    e = delay_error(0.5, 0.060)
    print(f"  传送带 0.5 m/s、延迟 60 ms → 差 {e * 100:.1f} cm")
    print(f"  按时间戳补偿：图里看到 x = 0.200 m → 预测 x̂ = {compensate(0.200, 0.5, 0.060):.3f} m")
    print(f"  延迟拖到 0：差 {delay_error(0.5, 0.0) * 100:.1f} cm")

    print("\n第四步：互补滤波——只用陀螺、只用加速度计、两者融合")
    dt = 0.005
    truth, gyro, acc = fake_imu(dt=dt)
    gyro_only = truth[0] + np.cumsum(gyro) * dt
    fused = complementary(gyro, acc, dt, alpha=0.95)

    def rms(x: np.ndarray) -> float:
        return float(np.sqrt(np.mean((x - truth) ** 2)))

    print(f"  只积分陀螺：10 秒后误差 {abs(gyro_only[-1] - truth[-1]):.3f} rad（零偏越积越大）")
    print(f"  只用加速度计：均方根误差 {rms(acc):.3f} rad（每一帧都抖）")
    print(f"  互补滤波 α = 0.95：均方根误差 {rms(fused):.3f} rad")

    print("\n公式表：针孔投影（编的：f = 600 像素，主点 (320, 240)）")
    u, v = project(np.array([0.1, -0.05, 1.0]), 600, 600, 320, 240)
    print(f"  相机系里 (0.1, −0.05, 1.0) m 的点 → 像素 (u, v) = ({u:.0f}, {v:.0f})")


if __name__ == "__main__":
    main()
