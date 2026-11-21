"""第 1 章 · 总图：一台机器人 = 一组快慢嵌套的闭环。

讲义：robot-system-architecture/notes/lessons/01-layered-loops.md
运行：uv run python ch01_layered_loops.py
"""

import numpy as np

# 第 3 步那张图里每层的典型频率（Hz）。范围取了一个代表值：L4 取 1 Hz，L3 取 30 Hz，L2 取 10 Hz，L1 取 500 Hz，L0 取 20 kHz。
LAYERS = {
    "L4 任务规划": 1.0,
    "L3 感知": 30.0,
    "L2 策略": 10.0,
    "L1 关节控制": 500.0,
    "L0 电机驱动": 20_000.0,
}


def period_ms(f_hz: float) -> float:
    """周期 T = 1 / f，换成毫秒。"""
    return 1000.0 / f_hz


def orders_of_magnitude(slow_s: float, fast_s: float) -> float:
    """两个时间差了几个数量级：log10(慢 / 快)。"""
    return float(np.log10(slow_s / fast_s))


def steps_between(t_slow_s: float, t_fast_s: float) -> int:
    """慢层每给一次目标，快层要执行几次（含给新目标那一次）。"""
    return round(t_slow_s / t_fast_s)


def track(target: float, alpha: float, steps: int, x0: float = 0.0) -> np.ndarray:
    """一阶追赶 x_{k+1} = x_k + α (x* − x_k)，返回 x_0 … x_steps。"""
    xs = [x0]
    for _ in range(steps):
        xs.append(xs[-1] + alpha * (target - xs[-1]))
    return np.array(xs)


def main() -> None:
    print("\n第一步 / 第三步：每层的周期 T = 1/f")
    for name, f in LAYERS.items():
        print(f"  {name:<8} {f:>8.0f} Hz → 周期 {period_ms(f):>8.3f} ms")

    print("\n第二步：为什么不能一层到底——电流环和 VLA 差几个数量级")
    t_current = 100e-6  # 电流环 10 kHz → 100 µs（讲义 25–100 µs 取慢的一头）
    t_vla = 100e-3  # VLA 一次推理按 100 ms 算
    print(f"  VLA 100 ms ÷ 电流环 100 µs = {t_vla / t_current:.0f} 倍 = {orders_of_magnitude(t_vla, t_current):.0f} 个数量级")

    print("\n第四步：插补——L2 每 100 ms 给一个目标，L1 每 1 ms 要一个指令")
    n = steps_between(0.1, 0.001)
    print(f"  L1 每个目标要跑 {n} 次，其中 {n - 1} 次要靠插补补点")

    print("\n第四步：一阶追赶 x_{k+1} = x_k + α (x* − x_k)，目标 x* = 1")
    for alpha in (0.1, 0.5):
        xs = track(1.0, alpha, 10)
        print(f"  α = {alpha}: 前 5 步 {np.round(xs[:6], 3).tolist()}  第 10 步 {xs[10]:.3f}")
    xs = track(1.0, 1.5, 6)
    print(f"  α = 1.5（太大）: {np.round(xs, 3).tolist()} ← 过冲、来回抖")
    print("  误差每步乘 (1 − α)：α = 0.5 时第 10 步误差 = 0.5^10 =", round(0.5**10, 5))


if __name__ == "__main__":
    main()
