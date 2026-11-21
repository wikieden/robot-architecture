"""第 8 章 · 自主栈：从模块化流水线到端到端 VLA 与双系统。

讲义：robot-system-architecture/notes/lessons/08-autonomy-stack.md
运行：uv run python ch08_autonomy_stack.py
各段耗时照动画（VLA：编码 20 + VLM 60 + 动作头 40 ms，每 0.2 s 一圈；S1：8 ms，每 10 ms 一圈）。
模块化串行那条就是练习题 1，这里不算。
"""

import numpy as np


def reaction_time(segments_s: list[float], period_s: float, event_s: float) -> float:
    """动画的算法：第 c 圈从 c × 周期 开始，用开始那一刻的画面；事件之后开始的第一圈结束时才反应。"""
    length = sum(segments_s)
    c = int(np.ceil(event_s / period_s - 1e-9))
    return c * period_s + length - event_s


def mean_wait(period_s: float, n: int = 100_000, seed: int = 0) -> float:
    """事件在一个周期里均匀出现，平均要等多久才轮到下一次采样（期望 T/2）。"""
    rng = np.random.default_rng(seed)
    t = rng.uniform(0, period_s, n)
    return float((period_s - t).mean())


def dual_system_react(t_s1_period: float, t_s1_compute: float, t_s2_period: float) -> float:
    """异步双系统：t_react ≈ T_S1 + t_S1，和 T_S2 无关（参数留着就是为了说明它没用上）。"""
    return t_s1_period + t_s1_compute


def main() -> None:
    event = 0.30
    print("\n第五步 / 动画：第 0.30 秒杯子被碰走，多久之后开始执行新动作")
    vla = reaction_time([0.02, 0.06, 0.04], 0.2, event)
    s1 = reaction_time([0.008], 0.01, event)
    print(f"  端到端 VLA（每 0.2 s 重想一次，一次 120 ms）：{vla * 1000:.0f} ms")
    print(f"  双系统 S1（每 10 ms 一圈，一圈 8 ms）：{s1 * 1000:.0f} ms")

    print("\n第五步：事件发生的时刻不一样，VLA 的反应时间也不一样")
    for ev in (0.21, 0.30, 0.39):
        print(f"  事件在 {ev:.2f} s → {reaction_time([0.02, 0.06, 0.04], 0.2, ev) * 1000:.0f} ms")

    print("\n数学原理：等下一次采样，平均等 T/2")
    print(f"  T = 0.2 s：模拟平均等 {mean_wait(0.2):.4f} s，公式 T/2 = 0.1 s")

    print("\n公式表：异步双系统 t_react ≈ T_S1 + t_S1，和 T_S2 无关")
    for t_s2 in (0.125, 0.5, 1.0):
        print(f"  T_S2 = {t_s2:.3f} s → 最坏 {dual_system_react(0.01, 0.008, t_s2) * 1000:.0f} ms")


if __name__ == "__main__":
    main()
