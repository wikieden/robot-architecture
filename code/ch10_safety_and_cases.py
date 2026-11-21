"""第 10 章 · 安全、可靠性与整机案例。

讲义：robot-system-architecture/notes/lessons/10-safety-and-cases.md
运行：uv run python ch10_safety_and_cases.py
停止距离用的是编的数（0.5 m/s、100 ms、2 m/s²），不是练习题 1 的数；力矩序列、阈值、失效概率也是编的。
"""

import numpy as np

NORMAL, REDUCED, SS1, STO = "正常", "降速", "保护停止 SS1", "STO"


def stop_distance(v: float, t_react: float, a: float) -> float:
    """停止距离 d = v t_react + v² / (2a)：反应时间里匀速走，之后匀减速。"""
    return v * t_react + v**2 / (2 * a)


def detect_collision(tau_meas: np.ndarray, tau_model: np.ndarray, tau_th: float) -> int:
    """碰撞检测：第一次 |τ_meas − τ_model| > τ_th 的下标；没有返回 −1。"""
    hit = np.nonzero(np.abs(tau_meas - tau_model) > tau_th)[0]
    return int(hit[0]) if hit.size else -1


def step(state: str, event: str) -> str:
    """故障降级状态机（讲义第三步那张图）。"""
    if event == "急停":
        return STO
    if state == STO:
        return NORMAL if event == "人工复位" else STO
    if event in ("心跳丢失", "力矩异常", "温度过高", "接触力超限"):
        return SS1
    if state == SS1:
        return NORMAL if event == "复位确认" else SS1
    if event == "有人进入":
        return REDUCED
    if state == REDUCED and event == "人离开":
        return NORMAL
    return state


def watchdog(heartbeats: list[int], max_missed: int = 3) -> int:
    """看门狗：连续 max_missed 拍没收到心跳（0）就触发，返回触发的那一拍；不触发返回 −1。"""
    missed = 0
    for k, hb in enumerate(heartbeats):
        missed = 0 if hb else missed + 1
        if missed >= max_missed:
            return k
    return -1


def main() -> None:
    print("\n公式表：停止距离 d = v t_react + v²/(2a)（编的：0.5 m/s、100 ms、2 m/s²）")
    d = stop_distance(0.5, 0.1, 2.0)
    print(f"  反应时间里走 {0.5 * 0.1:.4f} m + 减速走 {0.5**2 / 4:.4f} m = {d:.4f} m")

    print("\n第三步：碰撞检测 |τ_meas − τ_model| > τ_th（阈值 2 N·m）")
    model = np.full(10, 5.0)
    meas = model + np.array([0.1, -0.2, 0.3, 0.0, 0.2, 2.6, 4.0, 4.5, 4.2, 4.0])
    print(f"  第一次超阈值在第 {detect_collision(meas, model, 2.0)} 拍（从 0 数）")

    print("\n第三步：看门狗——连续 3 拍没心跳就进入保护停止")
    hb = [1, 1, 1, 0, 1, 0, 0, 0, 1]
    print(f"  心跳 {hb} → 第 {watchdog(hb)} 拍触发")

    print("\n动画：一串事件走过状态机")
    s = NORMAL
    for ev in ["有人进入", "人离开", "心跳丢失", "复位确认", "急停", "复位确认", "人工复位"]:
        s = step(s, ev)
        print(f"  {ev:<6} → {s}")

    print("\n数学原理：两路独立冗余，都失效的概率相乘（编的：每路 1e-3）")
    print(f"  {1e-3:.0e} × {1e-3:.0e} = {1e-3 * 1e-3:.0e}")


if __name__ == "__main__":
    main()
