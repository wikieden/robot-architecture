"""第 7 章 · 软件框架：ROS 2、中间件与实时系统。

讲义：robot-system-architecture/notes/lessons/07-ros2-middleware.md
运行：uv run python ch07_ros2_middleware.py
QoS 的数字照动画来：每个时刻发 1 条、链路每个时刻最多发 1.25 次、丢包率 35%。坐标变换的位姿是编的。
"""

import numpy as np


def expected_sends(p: float) -> float:
    """可靠传输每条平均要发 1 / (1 − p) 次。"""
    return 1.0 / (1.0 - p)


def monte_carlo_sends(p: float, n: int = 200_000, seed: int = 0) -> float:
    """模拟：每条消息一直重发直到送达，数一数平均发了几次。"""
    rng = np.random.default_rng(seed)
    return float(rng.geometric(1.0 - p, size=n).mean())


def queue_ok(rate: float, p: float, capacity: float) -> bool:
    """队列不爆的条件：发布速率 × 1/(1−p) < 链路容量。"""
    return rate * expected_sends(p) < capacity


def simulate_qos(p: float, ticks: int = 200, capacity: float = 1.25, seed: int = 1) -> dict[str, float]:
    """每个时刻发布 1 条。RELIABLE：排队、丢了重发；BEST_EFFORT：只发最新的，丢了就丢。返回最后的"数据年龄"（时刻数）。"""
    rng = np.random.default_rng(seed)
    queue: list[int] = []
    rel_last, be_last, credit = -1, -1, 0.0
    be_got = 0
    for t in range(ticks):
        queue.append(t)
        credit += capacity
        while credit >= 1.0:  # 这个时刻能发几次
            credit -= 1.0
            if queue and rng.random() > p:
                rel_last = queue.pop(0)
        if rng.random() > p:  # BEST_EFFORT 只发这一刻的新消息，一次
            be_last = t
            be_got += 1
    return {"reliable_age": ticks - 1 - rel_last, "reliable_queue": len(queue),
            "best_effort_age": ticks - 1 - be_last, "best_effort_recv": be_got / ticks}


def se2(x: float, y: float, yaw_deg: float) -> np.ndarray:
    """平面上的 3×3 齐次变换：先转 yaw，再平移 (x, y)。"""
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def main() -> None:
    print("\n第三步：RELIABLE 每条平均发几次 = 1 / (1 − p)")
    for p in (0.05, 0.35):
        print(f"  丢包率 {p:.2f}：公式 {expected_sends(p):.3f} 次，模拟 {monte_carlo_sends(p):.3f} 次")

    print("\n第三步：队列不爆的条件——发布 1 条/时刻 × 1/(1−p) < 链路 1.25 次/时刻")
    for p in (0.05, 0.35):
        need = expected_sends(p)
        print(f"  p = {p:.2f}：需要 {need:.3f} 次/时刻，{'能跟上' if queue_ok(1.0, p, 1.25) else '跟不上，队列越积越长'}")
    print(f"  临界丢包率 = 1 − 1/1.25 = {1 - 1 / 1.25:.2f}")

    print("\n动画：跑 200 个时刻，看收到的最新一帧有多旧")
    for p in (0.05, 0.35):
        r = simulate_qos(p)
        print(f"  p = {p:.2f}：RELIABLE 最新一帧旧 {r['reliable_age']:.0f} 个时刻（队列里还有 {r['reliable_queue']:.0f} 条）；"
              f"BEST_EFFORT 旧 {r['best_effort_age']:.0f} 个时刻，收到 {r['best_effort_recv']:.0%}")

    print("\n第四步：tf2 坐标变换链 A_T_C = A_T_B · B_T_C（编的位姿）")
    base_T_cam = se2(0.2, 0.0, 90.0)  # 相机装在底座前方 0.2 m，转了 90°
    p_cam = np.array([1.0, 0.0, 1.0])  # 相机系里正前方 1 m 的点
    p_base = base_T_cam @ p_cam
    print(f"  相机系 (1, 0) → base_link 系 ({p_base[0]:.2f}, {p_base[1]:.2f})")
    back = np.linalg.inv(base_T_cam) @ p_base
    print(f"  再用逆变换变回去 → ({back[0]:.2f}, {back[1]:.2f})")


if __name__ == "__main__":
    main()
