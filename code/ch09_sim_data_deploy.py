"""第 9 章 · 仿真、数据与部署闭环。

讲义：robot-system-architecture/notes/lessons/09-sim-data-deploy.md
运行：uv run python ch09_sim_data_deploy.py
成功率函数照动画里的玩具模型；系统辨识的一维模型和数据量估算的条数是编的。
"""

import numpy as np


def success(mu: float, seen: np.ndarray, lo: float, hi: float) -> float:
    """动画的玩具成功率：离最近一个训练摩擦越远越差；训练范围越宽，峰值越低（0.97 − 0.12 × 宽度 / 1.2）。"""
    width = hi - lo
    peak = 0.97 - 0.12 * width / 1.2
    sig = 0.12
    best = float(np.max(np.exp(-((mu - seen) ** 2) / (2 * sig**2))))
    cover = min(1.0, len(seen) / 12)
    return peak * cover * best


def expected_return(seen: np.ndarray, lo: float, hi: float, test: np.ndarray) -> float:
    """域随机化目标里的期望：把策略放到一批测试参数里，求平均成功率。"""
    return float(np.mean([success(m, seen, lo, hi) for m in test]))


def sysid_damping(xi_true: float = 0.8, n: int = 500, dt: float = 0.01, seed: int = 0) -> float:
    """一维系统辨识：s_{t+1} = s_t + dt (a_t − ξ s_t) + 噪声，用最小二乘估出 ξ。"""
    rng = np.random.default_rng(seed)
    s = np.zeros(n + 1)
    a = rng.uniform(-1, 1, n)
    for t in range(n):
        s[t + 1] = s[t] + dt * (a[t] - xi_true * s[t]) + rng.normal(0, 1e-4)
    # 把 (s_{t+1} − s_t)/dt − a_t = −ξ s_t 写成 y = X ξ
    y = (s[1:] - s[:-1]) / dt - a
    X = -s[:-1]
    return float(X @ y / (X @ X))


def dataset_hours(episodes: int, seconds_each: float) -> float:
    """数据量估算：小时数 = episode 数 × 每条秒数 / 3600。"""
    return episodes * seconds_each / 3600


def main() -> None:
    print("\n第三步 / 动画：窄训练（只在 μ = 0.5）vs 宽训练（0.2–1.4 均匀取 13 个）")
    narrow = np.full(13, 0.5)
    wide = np.linspace(0.2, 1.4, 13)
    for mu in (0.5, 0.9):
        print(f"  真机 μ = {mu}：窄 {success(mu, narrow, 0.5, 0.5):.0%}，宽 {success(mu, wide, 0.2, 1.4):.0%}")

    print("\n公式表：域随机化目标 E_ξ[J(π; ξ)]——在 0.2–1.4 里随机抽 2000 个真机摩擦求平均")
    test = np.random.default_rng(0).uniform(0.2, 1.4, 2000)
    print(f"  窄训练平均 {expected_return(narrow, 0.5, 0.5, test):.0%}，宽训练平均 {expected_return(wide, 0.2, 1.4, test):.0%}")

    print("\n公式表：系统辨识（编的一维模型，真实阻尼 ξ = 0.8）")
    print(f"  最小二乘估出 ξ̂ = {sysid_damping():.3f}")

    print("\n公式表：数据量估算（编的：200 条 × 每条 45 秒）")
    print(f"  {dataset_hours(200, 45):.2f} 小时")


if __name__ == "__main__":
    main()
