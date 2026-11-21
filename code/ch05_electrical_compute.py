"""第 5 章 · 电气与计算架构：大脑、小脑、神经和血管。

讲义：robot-system-architecture/notes/lessons/05-electrical-compute.md
运行：uv run python ch05_electrical_compute.py
EtherCAT 报文 300 字节、30 个从站、电池 48 V × 10 Ah 是编的。
"""


def frame_time_s(bits: float, rate_bps: float) -> float:
    """一帧在总线上要多久：位数 ÷ 速率。"""
    return bits / rate_bps


def can_cycle_s(n_joints: int, bits_per_frame: float = 130, rate_bps: float = 1e6) -> float:
    """CAN 一问一答一圈：T ≈ n × 2 × (bits/帧 ÷ 速率)。"""
    return n_joints * 2 * frame_time_s(bits_per_frame, rate_bps)


def ecat_cycle_s(frame_bytes: float, n_slaves: int, t_slave_s: float = 1e-6, rate_bps: float = 100e6) -> float:
    """EtherCAT 一圈：T ≈ 8 × 字节数 ÷ 100 Mbps + n × 每个从站的延迟。"""
    return 8 * frame_bytes / rate_bps + n_slaves * t_slave_s


def f_max(t_cycle_s: float) -> float:
    """最高控制频率 f_max = 1 / T_cycle。"""
    return 1.0 / t_cycle_s


def battery_wh(volt: float, amp_hour: float) -> float:
    """电池能量 Wh = V × Ah。"""
    return volt * amp_hour


def main() -> None:
    print("\n第二步：CAN 一帧多久（标准帧 110–130 位，1 Mbps）")
    for bits in (110, 130):
        print(f"  {bits} 位 → {frame_time_s(bits, 1e6) * 1e3:.2f} ms")
    print(f"  每个关节发 1 帧 + 回 1 帧 ≈ {can_cycle_s(1) * 1e3:.2f} ms")

    print("\n第二步：6 个关节挂一路 CAN，一圈要多久")
    t = can_cycle_s(6)
    print(f"  T ≈ 6 × 2 × 0.13 ms = {t * 1e3:.2f} ms → 最高 {f_max(t):.0f} Hz，跑不到 1 kHz")

    print("\n第二步：EtherCAT 一帧穿过所有从站（编的：300 字节、30 个关节、每站 1 µs）")
    print(f"  传输 8 × 300 ÷ 100 Mbps = {8 * 300 / 100e6 * 1e6:.0f} µs")
    t = ecat_cycle_s(300, 30)
    print(f"  加上 30 个从站 → 一圈 {t * 1e6:.0f} µs → 最高 {f_max(t):.0f} Hz")

    print("\n第三步：电池能量 Wh = V × Ah（编的：48 V × 10 Ah）")
    print(f"  {battery_wh(48, 10):.0f} Wh")


if __name__ == "__main__":
    main()
