#!/usr/bin/env python3
"""
Performance and Latency Benchmarking for Autonomous Vehicle Stack.
Measures execution time for perception, decision planning, PID control,
and simulation updates across 1,000 iterations.
"""

import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.config import DEFAULT_CONFIG
from src.simulation.simulator import AutonomousCarSimulator


def run_benchmark(num_steps: int = 1000):
    print("=" * 60)
    print(f"⏱️ RUNNING BENCHMARK ({num_steps} SIMULATION STEPS)")
    print("=" * 60)

    sim = AutonomousCarSimulator(DEFAULT_CONFIG)
    latencies = []

    for _ in range(num_steps):
        t0 = time.perf_counter()
        sim.step(dt=0.05)
        t1 = time.perf_counter()
        latencies.append((t1 - t0) * 1000.0) # ms

    mean_ms = sum(latencies) / len(latencies)
    sorted_l = sorted(latencies)
    p50_ms = sorted_l[len(sorted_l) // 2]
    p95_ms = sorted_l[int(len(sorted_l) * 0.95)]
    p99_ms = sorted_l[int(len(sorted_l) * 0.99)]
    max_fps = 1000.0 / mean_ms if mean_ms > 0 else 0

    print(f"Total Iterations:       {num_steps}")
    print(f"Mean Step Latency:      {mean_ms:.3f} ms")
    print(f"Median (p50) Latency:   {p50_ms:.3f} ms")
    print(f"95th Percentile:        {p95_ms:.3f} ms")
    print(f"99th Percentile:        {p99_ms:.3f} ms")
    print(f"Theoretical Max Rate:   {max_fps:.1f} FPS")
    print("=" * 60)
    if mean_ms < 10.0:
        print("✅ PASS: Real-time latency requirements satisfied (< 10 ms per loop).")
    else:
        print("⚠️ WARNING: Loop latency exceeds 10 ms target.")


if __name__ == "__main__":
    run_benchmark(1000)
