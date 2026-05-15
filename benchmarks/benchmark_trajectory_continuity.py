#!/usr/bin/env python3
"""Benchmark warm-start IK continuity along a reachable joint-space trajectory."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    import pinocchio  # noqa: F401
except ModuleNotFoundError as exc:
    raise SystemExit("Pinocchio is required. Install the project dynamics extra first.") from exc

from nero.kinematics.analytic_IK_solver import Pinocchio_Solver
from nero.kinematics.debug_tools import (
    DEFAULT_NERO_JOINT_LIMITS,
    clip_to_joint_limits,
    joint_limit_violation,
    pose_errors,
    sample_random_q,
    scalar_stats,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--num-samples", type=int, default=300)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-iters", type=int, default=80)
    parser.add_argument("--pos-tol", type=float, default=1e-3)
    parser.add_argument("--ori-tol", type=float, default=1e-2)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument(
        "--log-failures",
        nargs="?",
        const=Path("trajectory_ik_failures.jsonl"),
        default=None,
        type=Path,
        help="Optional JSONL path. Defaults to trajectory_ik_failures.jsonl when passed without a value.",
    )
    parser.add_argument("--joint-step-std", type=float, default=0.015)
    parser.add_argument("--jump-threshold", type=float, default=0.25)
    return parser.parse_args()


def make_solver(max_iters: int) -> Pinocchio_Solver:
    return Pinocchio_Solver(
        joint_limits=DEFAULT_NERO_JOINT_LIMITS,
        dt=0.05,
        max_iterations=max_iters,
        tol_pos=1e-5,
        tol_rot=1e-4,
    )


def build_joint_trajectory(rng: np.random.Generator, num_samples: int, step_std: float) -> np.ndarray:
    q = sample_random_q(rng, DEFAULT_NERO_JOINT_LIMITS, num_samples=1, margin=0.25)[0]
    path = [q.copy()]
    for _ in range(1, num_samples):
        q = clip_to_joint_limits(
            q + rng.normal(0.0, step_std, size=q.shape),
            DEFAULT_NERO_JOINT_LIMITS,
        )
        path.append(q.copy())
    return np.asarray(path, dtype=float)


def failure_record(index, target_pose, q_init, q_solution, pos_err, ori_err, latency_ms, report, reason):
    return {
        "index": int(index),
        "target_pose": np.asarray(target_pose, dtype=float).tolist(),
        "q_init": np.asarray(q_init, dtype=float).tolist(),
        "best_q": report.get("best_q"),
        "last_q": report.get("last_q"),
        "q_solution": None if q_solution is None else np.asarray(q_solution, dtype=float).tolist(),
        "position_error": None if pos_err is None else float(pos_err),
        "orientation_error": None if ori_err is None else float(ori_err),
        "iterations": int(report.get("iterations", 0)),
        "reason": reason,
        "solver_report": report,
        "solve_time_ms": float(latency_ms),
    }


def main():
    args = parse_args()
    rng = np.random.default_rng(args.seed)
    solver = make_solver(args.max_iters)
    q_path = build_joint_trajectory(rng, args.num_samples, args.joint_step_std)

    position_errors = []
    orientation_errors = []
    iterations = []
    latencies_ms = []
    joint_step_norms = []
    failure_records = []
    success_count = 0
    timeout_count = 0
    joint_limit_violation_count = 0

    q_prev_solution = q_path[0] + rng.normal(0.0, 0.01, size=q_path[0].shape)
    q_prev_solution = clip_to_joint_limits(q_prev_solution, DEFAULT_NERO_JOINT_LIMITS)
    solver.init_state(q_prev_solution)

    for idx, q_target in enumerate(q_path):
        target_T = solver.fk_matrix(q_target)
        target_pose = solver.fk_pose(q_target)
        q_init = np.asarray(solver.state.q_prev, dtype=float).copy()

        start = time.perf_counter()
        q_solution = solver.solve(target_pose, limit_output_step=False)
        latency_ms = (time.perf_counter() - start) * 1000.0
        report = dict(solver.last_report or {})

        latencies_ms.append(latency_ms)
        iterations.append(int(report.get("iterations", args.max_iters)))
        if report.get("timed_out") or (
            q_solution is None and int(report.get("iterations", 0)) >= args.max_iters
        ):
            timeout_count += 1

        pos_err = None
        ori_err = None
        if q_solution is not None:
            pos_err, ori_err = pose_errors(solver.fk_matrix(q_solution), target_T)
            position_errors.append(pos_err)
            orientation_errors.append(ori_err)
            if joint_limit_violation(q_solution, DEFAULT_NERO_JOINT_LIMITS):
                joint_limit_violation_count += 1
            if q_prev_solution is not None:
                joint_step_norms.append(float(np.linalg.norm(q_solution - q_prev_solution)))

        ok = (
            q_solution is not None
            and pos_err is not None
            and ori_err is not None
            and pos_err <= args.pos_tol
            and ori_err <= args.ori_tol
            and not joint_limit_violation(q_solution, DEFAULT_NERO_JOINT_LIMITS)
        )
        if ok:
            success_count += 1
            q_prev_solution = np.asarray(q_solution, dtype=float).copy()
        else:
            reason = "solver_failed" if q_solution is None else "tolerance_or_joint_limit"
            failure_records.append(
                failure_record(
                    idx,
                    target_pose,
                    q_init,
                    q_solution,
                    pos_err,
                    ori_err,
                    latency_ms,
                    report,
                    reason,
                )
            )

    latency_stats = scalar_stats(latencies_ms)
    pos_stats = scalar_stats(position_errors)
    ori_stats = scalar_stats(orientation_errors)
    step_stats = scalar_stats(joint_step_norms)
    jump_count = int(np.sum(np.asarray(joint_step_norms, dtype=float) > args.jump_threshold))

    results = {
        "num_samples": int(args.num_samples),
        "success_count": int(success_count),
        "success_rate": float(success_count / max(1, args.num_samples)),
        "mean_position_error": pos_stats["mean"],
        "median_position_error": pos_stats["median"],
        "max_position_error": pos_stats["max"],
        "mean_orientation_error": ori_stats["mean"],
        "median_orientation_error": ori_stats["median"],
        "max_orientation_error": ori_stats["max"],
        "mean_iterations": float(np.mean(iterations)) if iterations else float("nan"),
        "max_iterations": int(np.max(iterations)) if iterations else 0,
        "mean_latency_ms": latency_stats["mean"],
        "median_latency_ms": latency_stats["median"],
        "p90_latency_ms": latency_stats["p90"],
        "p95_latency_ms": latency_stats["p95"],
        "p99_latency_ms": latency_stats["p99"],
        "max_latency_ms": latency_stats["max"],
        "timeout_rate": float(timeout_count / max(1, args.num_samples)),
        "joint_limit_violation_rate": float(joint_limit_violation_count / max(1, args.num_samples)),
        "mean_joint_step_norm": step_stats["mean"],
        "median_joint_step_norm": step_stats["median"],
        "p95_joint_step_norm": step_stats["p95"],
        "max_joint_step_norm": step_stats["max"],
        "configuration_jump_count": int(jump_count),
        "configuration_jump_rate": float(jump_count / max(1, len(joint_step_norms))),
        "jump_threshold": float(args.jump_threshold),
    }

    print(json.dumps(results, indent=2, sort_keys=True))

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if args.log_failures:
        args.log_failures.parent.mkdir(parents=True, exist_ok=True)
        with args.log_failures.open("w", encoding="utf-8") as f:
            for record in failure_records:
                f.write(json.dumps(record, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
