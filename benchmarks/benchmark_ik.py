#!/usr/bin/env python3
"""Benchmark reachable-target IK quality for the Nero Pinocchio solver."""

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
    parser.add_argument("--num-samples", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-iters", type=int, default=80)
    parser.add_argument("--pos-tol", type=float, default=1e-3)
    parser.add_argument("--ori-tol", type=float, default=1e-2)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument(
        "--log-failures",
        nargs="?",
        const=Path("ik_failures.jsonl"),
        default=None,
        type=Path,
        help="Optional JSONL path. Defaults to ik_failures.jsonl when passed without a value.",
    )
    parser.add_argument(
        "--seed-trials",
        type=int,
        default=1,
        help="Number of different q_init seeds per target for seed-sensitivity stats.",
    )
    return parser.parse_args()


def make_solver(max_iters: int) -> Pinocchio_Solver:
    return Pinocchio_Solver(
        joint_limits=DEFAULT_NERO_JOINT_LIMITS,
        dt=0.05,
        max_iterations=max_iters,
        tol_pos=1e-5,
        tol_rot=1e-4,
    )


def solve_once(solver: Pinocchio_Solver, target_pose, q_init):
    solver.init_state(q_init)
    start = time.perf_counter()
    q_solution = solver.solve(target_pose, limit_output_step=False)
    latency_ms = (time.perf_counter() - start) * 1000.0
    report = dict(solver.last_report or {})
    return q_solution, latency_ms, report


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

    position_errors = []
    orientation_errors = []
    iterations = []
    latencies_ms = []
    failure_records = []
    success_count = 0
    timeout_count = 0
    joint_limit_violation_count = 0

    seed_all_success = []
    seed_solution_spreads = []
    seed_error_spreads = []

    for idx, q_target in enumerate(
        sample_random_q(rng, DEFAULT_NERO_JOINT_LIMITS, num_samples=args.num_samples, margin=0.08)
    ):
        target_T = solver.fk_matrix(q_target)
        target_pose = solver.fk_pose(q_target)
        q_init = clip_to_joint_limits(
            q_target + rng.normal(0.0, 0.03, size=q_target.shape),
            DEFAULT_NERO_JOINT_LIMITS,
        )

        q_solution, latency_ms, report = solve_once(solver, target_pose, q_init)
        latencies_ms.append(latency_ms)
        iterations.append(int(report.get("iterations", args.max_iters)))

        pos_err = None
        ori_err = None
        if q_solution is not None:
            pos_err, ori_err = pose_errors(solver.fk_matrix(q_solution), target_T)
            position_errors.append(pos_err)
            orientation_errors.append(ori_err)
            if joint_limit_violation(q_solution, DEFAULT_NERO_JOINT_LIMITS):
                joint_limit_violation_count += 1

        if report.get("timed_out") or (
            q_solution is None and int(report.get("iterations", 0)) >= args.max_iters
        ):
            timeout_count += 1

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

        if args.seed_trials > 1:
            trial_solutions = []
            trial_errors = []
            trial_successes = []
            for _ in range(args.seed_trials):
                trial_init = sample_random_q(rng, DEFAULT_NERO_JOINT_LIMITS, num_samples=1)[0]
                trial_q, _, trial_report = solve_once(solver, target_pose, trial_init)
                if trial_q is None:
                    trial_successes.append(False)
                    if args.log_failures:
                        failure_records.append(
                            failure_record(
                                idx,
                                target_pose,
                                trial_init,
                                trial_q,
                                None,
                                None,
                                0.0,
                                trial_report,
                                "seed_sensitivity_trial_failed",
                            )
                        )
                    continue
                trial_pos, trial_ori = pose_errors(solver.fk_matrix(trial_q), target_T)
                trial_ok = trial_pos <= args.pos_tol and trial_ori <= args.ori_tol
                trial_successes.append(trial_ok)
                if trial_ok:
                    trial_solutions.append(trial_q)
                    trial_errors.append([trial_pos, trial_ori])
                elif args.log_failures:
                    failure_records.append(
                        failure_record(
                            idx,
                            target_pose,
                            trial_init,
                            trial_q,
                            trial_pos,
                            trial_ori,
                            0.0,
                            trial_report,
                            "seed_sensitivity_trial_failed",
                        )
                    )
            seed_all_success.append(bool(trial_successes and all(trial_successes)))
            if len(trial_solutions) >= 2:
                sol_arr = np.asarray(trial_solutions, dtype=float)
                err_arr = np.asarray(trial_errors, dtype=float)
                seed_solution_spreads.append(float(np.max(np.linalg.norm(sol_arr - sol_arr[0], axis=1))))
                seed_error_spreads.append(float(np.max(np.linalg.norm(err_arr - err_arr[0], axis=1))))

    latency_stats = scalar_stats(latencies_ms)
    pos_stats = scalar_stats(position_errors)
    ori_stats = scalar_stats(orientation_errors)

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
        "seed_sensitivity": {
            "seed_trials": int(args.seed_trials),
            "all_trials_success_rate": (
                float(np.mean(seed_all_success)) if seed_all_success else None
            ),
            "mean_solution_spread": (
                float(np.mean(seed_solution_spreads)) if seed_solution_spreads else None
            ),
            "max_solution_spread": (
                float(np.max(seed_solution_spreads)) if seed_solution_spreads else None
            ),
            "mean_error_spread": (
                float(np.mean(seed_error_spreads)) if seed_error_spreads else None
            ),
            "max_error_spread": (
                float(np.max(seed_error_spreads)) if seed_error_spreads else None
            ),
        },
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
