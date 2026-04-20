import time
import math
import os
import sys
import numpy as np
from dataclasses import replace
from pyAgxArm import create_agx_arm_config, AgxArmFactory
from pyAgxArm.utiles.tf import rpy_to_rot

# 添加 ik_solver 路径
# sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'kinematics'))

from nero.kinematics.nero_kinematics.nero_ik.ik_solver import (
    fk,
    NeroParams,
    ContinuityParams,
    ContinuityRuntimeState,
    solve_pose_continuous_with_state,
)

class Solver:
    """
    基于 ik_solver.py 的解析 IK 求解器
    使用 ik_arm_angle_with_report 进行单帧求解
    
    性能优化：
    - n_psi: 全局扫描点数，默认61（原181）
    - local_theta0_count: 局部窗口点数，默认21（原41）
    - 禁用1D QP优化（额外开销）
    """
    def __init__(self, joint_limits, dt, n_psi=61):
        self.joint_limits = joint_limits
        self.dt = dt
        self.n_psi = n_psi  # 减少扫描点数：181→61
        
        # 使用默认的 NERO DH 参数
        self.nero_params = NeroParams.default()
        
        # 连续性参数 - 优化性能
        self.continuity = ContinuityParams(
            # 适度缩小局部臂角窗口，降低跨分支切换概率
            local_theta0_window=0.25,
            local_theta0_count=21,  # 减少局部扫描点：41→21
            # 连续性项权重上调，优先轨迹平滑
            w_vel=1.6,
            w_acc=0.55,
            w_pose=0.1,
            w_theta0=0.35,
            hysteresis_margin=0.08,
            # Default-off to avoid expensive fallback in the fast servo loop.
            enable_global_fallback=False,
            w_qp_joint_inc=0.0,  # 禁用QP优化
            w_qp_pose_err=0.0,   # 禁用QP优化
        )
        self._base_local_theta0_window = float(self.continuity.local_theta0_window)
        self._base_local_theta0_count = int(self.continuity.local_theta0_count)
        self._max_local_theta0_window = 0.45
        self._max_local_theta0_count = 33
        # Adaptive policy: only allow global fallback after multiple consecutive failures.
        self._consecutive_failures = 0
        self._fallback_after_failures = 4

        # 输出侧防跳变参数（rad/s 与 rad）
        # 若知道各轴最大安全速度，建议按实机参数配置。
        self.max_joint_vel = np.array([2.2, 2.0, 2.2, 2.2, 2.6, 2.6, 3.0], dtype=float)
        self.min_step_limit = 0.03
        self.jump_detect_scale = 3.0
        self.hard_jump_limit = 0.90

        # 最近一次求解诊断
        self.last_report = None
        self.last_jump_report = None
        
        # 运行时状态
        self.state = None
    
    def _pose_to_matrix(self, pose):
        """将 6D pose [x, y, z, roll, pitch, yaw] 转换为 4x4 齐次变换矩阵"""
        T = np.eye(4, dtype=float)
        T[:3, :3] = np.array(rpy_to_rot(pose[3], pose[4], pose[5]), dtype=float)
        T[:3, 3] = np.array(pose[:3], dtype=float)
        return T
    
    def _clamp_joints(self, q):
        """关节限位裁剪"""
        q_out = np.array(q, dtype=float)
        for i, (lo, hi) in enumerate(self.joint_limits):
            q_out[i] = min(max(q_out[i], lo), hi)
        return q_out

    def _compute_step_limit(self):
        """根据速度上限与控制周期，得到每个关节单步最大改变量。"""
        return np.maximum(self.max_joint_vel * float(self.dt), self.min_step_limit)

    def _detect_and_guard_output(self, q_cmd):
        """检测并抑制关节角跳变，返回 (q_safe, jump_report)。"""
        q_cmd = np.array(q_cmd, dtype=float)
        if self.state is None or self.state.q_prev is None:
            return self._clamp_joints(q_cmd), {
                "jump_detected": False,
                "joint_indices": [],
                "dq_raw": [0.0] * 7,
                "dq_limited": [0.0] * 7,
                "mode": "no_prev_state",
            }

        q_prev = np.array(self.state.q_prev, dtype=float)
        dq_raw = np.array((q_cmd - q_prev), dtype=float)
        dq_raw = (dq_raw + np.pi) % (2.0 * np.pi) - np.pi

        step_limit = self._compute_step_limit()
        detect_limit = np.maximum(step_limit * self.jump_detect_scale, self.min_step_limit)

        jump_mask = np.abs(dq_raw) > detect_limit
        very_large_jump = np.any(np.abs(dq_raw) > self.hard_jump_limit)

        dq_limited = np.clip(dq_raw, -step_limit, step_limit)
        q_safe = self._clamp_joints(q_prev + dq_limited)

        if very_large_jump:
            # 极端跳变时冻结到上一次状态，避免打杆。
            q_safe = q_prev.copy()

        jump_report = {
            "jump_detected": bool(np.any(jump_mask)),
            "joint_indices": np.where(jump_mask)[0].astype(int).tolist(),
            "dq_raw": dq_raw.astype(float).tolist(),
            "dq_limited": dq_limited.astype(float).tolist(),
            "step_limit": step_limit.astype(float).tolist(),
            "detect_limit": detect_limit.astype(float).tolist(),
            "very_large_jump": bool(very_large_jump),
            "mode": "freeze" if very_large_jump else "rate_limit",
        }
        return q_safe, jump_report
    
    def init_state(self, current_q):
        """初始化求解器状态（仅调用一次）"""
        current_q = self._clamp_joints(np.array(current_q, dtype=float))
        self.state = ContinuityRuntimeState(q_prev=current_q)
    
    def solve(self, target_pose):
        """
        求解目标位姿对应的关节角
        :param target_pose: 6D pose [x, y, z, roll, pitch, yaw]
        :return: 7维关节角，失败返回 None
        """
        T_target = self._pose_to_matrix(target_pose)

        # Adapt local window/count when failures accumulate to improve local solve hit-rate.
        fail_k = self._consecutive_failures
        run_continuity = replace(self.continuity)
        if fail_k > 0:
            run_continuity.local_theta0_window = min(
                self._max_local_theta0_window,
                self._base_local_theta0_window + 0.04 * fail_k,
            )
            run_continuity.local_theta0_count = min(
                self._max_local_theta0_count,
                self._base_local_theta0_count + 2 * fail_k,
            )
        run_continuity.enable_global_fallback = fail_k >= self._fallback_after_failures
        
        # 使用 solve_pose_continuous_with_state 求解
        q_cmd, report, new_state = solve_pose_continuous_with_state(
            T_target,
            state=self.state,
            p=self.nero_params,
            n_psi=self.n_psi,
            continuity=run_continuity,
        )
        self.last_report = report
        
        if q_cmd is None:
            self._consecutive_failures += 1
            # IK 求解失败，不更新状态，返回 None
            print(f"[IK] solve failed: {report.get('method')}")
            print(f"   目标位姿: x={target_pose[0]:.3f}, y={target_pose[1]:.3f}, z={target_pose[2]:.3f}")
            print(f"   候选解数量: {report.get('candidate_count', 0)}")
            return None
        self._consecutive_failures = 0
        
        # 关节限位裁剪
        q_cmd_clamped = self._clamp_joints(q_cmd)

        # 输出侧跳变检测与抑制
        q_out, jump_report = self._detect_and_guard_output(q_cmd_clamped)
        self.last_jump_report = jump_report
        if jump_report["jump_detected"]:
            idx_str = ",".join(str(i + 1) for i in jump_report["joint_indices"])
            print(f"[IK] jump detected on joints [{idx_str}], guard mode={jump_report['mode']}")
        
        # 成功时更新状态（使用裁剪后的关节角）
        self.state = ContinuityRuntimeState(
            q_prev=q_out,
            q_prev2=self.state.q_prev.copy() if self.state.q_prev is not None else q_out.copy(),
            theta0_prev=new_state.theta0_prev,
            q_lock=q_out,
        )
        
        return q_out