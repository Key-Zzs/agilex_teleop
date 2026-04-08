import time
import math
import os
import sys
import numpy as np
from pyAgxArm import create_agx_arm_config, AgxArmFactory
from pyAgxArm.utiles.tf import rpy_to_rot

# 添加 ik_solver 路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'robotic_arm_kinematics-main'))

from nero_ik.ik_solver import (
    fk,
    NeroParams,
    ContinuityParams,
    ContinuityRuntimeState,
    solve_pose_continuous_with_state,
)

def wrap_to_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def pose_error(target_pose, current_pose):
    """Compute 6D pose error [dx, dy, dz, droll, dpitch, dyaw]."""
    e = np.zeros(6, dtype=float)
    for i in range(3):
        e[i] = target_pose[i] - current_pose[i]
    for i in range(3, 6):
        e[i] = wrap_to_pi(target_pose[i] - current_pose[i])
    return e

def build_joint_limits_from_cfg(cfg: dict):
    """
    从 create_agx_arm_config() 的 cfg 中提取 7 轴关节限位。
    返回格式: [(lo1, hi1), ..., (lo7, hi7)]
    """
    limits = []
    for i in range(1, 8):
        lo, hi = cfg["joint_limits"][f"joint{i}"]
        limits.append((float(lo), float(hi)))
    return limits

class Solver:
    """
    基于 ik_solver.py 的解析 IK 求解器
    使用 ik_arm_angle_with_report 进行单帧求解
    """
    def __init__(self, joint_limits, dt, n_psi=181):
        self.joint_limits = joint_limits
        self.dt = dt
        self.n_psi = n_psi
        
        # 使用默认的 NERO DH 参数
        self.nero_params = NeroParams.default()
        
        # 连续性参数
        self.continuity = ContinuityParams()
        
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
    
    def init_state(self, current_q):
        """初始化求解器状态（仅调用一次）"""
        current_q = self._clamp_joints(np.array(current_q, dtype=float))
        self.state = ContinuityRuntimeState(q_prev=current_q)
    
    def solve(self, target_pose):
        """
        求解目标位姿对应的关节角
        :param target_pose: 6D pose [x, y, z, roll, pitch, yaw]
        :return: 7维关节角
        """
        T_target = self._pose_to_matrix(target_pose)
        
        # 使用 solve_pose_continuous_with_state 求解
        q_cmd, report, self.state = solve_pose_continuous_with_state(
            T_target, state=self.state, p=self.nero_params, n_psi=self.n_psi, continuity=self.continuity
        )
        
        if q_cmd is None:
            # IK 求解失败，返回上一帧的关节角
            print(f"⚠️ IK 求解失败: {report.get('method')}")
            print(f"   目标位姿: x={target_pose[0]:.3f}, y={target_pose[1]:.3f}, z={target_pose[2]:.3f}")
            print(f"   候选解数量: {report.get('candidate_count', 0)}")
            return self.state.q_prev.copy()
        
        # 关节限位裁剪
        q_cmd = self._clamp_joints(q_cmd)
        
        return q_cmd