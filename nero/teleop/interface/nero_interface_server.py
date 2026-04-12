'''
Nero dual-arm robot interface server.
Provides zerorpc interface for dual-arm control.
'''

# python nero_interface/nero_interface_server.py --ip 0.0.0.0 --port 4242
# sudo iptables -I INPUT -p tcp --dport 4242 -j ACCEPT

# Version 2: Asynchronous callback version

import zerorpc
import numpy as np
import logging
import time
import math
from typing import Optional, List
import sys, os
import pdb
import threading

from nero.kinematics.analytic_IK_solver import Solver
from nero.kinematics.nero_kinematics.nero_ik.ik_solver import fk

log = logging.getLogger(__name__)

# 手动实现四元数乘法 (输入输出均为 [x, y, z, w] 格式)
def _quat_multiply(q1, q2):
    """四元数乘法，输入输出格式均为 [x, y, z, w]"""
    x1, y1, z1, w1 = q1  # [x, y, z, w]
    x2, y2, z2, w2 = q2  # [x, y, z, w]
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,  # x
        w1*y2 - x1*z2 + y1*w2 + z1*x2,  # y
        w1*z2 + x1*y2 - y1*x2 + z1*w2,  # z
        w1*w2 - x1*x2 - y1*y2 - z1*z2   # w
    )
class NeroDualArmServer:
    """Dual-arm Nero server interface."""
    
    # ==================== Robots and IK solvers ====================
    def __init__(self, gripper_enabled: bool = True):
        self.gripper_enabled = gripper_enabled

        # Initialize left arm
        self.left_robot = None
        
        try:
            from pyAgxArm import create_agx_arm_config, AgxArmFactory
            self.left_cfg = create_agx_arm_config(robot="nero", comm="can", channel="can_left")
            self.left_robot = AgxArmFactory.create_arm(self.left_cfg)
            self.left_robot.connect()

            # Enable all joints
            start_t = time.monotonic()
            while not self.left_robot.enable(255):
                if time.monotonic() - start_t > 5.0:
                    log.warning("[SERVER] Left arm enable timeout")
                    break
                time.sleep(0.01)
            log.info("[SERVER] Left arm connected and enabled")
            
        except Exception as e:
            log.error(f"[SERVER] Failed to connect to left arm: {e}")

        # Initialize right arm
        self.right_robot = None
        
        try:
            from pyAgxArm import create_agx_arm_config, AgxArmFactory
            self.right_cfg = create_agx_arm_config(robot="nero", comm="can", channel="can_right")
            self.right_robot = AgxArmFactory.create_arm(self.right_cfg)
            self.right_robot.connect()

            # Enable all joints
            start_t = time.monotonic()
            while not self.right_robot.enable(255):
                if time.monotonic() - start_t > 5.0:
                    log.warning("[SERVER] Right arm enable timeout")
                    break
                time.sleep(0.01)
            log.info("[SERVER] Right arm connected and enabled")
            
        except Exception as e:
            log.error(f"[SERVER] Failed to connect to right arm: {e}")
        
        log.info("=" * 50)
        log.info("Nero Dual-Arm Server Ready")
        log.info("=" * 50)

        # Initialize left gripper
        self.left_gripper = None

        if gripper_enabled:
            try:
                if self.left_gripper is None:
                    self.left_gripper = self.left_robot.init_effector(self.left_robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
                    log.info("[SERVER] Left gripper initialized")
            except Exception as e:
                log.error(f"[SERVER] Failed to initialize left gripper: {e}")
        
        # Initialize right gripper
        self.right_gripper = None

        if gripper_enabled:
            try:
                if self.right_gripper is None:
                    self.right_gripper = self.right_robot.init_effector(self.right_robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
            except Exception as e:
                log.error(f"[SERVER] Failed to initialize right gripper: {e}")

        log.info("=" * 50)
        log.info("Nero Dual-Gripper Server Ready")
        log.info("=" * 50)

        # Initialize IK solver
        self.left_ik_solver = None
        self.right_ik_solver = None
        self.track_freq = 20.0
        self.dt = 1.0 / self.track_freq
        # servo_p 开环控制记录的当前位姿

        try:
            self.left_ik_solver = self._setup_ik_solver(self.left_robot, self.left_cfg, "Left Arm")
            self.right_ik_solver = self._setup_ik_solver(self.right_robot, self.right_cfg, "Right Arm")
        except Exception as e:
            log.error(f"[ERROR] IK solvers init failed: {e}")

        # target_state
        ## ===== home任务 =====
        self.left_go_home_flag = False
        self.right_go_home_flag = False
        self.go_home_flag = False
        ## ===== 位姿目标 =====
        self.target_pose_left = None
        self.target_pose_right = None
        # ===== gripper目标 =====
        self.left_gripper_target = None
        self.right_gripper_target = None

        ## ===== 防止重复触发 =====
        self.left_gripper_busy = False
        self.right_gripper_busy = False
        self.left_home_busy = False
        self.right_home_busy = False

        # control thread related
        self.control_freq = 100.0
        self.control_dt = 1.0 / self.control_freq

        self.control_running = True

        import threading
        self.control_thread = threading.Thread(
            target=self.control_loop,
            daemon=True
        )
        self.control_thread.start()
        log.info("[CONTROL] Control loop started (100Hz)")

    # ==================== Inverse Kinematics setup ====================

    def _setup_ik_solver(self, robot, cfg, name: str, timeout_sec: float = 2.0):
        """辅助方法：获取初始关节角，提取限位，并初始化 IK Solver"""
        log.info(f"[{name}] 正在获取当前关节角作为 IK 初始基准...")
        current_pose = None
        current_joints = None
        start_t = time.monotonic()
        while current_pose is None or current_joints is None:
            if time.monotonic() - start_t > timeout_sec:
                log.warning(f"[{name}] get flange/joint pose timeout after {timeout_sec}s, using default pose")
                current_pose = [0.0] * 6
                current_joints = [0.0] * 7
                break
            fp = robot.get_flange_pose()
            ja = robot.get_joint_angles()
            if fp is not None: current_pose = fp.msg
            if ja is not None: current_joints = ja.msg
            time.sleep(0.1)

        # 获取关节限位
        joint_limits = []
        for i in range(1, 8):
            lo, hi = cfg["joint_limits"][f"joint{i}"]
            joint_limits.append((lo, hi))

        # 实例化解析 IK 求解器
        ik_solver = Solver(
            joint_limits=joint_limits,
            dt=self.dt,
            n_psi=181,
        )

        # 机器人的真实状态给 IK 求解器初始化
        ik_solver.init_state(current_joints)
        log.info(f"[{name}] IK Solver 初始化完成！初始关节角: {np.array(current_joints).round(3)}")
        
        return ik_solver

    # ==================== Left Arm State Query ====================

    def left_robot_get_joint_positions(self) -> list:
        """Get left arm joint positions (radians)."""
        if self.left_robot is None:
            return [0.0] * 7
        result = self.left_robot.get_joint_angles()
        return result.msg if result is not None else [0.0] * 7
    
    def left_robot_get_joint_velocities(self) -> list:
        """Get left arm joint velocities (rad/s)."""
        if self.left_robot is None:
            return [0.0] * 7
        velocities = []
        for i in range(1, 8):
            result = self.left_robot.get_motor_states(i)
            if result is not None:
                velocities.append(result.msg.velocity)
            else:
                velocities.append(0.0)
        return velocities
    
    def left_robot_get_ee_pose(self) -> list:
        """Get left arm end-effector pose [x, y, z, roll, pitch, yaw] (m, rad)."""
        if self.left_robot is None:
            return [0.0] * 6
        result = self.left_robot.get_tcp_pose()
        return result.msg if result is not None else [0.0] * 6
    

    def left_robot_get_arm_status(self) -> dict:
        """Get left arm overall status."""
        if self.left_robot is None:
            return {"ctrl_mode": 0, "arm_status": 0, "motion_status": 0}
        result = self.left_robot.get_arm_status()
        if result is None:
            return {"ctrl_mode": 0, "arm_status": 0, "motion_status": 0}
        return {
            "ctrl_mode": result.msg.ctrl_mode,
            "arm_status": result.msg.arm_status,
            "motion_status": result.msg.motion_status,
            "trajectory_num": result.msg.trajectory_num
        }
    
    # ==================== Right Arm State Query ====================

    def right_robot_get_joint_positions(self) -> list:
        """Get right arm joint positions (radians)."""
        if self.right_robot is None:
            return [0.0] * 7
        result = self.right_robot.get_joint_angles()
        return result.msg if result is not None else [0.0] * 7
    
    def right_robot_get_joint_velocities(self) -> list:
        """Get right arm joint velocities (rad/s)."""
        if self.right_robot is None:
            return [0.0] * 7
        velocities = []
        for i in range(1, 8):
            result = self.right_robot.get_motor_states(i)
            if result is not None:
                velocities.append(result.msg.velocity)
            else:
                velocities.append(0.0)
        return velocities
    
    def right_robot_get_ee_pose(self) -> list:
        """Get right arm end-effector pose [x, y, z, roll, pitch, yaw] (m, rad)."""
        if self.right_robot is None:
            return [0.0] * 6
        result = self.right_robot.get_tcp_pose()
        return result.msg if result is not None else [0.0] * 6
    
    def right_robot_get_arm_status(self) -> dict:
        """Get right arm overall status."""
        if self.right_robot is None:
            return {"ctrl_mode": 0, "arm_status": 0, "motion_status": 0}
        result = self.right_robot.get_arm_status()
        if result is None:
            return {"ctrl_mode": 0, "arm_status": 0, "motion_status": 0}
        return {
            "ctrl_mode": result.msg.ctrl_mode,
            "arm_status": result.msg.arm_status,
            "motion_status": result.msg.motion_status,
            "trajectory_num": result.msg.trajectory_num
        }
    
    # ==================== Left Arm Motion ====================
    
    def left_robot_move_to_joint_positions(self, positions: list, delta: bool = False):
        """Move left arm to joint positions (radians)."""
        if self.left_robot is None:
            return
        
        # @Key-Zzs: fix TypeError
        positions = np.asarray(positions, dtype=float)
        if positions.shape[0] != 7:
            raise ValueError(f"Expected 7 joints, got {positions.shape[0]}")
        
        if delta:
            current = np.asarray(self.left_robot_get_joint_positions(), dtype=float)
            target = current + positions
        else:
            target = positions

        target_list = target.tolist()
        log.info("[DEBUG] move_j target: %s", target_list)

        self.left_robot.move_j(target_list)
        # time.sleep(3.0)
        log.info("move to joint positions completed")
        
    def left_robot_move_to_ee_pose(self, pose: list, delta: bool = False):
        """Move left arm to end-effector pose [x, y, z, roll, pitch, yaw] (m, rad)."""
        """use move_p for direct pose control, but it may cause discontinuity and vibration."""
        if self.left_robot is None:
            return
        
        pose = np.asarray(pose, dtype=float)
        if pose.shape[0] != 6:
            raise ValueError(f"Expected 6 joints, got {pose.shape[0]}")

        if delta:
            current = np.asarray(self.left_robot_get_ee_pose(), dtype=float)
            target = current + pose
        else:
            target = pose
        
        target_list = target.tolist()
        log.info("[DEBUG] move_p target: %s", target_list)
    
        self.left_robot.set_speed_percent(30)

        self.left_robot.move_p(target_list)
        # time.sleep(3.0)
        log.info("move to end-effector pose completed")

    # ==================== Right Arm Motion ====================
    
    # same as left_robot_move_to_joint_positions() and left_robot_move_to_ee_pose() 
    def right_robot_move_to_joint_positions(self, positions: list, delta: bool = False):
        """Move right arm to joint positions (radians)."""
        if self.right_robot is None:
            return
        
        positions = np.asarray(positions, dtype=float)
        if positions.shape[0] != 7:
            raise ValueError(f"Expected 7 joints, got {positions.shape[0]}")
        
        if delta:
            current = np.asarray(self.right_robot_get_joint_positions(), dtype=float)
            target = current + positions
        else:
            target = positions

        target_list = target.tolist()
        log.info("[DEBUG] move_j target: %s", target_list)

        self.right_robot.move_j(target_list)
        # time.sleep(3.0)
        log.info("move to joint positions completed")
    
    def right_robot_move_to_ee_pose(self, pose: list, delta: bool = False):
        """Move right arm to end-effector pose [x, y, z, roll, pitch, yaw] (m, rad)."""
        if self.right_robot is None:
            return
        
        if delta:
            current = self.right_robot_get_ee_pose()
            target = np.array(current) + np.array(pose)
        else:
            target = pose

        target_list = target.tolist()
        log.info("[DEBUG] move_p target: %s", target_list)
    
        self.right_robot.move_p(target_list)
        # time.sleep(3.0)
        log.info("move to end-effector pose completed")

    def dual_robot_move_to_ee_pose(self, left_pose: list, right_pose: list, delta: bool = False):
        if self.left_robot is None or self.right_robot is None:
            return
        self.left_robot_move_to_ee_pose(left_pose, delta=delta, wait=True)
        self.right_robot_move_to_ee_pose(right_pose, delta=delta, wait=True)

    def _go_home(self, robot_arm: str, callback=None):
        """
        机械臂回零，先重置状态机清除急停锁死，再切换回正常模式，最后使能上电并运动到初始位置
        
        Args:
            robot_arm: "left_robot" or "right_robot"
            callback: 完成回调函数，参数为 (success: bool, message: str)
        """
        success = False
        message = ""
        
        try:
            if robot_arm == "left_robot":
                robot = self.left_robot
                ik_solver = self.left_ik_solver
                home = [0.0, -0.13, 0.0, 1.87, 0.0, 0.0, -0.17]
                target_pose_attr = "target_pose_left"
            elif robot_arm == "right_robot":
                robot = self.right_robot
                ik_solver = self.right_ik_solver
                home = [0.0, -0.13, 0.0, 1.87, 0.0, 0.0, -0.17]
                target_pose_attr = "target_pose_right"
            else:
                message = f"Invalid robot_arm: {robot_arm}"
                log.error(f"[{robot_arm}_go_home] {message}")
                if callback:
                    callback(False, message)
                return
            
            if robot is None:
                message = "Robot not initialized"
                log.error(f"[{robot_arm}_go_home] {message}")
                if callback:
                    callback(False, message)
                return

            log.info(f"[{robot_arm}_go_home] Starting home sequence...")

            # Step 1: 重置状态机
            log.info(f"[{robot_arm}_go_home] Resetting state machine...")
            try:
                robot.reset()
                time.sleep(1.0)
            except Exception as e:
                message = f"Failed to reset robot: {e}"
                log.error(f"[{robot_arm}_go_home] {message}")
                if callback:
                    callback(False, message)
                return

            # Step 2: 切换到正常模式
            log.info(f"[{robot_arm}_go_home] Switching to normal mode...")
            try:
                robot.set_normal_mode()
                time.sleep(0.5)
            except Exception as e:
                message = f"Failed to set normal mode: {e}"
                log.error(f"[{robot_arm}_go_home] {message}")
                if callback:
                    callback(False, message)
                return

            # Step 3: 使能机械臂
            log.info(f"[{robot_arm}_go_home] Enabling robot...")
            start_t = time.monotonic()
            is_enabled = False
            while time.monotonic() - start_t < 5.0:
                try:
                    if robot.enable():
                        is_enabled = True
                        break
                except Exception as e:
                    log.warning(f"[{robot_arm}_go_home] Enable attempt failed: {e}")
                time.sleep(0.5)

            if not is_enabled:
                message = "Failed to enable robot after 5 seconds"
                log.error(f"[{robot_arm}_go_home] {message}")
                if callback:
                    callback(False, message)
                return

            log.info(f"[{robot_arm}_go_home] Robot enabled successfully")

            # Step 4: 设置速度并运动到home位置
            try:
                robot.set_speed_percent(30)
                log.info(f"[{robot_arm}_go_home] Moving to home position: {home}")
                robot.move_j(home)
            except Exception as e:
                message = f"Failed to move to home: {e}"
                log.error(f"[{robot_arm}_go_home] {message}")
                if callback:
                    callback(False, message)
                return

            # Step 5: 等待运动完成
            log.info(f"[{robot_arm}_go_home] Waiting for motion to complete...")
            time.sleep(3.0)

            # Step 6: 更新目标位姿
            if robot is not None and ik_solver is not None:
                from pyAgxArm.utiles.tf import rot_to_rpy
                try:
                    current_joints = None
                    timeout = 2.0
                    start_t = time.monotonic()
                    while current_joints is None:
                        ja = robot.get_joint_angles()
                        if ja is not None:
                            current_joints = ja.msg
                            break
                        if time.monotonic() - start_t > timeout:
                            log.warning(f"[{robot_arm}_go_home] get_joint_angles timeout")
                            break
                        time.sleep(0.01)
                    
                    if current_joints is not None:
                        q_current = np.array(current_joints, dtype=float)
                        T_fk = fk(q_current, ik_solver.nero_params)
                        fk_xyz = np.asarray(T_fk[:3, 3], dtype=float)
                        fk_rpy = np.asarray(rot_to_rpy(T_fk[:3, :3].tolist()), dtype=float)
                        target_pose = np.concatenate([fk_xyz, fk_rpy])
                        setattr(self, target_pose_attr, target_pose)
                        log.info(f"[{robot_arm}_go_home] Updated {target_pose_attr}: {target_pose}")
                    else:
                        log.warning(f"[{robot_arm}_go_home] Failed to get current joints, pose not updated")
                except Exception as e:
                    message = f"Failed to update pose: {e}"
                    log.error(f"[{robot_arm}_go_home] {message}")
                    if callback:
                        callback(False, message)
                    return

            success = True
            message = "Home sequence completed successfully"
            log.info(f"[{robot_arm}_go_home] {message}")

        except Exception as e:
            message = f"Unexpected error during home sequence: {e}"
            log.error(f"[{robot_arm}_go_home] {message}", exc_info=True)
            success = False

        finally:
            if callback:
                callback(success, message)

    def left_robot_go_home(self):
        """
        非阻塞触发左臂回零，由control_loop执行
        """
        self.left_go_home_flag = True
        return True
    
    def right_robot_go_home(self):
        """
        非阻塞触发右臂回零，由control_loop执行
        """
        self.right_go_home_flag = True
        return True

    def robot_go_home(self):
        """
        非阻塞触发左右臂回零，由control_loop执行
        """
        self.go_home_flag = True
        return True

    # ==================== ServoJ Control (Joint Servo) ====================

    
    def servo_j(self, robot_arm: str, joints: list, delta: bool) -> bool:
        """
        直接输入某个机械臂名称与目标关节角度（度），控制机械臂运动。

        Args:
            robot_arm: "left_robot" or "right_robot"
            joints: 7维绝对关节角度（度）
            delta: False=绝对控制, True=增量控制

        Returns:
            bool: 成功返回 True，失败返回 False
        """
        try:
            joints = np.asarray(joints, dtype=float)
            if joints.shape[0] != 7:
                raise ValueError(f"Expected 7 joints, got {joints.shape[0]}")
            
            # 根据 robot_arm 选择对应的机械臂和 get_joint_positions 方法
            if robot_arm == "left_robot":
                robot = self.left_robot
                get_joint_positions = self.left_robot_get_joint_positions()
            elif robot_arm == "right_robot":
                robot = self.right_robot
                get_joint_positions = self.right_robot_get_joint_positions()
            else:
                raise ValueError("robot_arm must be 'left_robot' or 'right_robot'")
            
            if robot is None:
                log.error(f"[ERROR] {robot_arm} not initialized")
                return False
            
            # 计算目标关节角度
            if delta:
                current = np.asarray(get_joint_positions, dtype=float)
                for i in range(7):
                    current[i] = np.rad2deg(current[i])
                target = current + joints
            else:
                target = joints

            log.info(f"[DEBUG] servo_j target (degree): {target}")
            
            # 转换为弧度
            for i in range(7):
                target[i] = np.deg2rad(target[i])
            
            target = target.tolist()

            # 下发关节控制
            robot.move_js(target)

            return True

        except Exception as e:
            log.error(f"[ERROR] servo_j failed: {e}")
            return False
        
    
    # ==================== ServoP Control (Pose Servo) ====================

    def _get_current_pose(self, robot, ik_solver):
        """
        获取当前机械臂末端执行器的实际位姿（通过FK正运动学计算）。

        该函数通常用于：
        - 初始化 target_pose（首次接收控制指令时）
        - 将机械臂当前状态作为控制起点

        Args:
            robot: 机械臂控制对象（left_robot 或 right_robot）
            ik_solver: 对应的逆运动学求解器（用于获取DH参数等）

        Returns:
            np.ndarray: 6维位姿 [x, y, z, rx, ry, rz]
                        若获取失败返回 None
        """
        from nero.kinematics.nero_kinematics.nero_ik.ik_solver import fk
        from pyAgxArm.utiles.tf import rot_to_rpy

        ja = robot.get_joint_angles()
        if ja is None:
            return None

        q = np.array(ja.msg, dtype=float)
        T_fk = fk(q, ik_solver.nero_params)

        xyz = np.asarray(T_fk[:3, 3], dtype=float)
        rpy = np.asarray(rot_to_rpy(T_fk[:3, :3].tolist()), dtype=float)

        return np.concatenate([xyz, rpy])
    
    def _apply_delta(self, current_pose, delta):
        """
        将增量位姿（delta）作用到当前位姿上，计算新的目标位姿。

        处理逻辑：
        - 平移部分：直接相加
        - 旋转部分：通过四元数相乘进行组合，避免欧拉角奇异性问题

        该函数用于将“遥操作输入（增量）”转换为“控制目标（绝对位姿）”，
        是实现 delta 控制模式的关键步骤。

        Args:
            current_pose: 当前目标位姿（6维 [x, y, z, rx, ry, rz]）
            delta: 增量位姿（6维 [dx, dy, dz, drx, dry, drz]）

        Returns:
            np.ndarray: 更新后的目标位姿（6维）
        """
        from pyAgxArm.utiles.tf import euler_convert_quat, quat_convert_euler

        cur_xyz = current_pose[:3]
        cur_rpy = current_pose[3:]

        # 位置
        target_xyz = cur_xyz + delta[:3]

        # 姿态
        current_quat = euler_convert_quat(*cur_rpy)
        delta_quat = euler_convert_quat(*delta[3:])
        target_quat = _quat_multiply(current_quat, delta_quat)

        target_quat = target_quat / np.linalg.norm(target_quat)
        target_rpy = quat_convert_euler(*target_quat)

        return np.concatenate([target_xyz, target_rpy])

    def servo_p_OL(self, robot_arm: str, pose: list, delta: bool) -> bool:
        """
        接收来自客户端的末端位姿控制指令（支持增量/绝对），并更新服务器内部的目标位姿状态（target_pose）。

        该函数不直接控制机械臂运动，仅用于更新控制目标，由后台控制线程以固定频率执行实际控制。

        Args:
            robot_arm: "left_robot" 或 "right_robot"
            pose: 6维位姿（[x, y, z, rx, ry, rz]）
                当 delta=True 时表示位姿增量
                当 delta=False 时表示绝对目标位姿
            delta: True=增量控制（推荐用于遥操作），False=绝对位姿控制

        Returns:
            bool: 成功返回 True，失败返回 False
        """
        try:
            pose = np.asarray(pose, dtype=float)

            # TODO:限幅
            # max_trans = 0.01   # 1cm
            # max_rot = 0.05     # rad
            # pose[:3] = np.clip(pose[:3], -max_trans, max_trans)
            # pose[3:] = np.clip(pose[3:], -max_rot, max_rot)

            if robot_arm == "left_robot":
                # 初始化目标位姿（仅在第一次调用时）
                if self.target_pose_left is None:
                    
                    self.target_pose_left = self._get_current_pose(
                        self.left_robot, self.left_ik_solver
                    )

                if delta:
                    self.target_pose_left = self._apply_delta(
                        self.target_pose_left, pose
                    )
                else:
                    self.target_pose_left = pose

            elif robot_arm == "right_robot":
                # 初始化目标位姿（仅在第一次调用时）
                if self.target_pose_right is None:
                    self.target_pose_right = self._get_current_pose(
                        self.right_robot, self.right_ik_solver
                    )

                if delta:
                    self.target_pose_right = self._apply_delta(
                        self.target_pose_right, pose
                    )
                else:
                    self.target_pose_right = pose

            return True

        except Exception as e:
            log.error(f"[ERROR] servo_p_OL failed: {e}")
            return False  
    
    # ==================== Gripper (Placeholder) ====================

    def _gripper_goto(self, gripper, width: float, force: float = 1.0, callback=None):
        if not self.gripper_enabled or gripper is None:
            log.warning("[SERVER] Gripper not available")
            if callback:
                callback(False)
            return False

        width = float(max(0.0, min(width, 0.1)))

        log.info(f"[SERVER] Gripper goto: width={width:.3f}, force={force}")

        try:
            gripper.move_gripper(width=width, force=force)
            if callback:
                callback(True)
            return True
        except Exception as e:
            log.error(f"[SERVER] Gripper goto failed: {e}")
            if callback:
                callback(False)
            return False

    def left_gripper_goto(self, width, force):
        """
        非阻塞设置左夹爪目标开度，由control_loop执行
        """
        self.left_gripper_target = (width, force)
        return True
        
    def left_gripper_get_state(self):
        if not self.gripper_enabled or self.left_gripper is None:
            return {"is_moving": False, "is_grasped": False}

        try:
            status = self.left_gripper.get_gripper_status()
            if status is None:
                return {"is_moving": False, "is_grasped": False}

            width = status.msg.width
            force = status.msg.force

            is_moving = abs(force) > 0.1
            is_grasped = (width > 0.005) and (force > 0.5)

            return {
                "width": width,
                "force": force,
                "is_moving": is_moving,
                "is_grasped": is_grasped
            }

        except Exception as e:
            log.error(f"[SERVER] Left gripper state failed: {e}")
            return {"is_moving": False, "is_grasped": False}

    def right_gripper_goto(self, width, force):
        """
        非阻塞设置右夹爪目标开度，由control_loop执行
        """
        self.right_gripper_target = (width, force)
        return True

    def right_gripper_get_state(self):
        if not self.gripper_enabled or self.right_gripper is None:
            return {"is_moving": False, "is_grasped": False}

        try:
            status = self.right_gripper.get_gripper_status()
            if status is None:
                return {"is_moving": False, "is_grasped": False}

            width = status.msg.width
            force = status.msg.force

            is_moving = abs(force) > 0.1
            is_grasped = (width > 0.005) and (force > 0.5)

            return {
                "width": width,
                "force": force,
                "is_moving": is_moving,
                "is_grasped": is_grasped
            }

        except Exception as e:
            log.error(f"[SERVER] Right gripper state failed: {e}")
            return {"is_moving": False, "is_grasped": False}
    
    # ==================== Utility ====================
    
    def stop(self, robot_arm: str):
        """
        Stops the specified robot arm by triggering an emergency stop.

        Args:
            robot_arm (str): The name of the robot arm to stop, either "left_robot" or "right_robot".

        Returns:
            bool: True if the stop command was executed successfully, False otherwise.
        """
        try:
            if robot_arm == "left_robot":
                if self.left_robot is not None:
                    self.left_robot.electronic_emergency_stop()
                    log.info("[SERVER] Left robot emergency stopped")
                else:
                    log.warning("[SERVER] Left robot is not initialized")
            elif robot_arm == "right_robot":
                if self.right_robot is not None:
                    self.right_robot.electronic_emergency_stop()
                    log.info("[SERVER] Right robot emergency stopped")
                else:
                    log.warning("[SERVER] Right robot is not initialized")
            else:
                raise ValueError("robot_arm must be 'left_robot' or 'right_robot'")

            return True

        except Exception as e:
            log.error(f"[ERROR] servo_j failed: {e}")
            return False
    
    # ==================== Control Loop ====================
    def control_loop(self):
        """
        控制主循环线程，以固定频率（100Hz）持续执行机械臂控制。

        该函数负责：
        1. 读取当前目标位姿（target_pose）
        2. 同步IK求解器状态（降低频率）
        3. 调用逆运动学（IK）求解目标关节角
        4. 通过 move_js 接口下发关节命令，实现连续控制
        5. 处理gripper和home任务（异步）

        注意：
        - 该循环为“时钟驱动”，不依赖客户端调用频率
        - 即使没有新的输入指令，也会持续跟踪当前目标位姿
        - 是整个系统实现“平滑、连续控制”的核心

        无参数，无返回值
        """
        sync_counter = 0
        sync_interval = 10  # 每10个控制周期同步一次IK状态（100ms）
        
        while self.control_running:
            start = time.perf_counter()
            sync_counter += 1

            try:
                # ===== IK状态同步（降低频率） =====
                if sync_counter % sync_interval == 0:
                    try:
                        if self.left_robot is not None and self.left_ik_solver is not None:
                            ja = self.left_robot.get_joint_angles()
                            if ja is not None:
                                self.left_ik_solver.init_state(ja.msg)
                        
                        if self.right_robot is not None and self.right_ik_solver is not None:
                            ja = self.right_robot.get_joint_angles()
                            if ja is not None:
                                self.right_ik_solver.init_state(ja.msg)
                    except Exception as e:
                        log.warning(f"[CONTROL] IK sync failed: {e}")

                # ===== LEFT POSE控制 =====
                if self.target_pose_left is not None:
                    try:
                        q_cmd = self.left_ik_solver.solve(self.target_pose_left)
                        if q_cmd is not None:
                            if isinstance(q_cmd, np.ndarray):
                                q_cmd = q_cmd.tolist()
                            self.left_robot.move_js(q_cmd)
                    except Exception as e:
                        log.error(f"[CONTROL] Left arm IK solve failed: {e}")

                # ===== RIGHT POSE控制 =====
                if self.target_pose_right is not None:
                    try:
                        q_cmd = self.right_ik_solver.solve(self.target_pose_right)
                        if q_cmd is not None:
                            if isinstance(q_cmd, np.ndarray):
                                q_cmd = q_cmd.tolist()
                            self.right_robot.move_js(q_cmd)
                    except Exception as e:
                        log.error(f"[CONTROL] Right arm IK solve failed: {e}")

                # ===== LEFT GRIPPER任务 =====
                if self.left_gripper_target is not None and not self.left_gripper_busy:
                    width, force = self.left_gripper_target
                    self.left_gripper_busy = True
                    self.left_gripper_target = None  # 清除目标，避免重复触发

                    def on_left_gripper_done(success: bool):
                        self.left_gripper_busy = False
                        if not success:
                            log.warning(f"[CONTROL] Left gripper command failed, retrying...")
                            self.left_gripper_target = (width, force)  # 失败则重新设置目标

                    try:
                        threading.Thread(
                            target=self._gripper_goto,
                            args=(self.left_gripper, width, force, on_left_gripper_done),
                            daemon=True
                        ).start()
                    except Exception as e:
                        log.error(f"[CONTROL] Left gripper thread start failed: {e}")
                        self.left_gripper_busy = False
                        self.left_gripper_target = (width, force)  # 恢复目标

                # ===== RIGHT GRIPPER任务 =====
                if self.right_gripper_target is not None and not self.right_gripper_busy:
                    width, force = self.right_gripper_target
                    self.right_gripper_busy = True
                    self.right_gripper_target = None  # 清除目标，避免重复触发

                    def on_right_gripper_done(success: bool):
                        self.right_gripper_busy = False
                        if not success:
                            log.warning(f"[CONTROL] Right gripper command failed, retrying...")
                            self.right_gripper_target = (width, force)  # 失败则重新设置目标

                    try:
                        threading.Thread(
                            target=self._gripper_goto,
                            args=(self.right_gripper, width, force, on_right_gripper_done),
                            daemon=True
                        ).start()
                    except Exception as e:
                        log.error(f"[CONTROL] Right gripper thread start failed: {e}")
                        self.right_gripper_busy = False
                        self.right_gripper_target = (width, force)  # 恢复目标

                # ===== LEFT HOME任务 =====
                if self.left_go_home_flag and not self.left_home_busy:
                    self.left_home_busy = True
                    self.left_go_home_flag = False

                    def on_left_home_done(success: bool, message: str):
                        self.left_home_busy = False
                        if success:
                            log.info(f"[CONTROL] Left home completed: {message}")
                        else:
                            log.error(f"[CONTROL] Left home failed: {message}")

                    try:
                        threading.Thread(
                            target=self._go_home,
                            args=("left_robot", on_left_home_done),
                            daemon=True
                        ).start()
                    except Exception as e:
                        log.error(f"[CONTROL] Left home thread start failed: {e}")
                        self.left_home_busy = False

                # ===== RIGHT HOME任务 =====
                if self.right_go_home_flag and not self.right_home_busy:
                    self.right_home_busy = True
                    self.right_go_home_flag = False

                    def on_right_home_done(success: bool, message: str):
                        self.right_home_busy = False
                        if success:
                            log.info(f"[CONTROL] Right home completed: {message}")
                        else:
                            log.error(f"[CONTROL] Right home failed: {message}")

                    try:
                        threading.Thread(
                            target=self._go_home,
                            args=("right_robot", on_right_home_done),
                            daemon=True
                        ).start()
                    except Exception as e:
                        log.error(f"[CONTROL] Right home thread start failed: {e}")
                        self.right_home_busy = False

                # ===== DUAL HOME任务 =====
                if self.go_home_flag:
                    self.go_home_flag = False
                    if not self.left_home_busy:
                        self.left_go_home_flag = True
                    if not self.right_home_busy:
                        self.right_go_home_flag = True

            except KeyboardInterrupt:
                log.warning("[CONTROL] Control loop interrupted by user")
                self.control_running = False
                break
            except Exception as e:
                log.error(f"[CONTROL] Unexpected error in control loop: {e}", exc_info=True)

            # ===== 固定频率控制 =====
            dt = time.perf_counter() - start
            sleep_time = self.control_dt - dt
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                log.warning(f"[CONTROL] Control loop overrun by {-sleep_time:.3f}s")

def start_server(ip: str, port: int = 4242, gripper_enabled: bool = True):
    server = zerorpc.Server(NeroDualArmServer(gripper_enabled))
    server.bind(f"tcp://{ip}:{port}")
    log.info(f"[SERVER] Listening on tcp://{ip}:{port}")
    server.run()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', type=str, default='0.0.0.0')
    parser.add_argument('--port', type=int, default=4242)
    parser.add_argument('--no-gripper', action='store_true')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
    start_server(ip=args.ip, port=args.port, gripper_enabled=not args.no_gripper)