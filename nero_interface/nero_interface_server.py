'''
Nero dual-arm robot interface server.
Provides zerorpc interface for dual-arm control.
'''

import zerorpc
import numpy as np
import logging
import time
import math
from typing import Optional, List
import sys, os
import pdb

# ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "."))
# sys.path.insert(0, ROOT_DIR)

ROOT_DIR = os.path.abspath(os.path.dirname(__file__))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from analytic_IK_solver import Solver, build_joint_limits_from_cfg
from nero_ik.ik_solver import fk

log = logging.getLogger(__name__)

# 手动实现四元数乘法 (输入输出均为 [x, y, z, w] 格式)
def quat_multiply(q1, q2):
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
        self.left_cur_pose = None
        self.right_cur_pose = None

        try:
            self.left_ik_solver = self.setup_ik_solver(self.left_robot, self.left_cfg, "Left Arm")
            # self.right_ik_solver = self.setup_ik_solver(self.right_robot, self.right_cfg, "Right Arm")
        except Exception as e:
            log.error(f"[ERROR] IK solvers init failed: {e}")

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
        
        # @Key-Zzs: fix TypeError
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
    
    # @Key-Zzs: copy from left_robot_move_to_joint_positions() and left_robot_move_to_ee_pose() 
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

    # TODO
    # def dual_robot_move_to_ee_pose(self, left_pose: list, right_pose: list, delta: bool = False):
    #     if self._robot is None:
    #         return
    #     self._robot.dual_move_to_ee_pose(left_pose, right_pose, delta=delta, wait=True)
    
    # ==================== Go Home ====================

    # @Key-Zzs: [feat] left_robot_go_home() and right_robot_go_home()
    def left_robot_go_home(self):
        if self.left_robot is None:
            log.error("Left robot not initialized")
            return

        log.info("正在连接机械臂...")
        self.left_robot.connect()
        time.sleep(0.5)

        log.info("\n--- 开始重置 ---")
        log.info("正在清除急停锁死标志...")
        self.left_robot.reset() 
        time.sleep(1.0)  # 给主控足够的时间重启状态机
        
        log.info("正在切换回正常控制模式...")
        self.left_robot.set_normal_mode()
        time.sleep(0.5)
        log.info("--- 状态机重置完毕 ---\n")

        log.info("正在使能机械臂...")
        start_t = time.monotonic()
        is_enabled = False
        while time.monotonic() - start_t < 5.0:
            if self.left_robot.enable():
                is_enabled = True
                break
            time.sleep(0.5)

        if not is_enabled:
            log.error("failed to enable left robot")
            return

        log.info("Left robot enabled!")

        self.left_robot.set_speed_percent(30)

        home = [0.0, -0.13, 0.0, 1.87, 0.0, 0.0, -0.17]

        log.info("[DEBUG] Moving to home: %s", home)
        self.left_robot.move_j(home)

        result = self.left_robot.get_joint_angles()
        log.info("当前关节角度:", result.msg)

        time.sleep(3.0)  # 等待运动完成
        log.info("已回到初始位置")
    
    def right_robot_go_home(self):
        if self.right_robot is None:
            log.error("Right robot not initialized")
            return

        log.info("正在连接机械臂...")
        self.right_robot.connect()
        time.sleep(0.5)

        log.info("\n--- 开始重置 ---")
        log.info("正在清除急停锁死标志...")
        self.right_robot.reset() 
        time.sleep(1.0)  # 给主控足够的时间重启状态机
        
        log.info("正在切换回正常控制模式...")
        self.right_robot.set_normal_mode()
        time.sleep(0.5)
        log.info("--- 状态机重置完毕 ---\n")

        log.info("正在使能机械臂...")
        start_t = time.monotonic()
        is_enabled = False
        while time.monotonic() - start_t < 5.0:
            if self.right_robot.enable():
                is_enabled = True
                break
            time.sleep(0.5)

        if not is_enabled:
            log.info("使能失败！")
            return
            
        log.info("机械臂已使能上电！")

        self.right_robot.set_speed_percent(30)

        #TODO：not test yet, use 'left_robot_move_to_joint_positions(left_joints_target, delta=False)' to determine
        home = [0.0, -0.13, 0.0, 1.87, 0.0, 0.0, -0.17]

        log.info("[DEBUG] Moving to home:", home)
        self.left_robot.move_j(home)
        
        result = self.right_robot.get_joint_angles()
        log.info("当前关节角度:", result.msg)

        time.sleep(3.0)  # 等待运动完成
        log.info("已回到初始位置")

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

    def servo_p_OL(self, robot_arm: str, pose: list, delta: bool) -> bool:
        """
        Send ServoP open loop with target pose [x, y, z, rx, ry, rz] (m, radians).
        Args:
            robot_arm: "left_robot" or "right_robot"
            pose: 末端位置(m, radians)
            delta: 绝对控制(False)，增量控制(True)
        Returns:
            bool: 成功返回 True，失败返回 False
        """
        try:
            from pyAgxArm.utiles.tf import rot_to_rpy, euler_convert_quat, quat_convert_euler
            # 1. 选择 robot & IK
            if robot_arm == "left_robot":
                robot = self.left_robot
                ik_solver = self.left_ik_solver
                cur_pose_attr = "left_cur_pose"
            elif robot_arm == "right_robot":
                robot = self.right_robot
                ik_solver = self.right_ik_solver
                cur_pose_attr = "right_cur_pose"
            else:
                log.error(f"[ERROR] invalid robot_arm: {robot_arm}")
                return False

            if robot is None or ik_solver is None:
                log.error("[ERROR] robot or IK solver not ready")
                return False
            
            # 首次调用时，FK 获取当前位姿作为 servo_p 增量控制的基准
            if getattr(self, cur_pose_attr) is None:
                # 获取当前真实关节角（带超时保护）
                current_joints = None
                timeout = 2.0  # 2秒超时
                start_t = time.monotonic()
                while current_joints is None:
                    ja = robot.get_joint_angles()

                    if ja is not None:
                        current_joints = ja.msg
                        break
                    if time.monotonic() - start_t > timeout:
                        log.error("[ERROR] get_joint_angles timeout")
                        return False
                    time.sleep(0.01)

                q_current = np.array(current_joints, dtype=float)
                # FK 计算当前位姿
                T_fk = fk(q_current, ik_solver.nero_params)
                fk_xyz = np.asarray(T_fk[:3, 3], dtype=float)
                fk_rpy = np.asarray(rot_to_rpy(T_fk[:3, :3].tolist()), dtype=float)
                fk_pose = np.concatenate([fk_xyz, fk_rpy])
                if fk_pose is not None:
                    setattr(self, cur_pose_attr, fk_pose)
                else:
                    log.error("[ERROR] cur_pose is None")
                    return False
            
            # TODO: wait for testing
            cur_pose = getattr(self, cur_pose_attr)
            pose = np.asarray(pose, dtype=float).reshape(-1)
            if pose.size != 6:
                raise ValueError(f"Expected 6 pose values, got {pose.size}")

            # 2. 计算 target_pose
            if delta:
                cur_xyz = np.asarray(cur_pose[:3], dtype=float)
                cur_rpy = np.asarray(cur_pose[3:], dtype=float)
                current_pose = np.concatenate([cur_xyz, cur_rpy])
                
                print(f"当前位姿: {current_pose}")

                # --- 计算目标位姿 ---
                # 1. 位置直接相加
                target_xyz = cur_xyz + np.array(pose[:3], dtype=float)

                # 2. rpy相加转为四元数相乘
                ## 当前姿态 RPY → 四元数
                current_quat = euler_convert_quat(cur_rpy[0], cur_rpy[1], cur_rpy[2])
                ## 增量 RPY → 四元数
                delta_quat = euler_convert_quat(pose[3], pose[4], pose[5])

                ## 末端姿态增量四元数相乘得到目标姿态四元数
                target_quat = quat_multiply(current_quat, delta_quat)
                ## 目标姿态四元数归一化
                # target_quat = quat_normalize(target_quat) # wait for function
                q_norm = np.sqrt(target_quat[0]**2 + target_quat[1]**2 + target_quat[2]**2 + target_quat[3]**2)
                target_quat = tuple(v / q_norm for v in target_quat)
                ## 目标姿态四元数 → RPY
                target_rpy = quat_convert_euler(*target_quat)
                print(f"目标姿态 XYZ: {target_xyz}")
                print(f"目标姿态 RPY: {target_rpy}")

                # 3. 合并位置和姿态
                target_pose = np.concatenate([target_xyz, target_rpy])
            else:
                # target_pose = np.array(pose, dtype=float)
                target_pose = pose

            # 3. IK 求解
            q_cmd = ik_solver.solve(target_pose)
            print(f"计算出的关节角度: {q_cmd}")
            print("-------------------------------")

            # 增加对求解失败的安全校验
            if q_cmd is None or len(q_cmd) == 0:
                log.error("[ERROR] IK solve failed: returned None/Empty")
                return False

            if isinstance(q_cmd, np.ndarray):
                q_cmd = q_cmd.tolist()

            # 4. 下发关节控制
            robot.move_js(q_cmd)

            # 5. 更新当前位姿
            setattr(self, cur_pose_attr, target_pose)

            return True

        except Exception as e:
            log.error(f"[ERROR] servo_p_OL failed", e)
            return False

    def servo_p(self, robot_arm: str, pose: list, delta: bool) -> bool:
        """
        Send ServoP with target pose [x, y, z, rx, ry, rz] (m, radians).
        Args:
            robot_arm: "left_robot" or "right_robot"
            pose: 末端位置(m, radians)
            delta: 绝对控制(False)，增量控制(True)
        Returns:
            bool: 成功返回 True，失败返回 False
        """
        try:
            from pyAgxArm.utiles.tf import rot_to_rpy, euler_convert_quat, quat_convert_euler
            # 1. 选择 robot & IK
            if robot_arm == "left_robot":
                robot = self.left_robot
                ik_solver = self.left_ik_solver
            elif robot_arm == "right_robot":
                robot = self.right_robot
                ik_solver = self.right_ik_solver
            else:
                log.error(f"[ERROR] invalid robot_arm: {robot_arm}")
                return False

            if robot is None or ik_solver is None:
                log.error("[ERROR] robot or IK solver not ready")
                return False
            
            # TODO: wait for testing
            pose = np.asarray(pose, dtype=float).reshape(-1)
            if pose.size != 6:
                raise ValueError(f"Expected 6 pose values, got {pose.size}")

            # 2. 计算 target_pose
            if delta:
                # 获取当前真实关节角（带超时保护）
                current_joints = None
                timeout = 2.0  # 2秒超时
                start_t = time.monotonic()
                while current_joints is None:
                    ja = robot.get_joint_angles()

                    if ja is not None:
                        current_joints = ja.msg
                        break
                    if time.monotonic() - start_t > timeout:
                        log.error("[ERROR] get_joint_angles timeout")
                        return False
                    time.sleep(0.01)

                q_current = np.array(current_joints, dtype=float)
                
                # # 关键：同步 IK 求解器状态（仅在差异较大时）
                # # if not hasattr(ik_solver, 'state') or ik_solver.state is None:
                # if ik_solver.state is None:
                #     ik_solver.init_state(q_current)
                # else:
                #     q_prev = np.asarray(ik_solver.state.q_prev, dtype=float).flatten()
                #     if q_prev.shape[0] != 7 or np.linalg.norm(q_current - q_prev) > 0.1:
                #         # 差异超过 0.1 rad 才重新同步
                #         ik_solver.init_state(q_current)
                
                # 用当前关节角做 FK，得到当前末端位姿
                T_fk = fk(q_current, ik_solver.nero_params)
                fk_xyz = np.asarray(T_fk[:3, 3], dtype=float)
                fk_rpy = np.asarray(rot_to_rpy(T_fk[:3, :3].tolist()), dtype=float)
                current_pose = np.concatenate([fk_xyz, fk_rpy])

                # test2: get_tcp_pose
                # robot.set_tcp_offset([0.0, 0.0, -0.0235, 0.0, 0.0, 0.0])
                tcp_pose = robot.get_tcp_pose()
                if tcp_pose is None:
                    raise RuntimeError("get_tcp_pose() 返回 None")
                flange_pose = robot.get_flange_pose()
                if flange_pose is None:
                    raise RuntimeError("get_flange_pose() 返回 None")
                # T_fk = np.array(tcp_pose.msg, dtype=float)
                # fk_xyz = T_fk[:3]
                # fk_rpy = T_fk[3:]
                # current_pose = T_fk

                print("-------------------------------")
                print(f"FK当前位姿: {current_pose}")
                print(f"TCP当前位姿: {tcp_pose.msg}")
                print(f"Flange当前位姿: {flange_pose.msg}")

                # --- 计算目标位姿 ---
                # 1. 位置直接相加
                target_fk_xyz = fk_xyz + np.array(pose[:3], dtype=float)

                # # 2. 姿态用小增量近似相加后归一化到 [-pi, pi]
                # target_fk_rpy = (fk_rpy + np.pi) % (2.0 * np.pi) - np.pi

                # 2. rpy相加转为四元数相乘
                ## 当前姿态 RPY → 四元数
                current_quat = euler_convert_quat(fk_rpy[0], fk_rpy[1], fk_rpy[2])
                ## 增量 RPY → 四元数
                delta_quat = euler_convert_quat(pose[3], pose[4], pose[5])

                ## 末端姿态增量四元数相乘得到目标姿态四元数
                target_quat = quat_multiply(current_quat, delta_quat)
                ## 目标姿态四元数归一化
                # target_quat = quat_normalize(target_quat) # wait for function
                q_norm = np.sqrt(target_quat[0]**2 + target_quat[1]**2 + target_quat[2]**2 + target_quat[3]**2)
                target_quat = tuple(v / q_norm for v in target_quat)
                ## 目标姿态四元数 → RPY
                target_fk_rpy = quat_convert_euler(*target_quat)
                print(f"目标姿态 XYZ: {target_fk_xyz}")
                print(f"目标姿态 RPY: {target_fk_rpy}")

                # 3. 合并位置和姿态
                target_pose = np.concatenate([target_fk_xyz, target_fk_rpy])
            else:
                # target_pose = np.array(pose, dtype=float)
                target_pose = pose

            # 3. IK 求解
            q_cmd = ik_solver.solve(target_pose)
            print(f"计算出的关节角度: {q_cmd}")
            print("-------------------------------")

            # 增加对求解失败的安全校验 (判断是否返回了 None 或者空数组)
            if q_cmd is None or len(q_cmd) == 0:
                log.error("[ERROR] IK solve failed: returned None/Empty")
                return False

            if isinstance(q_cmd, np.ndarray):
                q_cmd = q_cmd.tolist()

            # 4. 下发关节控制
            robot.move_js(q_cmd)

            return True

        except Exception as e:
            log.error(f"[ERROR] servo_p failed: %s", e)
            return False
    
    # ==================== Inverse Kinematics ====================

    def setup_ik_solver(self, robot, cfg, name: str):
        """辅助方法：获取初始关节角，提取限位，并初始化 IK Solver"""
        print(f"[{name}] 正在获取当前关节角作为 IK 初始基准...")
        current_pose = None
        current_joints = None
        while current_pose is None or current_joints is None:
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
        print(f"[{name}] IK Solver 初始化完成！初始关节角: {np.array(current_joints).round(3)}")
        
        return ik_solver
    
    # ==================== Gripper (Placeholder) ====================
    
    # @Key-Zzs: [feat] gripper_goto and gripper_get_state implementation
    # TODO: 重构为非阻塞控制
    ## def task():
    ## threading.Thread(target=task, daemon=True).start()
    def left_gripper_goto(self, width: float, force: float = 1.0, wait: bool = True):
        if not self.gripper_enabled or self.left_gripper is None:
            log.warning("[SERVER] Left gripper not available")
            return False

        width = float(max(0.0, min(width, 0.1)))

        log.info(f"[SERVER] Left gripper goto: width={width:.3f}, force={force}")

        try:
            self.left_gripper.move_gripper(width=width, force=force)
            if wait:
                time.sleep(1.0)
            return True
        except Exception as e:
            log.error(f"[SERVER] Left gripper goto failed: {e}")
            return False
     
    # TODO: 实现逻辑未确定
    def left_gripper_grasp(self, force: float = 1.0, width: float = 0.05):
        if not self.gripper_enabled or self.left_gripper is None:
            log.warning("[SERVER] Left gripper not available")
            return False

        width = float(max(0.0, min(width, 0.1)))

        log.info(f"[SERVER] Left gripper grasp: width={width}, force={force}")

        try:
            self.left_gripper.move_gripper(width=width, force=force)
            time.sleep(1.5)

            status = self.left_gripper.get_gripper_status()
            if status is None:
                return False

            current_width = status.msg.width

            # 抓取判断（核心 heuristic）
            is_grasped = (width < 0.01) and (current_width > 0.005)

            log.info(f"[SERVER] Left grasp result: width={current_width:.4f}, grasped={is_grasped}")

            return is_grasped

        except Exception as e:
            log.error(f"[SERVER] Left gripper grasp failed: {e}")
            return False
        
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

    def right_gripper_goto(self, width: float, force: float = 1.0, wait: bool = True):
        if not self.gripper_enabled or self.right_gripper is None:
            log.warning("[SERVER] Right gripper not available")
            return False

        width = float(max(0.0, min(width, 0.1)))

        log.info(f"[SERVER] Right gripper goto: width={width:.3f}, force={force}")

        try:
            self.right_gripper.move_gripper(width=width, force=force)
            if wait:
                time.sleep(1.0)
            return True
        except Exception as e:
            log.error(f"[SERVER] Right gripper goto failed: {e}")
            return False
     
    # TODO: 实现逻辑未确定
    def right_gripper_grasp(self, force: float = 1.0, width: float = 0.05): pass

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
    
    # TODO: wait for testing
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

def start_server(port: int = 4242, gripper_enabled: bool = True):
    server = zerorpc.Server(NeroDualArmServer(gripper_enabled))
    server.bind(f"tcp://0.0.0.0:{port}")
    log.info(f"[SERVER] Listening on tcp://0.0.0.0:{port}")
    server.run()

# TODO: test server
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=4242)
    parser.add_argument('--no-gripper', action='store_true')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
    start_server(port=args.port, gripper_enabled=not args.no_gripper)