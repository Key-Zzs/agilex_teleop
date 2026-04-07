'''
Nero dual-arm robot interface server.
Provides zerorpc interface for dual-arm control.
'''

import zerorpc
import numpy as np
import logging
import time
from typing import Optional, List
import sys, os

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT_DIR)

log = logging.getLogger(__name__)


class NeroDualArmServer:
    """Dual-arm Nero server interface."""
    
    def __init__(self, gripper_enabled: bool = True):
        self.gripper_enabled = gripper_enabled

        # Initialize left arm
        self.left_robot = None
        
        try:
            
            from pyAgxArm import create_agx_arm_config, AgxArmFactory
            cfg = create_agx_arm_config(robot="nero", comm="can", channel="can_left")
            self.left_robot = AgxArmFactory.create_arm(cfg)
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
            cfg = create_agx_arm_config(robot="nero", comm="can", channel="can_right")
            self.right_robot = AgxArmFactory.create_arm(cfg)
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
                    self.left_gripper = self.left_robot.gripper()
            except Exception as e:
                log.error(f"[SERVER] Failed to initialize left gripper: {e}")

        # Initialize right gripper
        self.right_gripper = None
        
        if gripper_enabled:
            try:
                if self.right_gripper is None:
                    self.right_gripper = self.right_robot.gripper()
            except Exception as e:
                log.error(f"[SERVER] Failed to initialize right gripper: {e}")

            log.info("=" * 50)
            log.info("Nero Dual-Gripper Server Ready")
            log.info("=" * 50)

        # Initialize right IK solver
        self.left_ik_solver = None
        self.right_ik_solver = None

        try:
            joint_limits = [(-3.14, 3.14)] * 7  # TODO: 用真实关节限位替换

            # 直接复用你已有类
            from ik_test.test_pos_flw_ik import AnalyticIkSolver

            self.left_ik_solver = AnalyticIkSolver(joint_limits, dt=0.02)
            self.right_ik_solver = AnalyticIkSolver(joint_limits, dt=0.02)

            # 初始化状态
            left_q = self.left_robot_get_joint_positions()
            right_q = self.right_robot_get_joint_positions()

            self.left_ik_solver.init_state(left_q)
            self.right_ik_solver.init_state(right_q)

            log.info("[SERVER] IK solver initialized")

        except Exception as e:
            log.error(f"[ERROR] IK init failed: {e}")

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

        #TODO: 限幅

        target_list = target.tolist()
        log.info("[DEBUG] move_j target:", target_list)

        self.left_robot.move_j(target_list)
        time.sleep(3.0)
        log.info("move to joint positions completed")
        
    def left_robot_move_to_ee_pose(self, pose: list, delta: bool = False):
        """Move left arm to end-effector pose [x, y, z, roll, pitch, yaw] (m, rad)."""
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

        #TODO: 限幅
        
        target_list = target.tolist()
        log.info("[DEBUG] move_p target:", target_list)
    
        self.left_robot.set_speed_percent(30)

        self.left_robot.move_p(target_list)
        time.sleep(3.0)
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

        #TODO: 限幅

        target_list = target.tolist()
        log.info("[DEBUG] move_j target:", target_list)

        self.right_robot.move_j(target_list)
        time.sleep(3.0)
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
        
        #TODO: 限幅

        target_list = target.tolist()
        log.info("[DEBUG] move_p target:", target_list)
    
        self.right_robot.move_p(target_list)
        time.sleep(3.0)
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

        home = [0.1, -1.6, 0.0, 2.1, 0.0, 0.0, 1.3]

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
        home = [0.1, -1.6, 0.0, 2.1, 0.0, 0.0, 1.3]

        log.info("[DEBUG] Moving to home:", home)
        self.left_robot.move_j(home)
        
        result = self.right_robot.get_joint_angles()
        log.info("当前关节角度:", result.msg)

        time.sleep(3.0)  # 等待运动完成
        log.info("已回到初始位置")

    # ==================== ServoJ Control (Joint Servo) ====================
    
    def servo_j(self, robot_arm: str, joints: list) -> bool:
        """
        直接输入某个机械臂名称与目标关节角度（度），控制机械臂运动。

        Args:
            robot_arm: "left_robot" or "right_robot"
            joints: 7维绝对关节角度（度）

        Returns:
            bool: 成功返回 True，失败返回 False
        """
        try:
            joints = np.asarray(joints, dtype=float)
            if joints.shape[0] != 7:
                raise ValueError(f"Expected 7 joints, got {joints.shape[0]}")

            log.info(f"[DEBUG] servo_j target (degree): {joints}")
            
            for i in range(7):
                joints[i] = np.deg2rad(joints[i])
                
            if robot_arm == "left_robot":
                self.left_robot_move_to_joint_positions(joints.tolist(), delta=False)
            elif robot_arm == "right_robot":
                self.right_robot_move_to_joint_positions(joints.tolist(), delta=False)
            else:
                raise ValueError("robot_arm must be 'left_robot' or 'right_robot'")

            return True

        except Exception as e:
            log.error(f"[ERROR] servo_j failed: {e}")
            return False


    def servo_j_delta(self, robot_arm: str, delta_joints: list) -> bool:
        """
        直接输入某个机械臂名称与关节角度增量（度），控制机械臂运动。

        Args:
            robot_arm: "left_robot" or "right_robot"
            delta_joints: 7维关节角度增量（度）

        Returns:
            bool: 成功返回 True，失败返回 False
        """
        try:
            delta_joints = np.asarray(delta_joints, dtype=float)
            if delta_joints.shape[0] != 7:
                raise ValueError(f"Expected 7 joints, got {delta_joints.shape[0]}")

            log.info(f"[DEBUG] servo_j_delta target (degree): {delta_joints}")

            for i in range(7):
                delta_joints[i] = np.deg2rad(delta_joints[i])

            if robot_arm == "left_robot":
                self.left_robot_move_to_joint_positions(delta_joints.tolist(), delta=True)
            elif robot_arm == "right_robot":
                self.right_robot_move_to_joint_positions(delta_joints.tolist(), delta=True)
            else:
                raise ValueError("robot_arm must be 'left_robot' or 'right_robot'")

            return True

        except Exception as e:
            log.error(f"[ERROR] servo_j_delta failed: {e}")
            return False
    
    # ==================== ServoP Control (Pose Servo) ====================
    
    # TODO: scaling for what?
    def servo_p(self, arm_name: str, pose: list) -> bool:
        """
        Send ServoP with target pose [x, y, z, rx, ry, rz] (m, radians).
        Args:
            pose: Target pose in METERS and RADIANS
        """
        if self._robot is None:
            return True
        # Convert m to mm, radians to degrees for ROS wrapper
        pose_array = np.array(pose)
        pose_nero = np.concatenate([
            pose_array[:3] * 1000,  # m -> mm
            np.degrees(pose_array[3:])  # rad -> deg
        ]).tolist()
        return self._robot.servo_p(arm_name, pose_nero)
    
    def servo_p_delta(self, arm_name: str, delta_pose: list) -> bool:
        """
        Send ServoP with RELATIVE pose increments (m, radians).
        Args:
            delta_pose: Pose increments in METERS and RADIANS
        """
        if self._robot is None:
            return True
        # Convert m to mm, radians to degrees for ROS wrapper
        delta_array = np.array(delta_pose)
        delta_nero = np.concatenate([
            delta_array[:3] * 1000,  # m -> mm
            np.degrees(delta_array[3:])  # rad -> deg
        ]).tolist()
        return self._robot.servo_p_delta(arm_name, delta_nero)
    
    # ==================== Inverse Kinematics ====================
    
    def inverse_kinematics(self, arm_name: str, pose: list, current_joints: list = None):
        """
        Solve IK using Nero controller.

        Args:
            arm_name: 'left_robot', 'right_robot', 'left', or 'right'
            pose: Target pose [x, y, z, rx, ry, rz] (m, radians)
            current_joints: Current joints for reference (radians)

        Returns:
            Joint angles (radians) or None if failed
        """
        try:
            pose = np.asarray(pose, dtype=float)
            if pose.shape[0] != 6:
                raise ValueError(f"Expected 6 pose values, got {pose.shape[0]}")

            if arm_name in ("left", "left_robot"):
                solver = self.left_ik_solver
            elif arm_name in ("right", "right_robot"):
                solver = self.right_ik_solver
            else:
                raise ValueError("Invalid arm_name")

            if solver is None:
                log.error("[ERROR] IK solver not initialized")
                return None

            # solver
            q = solver.solve(pose.tolist())

            return q.tolist()

        except Exception as e:
            log.error(f"[ERROR] inverse_kinematics failed: {e}")
            return None
    
    # ==================== Gripper (Placeholder) ====================
    
    def left_gripper_initialize(self): pass
    def left_gripper_goto(self, *args, **kwargs): pass
    def left_gripper_grasp(self, *args, **kwargs): pass
    def left_gripper_get_state(self): return {"width": 0.04, "is_moving": False, "is_grasped": False}
    
    def right_gripper_initialize(self): pass
    def right_gripper_goto(self, *args, **kwargs): pass
    def right_gripper_grasp(self, *args, **kwargs): pass
    def right_gripper_get_state(self): return {"width": 0.04, "is_moving": False, "is_grasped": False}
    
    def gripper_initialize(self):
        self.left_gripper_initialize()
        self.right_gripper_initialize()
    
    # ==================== Utility ====================
    
    def stop(self, arm_name: str):
        if self._robot is None:
            return
        self._robot.stop(arm_name)


def start_server(port: int = 4242, gripper_enabled: bool = True):
    server = zerorpc.Server(NeroDualArmServer())
    server.bind(f"tcp://0.0.0.0:{port}")
    log.info(f"[SERVER] Listening on port {port}")
    server.run()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=4242)
    parser.add_argument('--no-gripper', action='store_true')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
    start_server(port=args.port, gripper_enabled=not args.no_gripper)