'''
Nero dual-arm robot interface server.
Provides zerorpc interface for dual-arm control.
'''

import zerorpc
import numpy as np
import logging
import time
from typing import Optional, List

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

        self.right_gripper = None\
        
        if gripper_enabled:
            try:
                if self.right_gripper is None:
                    self.right_gripper = self.right_robot.gripper()
            except Exception as e:
                log.error(f"[SERVER] Failed to initialize right gripper: {e}")

            log.info("=" * 50)
            log.info("Nero Dual-Gripper Server Ready")
            log.info("=" * 50)

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
        print("[DEBUG] move_j target:", target_list)

        self.left_robot.move_j(target_list)
        time.sleep(3.0)
        print("move to joint positions completed")
        
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
        
        target_list = target.tolist()
        print("[DEBUG] move_p target:", target_list)
    
        self.left_robot.set_speed_percent(30)
        
        self.left_robot.move_p(target_list)
        time.sleep(3.0)
        print("move to end-effector pose completed")

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
        print("[DEBUG] move_j target:", target_list)

        self.right_robot.move_j(target_list)
        time.sleep(3.0)
        print("move to joint positions completed")
    
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
        print("[DEBUG] move_p target:", target_list)
    
        self.right_robot.move_p(target_list)
        time.sleep(3.0)
        print("move to end-effector pose completed")

    # TODO
    # def dual_robot_move_to_ee_pose(self, left_pose: list, right_pose: list, delta: bool = False):
    #     if self._robot is None:
    #         return
    #     self._robot.dual_move_to_ee_pose(left_pose, right_pose, delta=delta, wait=True)
    
    # ==================== Go Home ====================

    # @Key-Zzs: [feat] left_robot_go_home() and right_robot_go_home()
    def left_robot_go_home(self):
        if self.left_robot is None:
            print("Left robot not initialized")
            return

        print("正在连接机械臂...")
        self.left_robot.connect()
        time.sleep(0.5)

        print("\n--- 开始重置 ---")
        print("正在清除急停锁死标志...")
        self.left_robot.reset() 
        time.sleep(1.0)  # 给主控足够的时间重启状态机
        
        print("正在切换回正常控制模式...")
        self.left_robot.set_normal_mode()
        time.sleep(0.5)
        print("--- 状态机重置完毕 ---\n")

        print("正在使能机械臂...")
        start_t = time.monotonic()
        is_enabled = False
        while time.monotonic() - start_t < 5.0:
            if self.left_robot.enable():
                is_enabled = True
                break
            time.sleep(0.5)

        if not is_enabled:
            print("使能失败！")
            return
            
        print("机械臂已使能上电！")

        self.left_robot.set_speed_percent(30)

        home = [0.1, -1.6, 0.0, 2.1, 0.0, 0.0, 1.3]
        
        print("[DEBUG] Moving to home:", home)
        self.left_robot.move_j(home)

        result = self.left_robot.get_joint_angles()
        print("当前关节角度:", result.msg)

        time.sleep(3.0)  # 等待运动完成
        print("已回到初始位置")
    
    def right_robot_go_home(self):
        if self.right_robot is None:
            print("Right robot not initialized")
            return

        print("正在连接机械臂...")
        self.right_robot.connect()
        time.sleep(0.5)

        print("\n--- 开始重置 ---")
        print("正在清除急停锁死标志...")
        self.right_robot.reset() 
        time.sleep(1.0)  # 给主控足够的时间重启状态机
        
        print("正在切换回正常控制模式...")
        self.right_robot.set_normal_mode()
        time.sleep(0.5)
        print("--- 状态机重置完毕 ---\n")

        print("正在使能机械臂...")
        start_t = time.monotonic()
        is_enabled = False
        while time.monotonic() - start_t < 5.0:
            if self.right_robot.enable():
                is_enabled = True
                break
            time.sleep(0.5)

        if not is_enabled:
            print("使能失败！")
            return
            
        print("机械臂已使能上电！")

        self.right_robot.set_speed_percent(30)

        #TODO：not test yet, use 'left_robot_move_to_joint_positions(left_joints_target, delta=False)' to determine
        home = [0.1, -1.6, 0.0, 2.1, 0.0, 0.0, 1.3]

        print("[DEBUG] Moving to home:", home)
        self.left_robot.move_j(home)
        
        result = self.right_robot.get_joint_angles()
        print("当前关节角度:", result.msg)

        time.sleep(3.0)  # 等待运动完成
        print("已回到初始位置")

    # ==================== ServoJ Control (Joint Servo) ====================
    
    def servo_j(self, arm_name: str, joints: list, t: float = 0.1, 
                lookahead_time: float = 0.05, gain: float = 300) -> bool:
        """
        Send ServoJ with ABSOLUTE joint angles (radians).
        Args:
            joints: Joint angles in RADIANS
        """
        if self._robot is None:
            return True
        # Convert radians to degrees for ROS wrapper
        joints_deg = np.degrees(joints).tolist()
        return self._robot.servo_j(arm_name, joints_deg, t, lookahead_time, gain)
    
    def servo_j_delta(self, arm_name: str, delta_joints: list, t: float = 0.1,
                      lookahead_time: float = 0.05, gain: float = 300) -> bool:
        """
        Send ServoJ with RELATIVE joint increments (radians).
        Args:
            delta_joints: Joint increments in RADIANS
        """
        if self._robot is None:
            return True
        # Convert radians to degrees for ROS wrapper
        delta_joints_deg = np.degrees(delta_joints).tolist()
        return self._robot.servo_j_delta(arm_name, delta_joints_deg, t, lookahead_time, gain)
    
    # ==================== ServoP Control (Pose Servo) ====================
    
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
            arm_name: 'left' or 'right'
            pose: Target pose [x, y, z, rx, ry, rz] (m, radians)
            current_joints: Current joints for reference (radians)
        Returns:
            Joint angles (radians) or None if failed
        """
        if self._robot is None:
            return [0.0] * 7
        
        # ROS wrapper expects mm and degrees for IK
        pose_array = np.array(pose)
        pose_nero = np.concatenate([
            pose_array[:3] * 1000,  # m -> mm
            np.degrees(pose_array[3:])  # rad -> deg
        ]).tolist()
        
        # Convert current joints to degrees
        current_joints_deg = None
        if current_joints is not None:
            current_joints_deg = np.degrees(current_joints).tolist()
        
        # Solve IK (ROS wrapper returns degrees)
        result_deg = self._robot.inverse_kinematics(arm_name, pose_nero, current_joints_deg)
        
        # Convert back to radians
        if result_deg is not None:
            return np.radians(result_deg).tolist()
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