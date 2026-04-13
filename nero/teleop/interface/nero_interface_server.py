'''
Nero dual-arm robot interface server.
Provides zerorpc interface for dual-arm control.
'''

# python nero_interface/nero_interface_server.py --ip 0.0.0.0 --port 4242
# sudo iptables -I INPUT -p tcp --dport 4242 -j ACCEPT

# Version 3: Task queue + state machine

import zerorpc
import numpy as np
import logging
import time
import math
from typing import Optional, List, Dict, Callable
import sys, os
import pdb
import threading
from enum import Enum
from dataclasses import dataclass, field
import queue
from datetime import datetime

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

# ==================== Task Type Enum ====================
class TaskType(Enum):
    """任务类型枚举"""
    LEFT_GRIPPER_GOTO = "left_gripper_goto"
    RIGHT_GRIPPER_GOTO = "right_gripper_goto"
    ROBOT_GO_HOME = "robot_go_home"
    LEFT_ROBOT_GO_HOME = "left_robot_go_home"
    RIGHT_ROBOT_GO_HOME = "right_robot_go_home"
    SERVO_P_OL = "servo_p_OL"

# ==================== Task Status Enum ====================
class TaskStatus(Enum):
    """任务状态枚举"""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"

# ==================== Task Priority Enum ====================
class TaskPriority(Enum):
    """任务优先级"""
    LOW = 0
    NORMAL = 1
    HIGH = 2
    EMERGENCY = 3

# ==================== Task Data Class ====================
@dataclass
class Task:
    """任务数据类"""
    task_id: str
    task_type: TaskType
    status: TaskStatus = TaskStatus.PENDING
    priority: TaskPriority = TaskPriority.NORMAL
    created_at: datetime = field(default_factory=datetime.now)
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    result: Optional[bool] = None
    error_message: Optional[str] = None
    params: Dict = field(default_factory=dict)
    callback: Optional[Callable] = None
    
    def __lt__(self, other):
        """用于优先级队列比较"""
        return self.priority.value > other.priority.value

# ==================== Task Queue Manager ====================
class TaskQueueManager:
    """任务队列管理器"""
    
    def __init__(self):
        self.task_queue = queue.PriorityQueue()
        self.active_tasks: Dict[str, Task] = {}
        self.completed_tasks: Dict[str, Task] = {}
        self.task_counter = 0
        self.lock = threading.Lock()
        self.max_concurrent_tasks = 3
    
    def add_task(self, task_type: TaskType, params: Dict = None, 
                  priority: TaskPriority = TaskPriority.NORMAL,
                  callback: Callable = None) -> str:
        """添加任务到队列"""
        with self.lock:
            self.task_counter += 1
            task_id = f"{task_type.value}_{self.task_counter}_{int(time.time() * 1000)}"
            
            task = Task(
                task_id=task_id,
                task_type=task_type,
                priority=priority,
                params=params or {},
                callback=callback
            )
            
            self.task_queue.put(task)
            log.info(f"[TASK_QUEUE] Added task: {task_id} (type={task_type.value}, priority={priority.name})")
            return task_id
    
    def get_next_task(self) -> Optional[Task]:
        """获取下一个待执行任务"""
        with self.lock:
            if self.task_queue.empty():
                return None
            
            task = self.task_queue.get()
            
            if len(self.active_tasks) >= self.max_concurrent_tasks:
                self.task_queue.put(task)
                return None
            
            task.status = TaskStatus.RUNNING
            task.started_at = datetime.now()
            self.active_tasks[task.task_id] = task
            
            log.info(f"[TASK_QUEUE] Started task: {task.task_id} (type={task.task_type.value})")
            return task
    
    def complete_task(self, task_id: str, success: bool, error_message: str = None):
        """完成任务"""
        with self.lock:
            if task_id in self.active_tasks:
                task = self.active_tasks[task_id]
                task.status = TaskStatus.COMPLETED if success else TaskStatus.FAILED
                task.completed_at = datetime.now()
                task.result = success
                task.error_message = error_message
                
                self.completed_tasks[task_id] = task
                del self.active_tasks[task_id]
                
                if task.callback:
                    try:
                        task.callback(success, error_message)
                    except Exception as e:
                        log.error(f"[TASK_QUEUE] Callback failed for task {task_id}: {e}")
                
                log.info(f"[TASK_QUEUE] Completed task: {task_id} (success={success})")
    
    def cancel_task(self, task_id: str) -> bool:
        """取消任务"""
        with self.lock:
            if task_id in self.active_tasks:
                task = self.active_tasks[task_id]
                task.status = TaskStatus.CANCELLED
                task.completed_at = datetime.now()
                self.completed_tasks[task_id] = task
                del self.active_tasks[task_id]
                return True
            return False
    
    def get_task_status(self, task_id: str) -> Optional[TaskStatus]:
        """获取任务状态"""
        with self.lock:
            if task_id in self.active_tasks:
                return self.active_tasks[task_id].status
            if task_id in self.completed_tasks:
                return self.completed_tasks[task_id].status
            return None
    
    def get_queue_size(self) -> int:
        """获取队列大小"""
        with self.lock:
            return self.task_queue.qsize()
    
    def get_active_task_count(self) -> int:
        """获取活跃任务数量"""
        with self.lock:
            return len(self.active_tasks)
    
    def has_active_home_task(self) -> bool:
        """检查是否有home任务在执行"""
        with self.lock:
            home_task_types = [
                TaskType.LEFT_ROBOT_GO_HOME,
                TaskType.RIGHT_ROBOT_GO_HOME,
                TaskType.ROBOT_GO_HOME
            ]
            for task in self.active_tasks.values():
                if task.task_type in home_task_types:
                    return True
            return False

# ==================== Task State Machine ====================
class TaskStateMachine:
    """任务状态机"""
    
    def __init__(self, server_instance):
        self.server = server_instance
        self.state_transitions = {
            TaskStatus.PENDING: [TaskStatus.RUNNING, TaskStatus.CANCELLED],
            TaskStatus.RUNNING: [TaskStatus.COMPLETED, TaskStatus.FAILED, TaskStatus.CANCELLED],
            TaskStatus.COMPLETED: [],
            TaskStatus.FAILED: [TaskStatus.PENDING],
            TaskStatus.CANCELLED: []
        }
    
    def can_transition(self, current_status: TaskStatus, new_status: TaskStatus) -> bool:
        """检查状态转换是否合法"""
        return new_status in self.state_transitions.get(current_status, [])
    
    def execute_task(self, task: Task) -> bool:
        """执行任务"""
        try:
            log.info(f"[STATE_MACHINE] Executing task: {task.task_id} (type={task.task_type.value})")
            
            success = False
            error_msg = None
            
            if task.task_type == TaskType.LEFT_GRIPPER_GOTO:
                success = self._execute_left_gripper_goto(task)
            elif task.task_type == TaskType.RIGHT_GRIPPER_GOTO:
                success = self._execute_right_gripper_goto(task)
            elif task.task_type == TaskType.LEFT_ROBOT_GO_HOME:
                success = self._execute_left_robot_go_home(task)
            elif task.task_type == TaskType.RIGHT_ROBOT_GO_HOME:
                success = self._execute_right_robot_go_home(task)
            elif task.task_type == TaskType.ROBOT_GO_HOME:
                success = self._execute_robot_go_home(task)
            elif task.task_type == TaskType.SERVO_P_OL:
                success = self._execute_servo_p_ol(task)
            else:
                error_msg = f"Unknown task type: {task.task_type}"
                log.error(f"[STATE_MACHINE] {error_msg}")
            
            return success
            
        except Exception as e:
            error_msg = f"Task execution failed: {e}"
            log.error(f"[STATE_MACHINE] {error_msg}", exc_info=True)
            return False
    
    def _execute_left_gripper_goto(self, task: Task) -> bool:
        """执行左夹爪任务"""
        width = task.params.get('width', 0.0)
        force = task.params.get('force', 1.0)
        return self.server._gripper_goto(self.server.left_gripper, width, force)
    
    def _execute_right_gripper_goto(self, task: Task) -> bool:
        """执行右夹爪任务"""
        width = task.params.get('width', 0.0)
        force = task.params.get('force', 1.0)
        return self.server._gripper_goto(self.server.right_gripper, width, force)
    
    def _execute_left_robot_go_home(self, task: Task) -> bool:
        """执行左机械臂回零任务"""
        def _home_and_update():
            try:
                success = self.server._go_home("left_robot")
                
                if success:
                    with self.server.home_flag_lock:
                        current_pose = self.server._get_current_pose(
                            self.server.left_robot, 
                            self.server.left_ik_solver
                        )
                        if current_pose is not None:
                            self.server.target_pose_left = current_pose
                            self.server.left_state = "IDLE"
                            log.info(f"[HOME] Updated target_pose_left: {current_pose}")
                        else:
                            log.warning(f"[HOME] Failed to get current pose after home")
                else:
                    with self.server.home_flag_lock:
                        self.server.left_state = "IDLE"
                    log.error(f"[HOME] Left robot go home failed")
                    
            except Exception as e:
                log.error(f"[HOME ERROR] {e}")
                with self.server.home_flag_lock:
                    self.server.left_state = "IDLE"
        
        with self.server.home_flag_lock:
            self.server.left_state = "HOMING"
            self.server.left_home_busy = True
        
        import threading
        threading.Thread(target=_home_and_update, daemon=True).start()
        
        return True
    
    def _execute_right_robot_go_home(self, task: Task) -> bool:
        """执行右机械臂回零任务"""
        def _home_and_update():
            try:
                success = self.server._go_home("right_robot")
                
                if success:
                    with self.server.home_flag_lock:
                        current_pose = self.server._get_current_pose(
                            self.server.right_robot, 
                            self.server.right_ik_solver
                        )
                        if current_pose is not None:
                            self.server.target_pose_right = current_pose
                            self.server.right_state = "IDLE"
                            log.info(f"[HOME] Updated target_pose_right: {current_pose}")
                        else:
                            log.warning(f"[HOME] Failed to get current pose after home")
                else:
                    with self.server.home_flag_lock:
                        self.server.right_state = "IDLE"
                    log.error(f"[HOME] Right robot go home failed")
                    
            except Exception as e:
                log.error(f"[HOME ERROR] {e}")
                with self.server.home_flag_lock:
                    self.server.right_state = "IDLE"
        
        with self.server.home_flag_lock:
            self.server.right_state = "HOMING"
            self.server.right_home_busy = True
        
        import threading
        threading.Thread(target=_home_and_update, daemon=True).start()
        
        return True
    
    def _execute_robot_go_home(self, task: Task) -> bool:
        """执行双机械臂回零任务"""
        def _home_and_update():
            try:
                left_success = self.server._go_home("left_robot")
                right_success = self.server._go_home("right_robot")
                
                if left_success:
                    with self.server.home_flag_lock:
                        current_pose = self.server._get_current_pose(
                            self.server.left_robot, 
                            self.server.left_ik_solver
                        )
                        if current_pose is not None:
                            self.server.target_pose_left = current_pose
                            self.server.left_state = "IDLE"
                            log.info(f"[HOME] Updated target_pose_left: {current_pose}")
                        else:
                            log.warning(f"[HOME] Failed to get left current pose after home")
                else:
                    with self.server.home_flag_lock:
                        self.server.left_state = "IDLE"
                    log.error(f"[HOME] Left robot go home failed")
                
                if right_success:
                    with self.server.home_flag_lock:
                        current_pose = self.server._get_current_pose(
                            self.server.right_robot, 
                            self.server.right_ik_solver
                        )
                        if current_pose is not None:
                            self.server.target_pose_right = current_pose
                            self.server.right_state = "IDLE"
                            log.info(f"[HOME] Updated target_pose_right: {current_pose}")
                        else:
                            log.warning(f"[HOME] Failed to get right current pose after home")
                else:
                    with self.server.home_flag_lock:
                        self.server.right_state = "IDLE"
                    log.error(f"[HOME] Right robot go home failed")
                    
            except Exception as e:
                log.error(f"[HOME ERROR] {e}")
                with self.server.home_flag_lock:
                    self.server.left_state = "IDLE"
                    self.server.right_state = "IDLE"
        
        with self.server.home_flag_lock:
            self.server.left_state = "HOMING"
            self.server.right_state = "HOMING"
            self.server.left_home_busy = True
            self.server.right_home_busy = True
        
        import threading
        threading.Thread(target=_home_and_update, daemon=True).start()
        
        return True
    
    def _execute_servo_p_ol(self, task: Task) -> bool:
        """执行位姿控制任务"""
        robot_arm = task.params.get('robot_arm')
        pose = task.params.get('pose')
        delta = task.params.get('delta', False)
        return self.server._servo_p_OL_internal(robot_arm, pose, delta)

class NeroDualArmServer:
    """Dual-arm Nero server interface with task queue and state machine."""
    
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
        self.track_freq = 50.0
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
        self.control_freq = 20.0
        self.control_dt = 1.0 / self.control_freq

        self.control_running = True
        
        # 机械臂访问锁，防止任务执行器和control loop同时访问
        self.left_robot_lock = threading.Lock()
        self.right_robot_lock = threading.Lock()
        
        # home任务标志锁，确保线程可见性
        self.home_flag_lock = threading.Lock()
        
        # IK缓存变量（减少overrun）
        self.last_q_cmd_left = None
        self.last_q_cmd_right = None
        self.ik_counter = 0
        self.ik_divider = 2  # IK降到50Hz
        
        # move_js降频变量（解决overrun）
        self.cmd_counter = 0
        self.cmd_divider = 5  # 100Hz → 实际发送20Hz
        
        # 状态机变量
        self.left_state = "IDLE"
        self.right_state = "IDLE"

        # ==================== Task Queue & State Machine ====================
        self.task_queue_manager = TaskQueueManager()
        self.task_state_machine = TaskStateMachine(self)
        self.task_executor_running = True
        
        # 启动任务执行线程
        self.task_executor_thread = threading.Thread(
            target=self.task_executor_loop,
            daemon=True
        )
        self.task_executor_thread.start()
        log.info("[TASK_EXECUTOR] Task executor thread started")

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

    def _go_home(self, robot_arm: str) -> bool:
        """
        机械臂回零，先重置状态机清除急停锁死，再切换回正常模式，最后使能上电并运动到初始位置
        
        Args:
            robot_arm: "left_robot" or "right_robot"
        
        Returns:
            bool: 成功返回True，失败返回False
        """
        try:
            if robot_arm == "left_robot":
                robot = self.left_robot
                ik_solver = self.left_ik_solver
                home = [0.0, -0.13, 0.0, 1.87, 0.0, 0.0, -0.17]
                target_pose_attr = "target_pose_left"
                print(f"current_pose_left: {self.target_pose_left}")
            elif robot_arm == "right_robot":
                robot = self.right_robot
                ik_solver = self.right_ik_solver
                home = [0.0, -0.13, 0.0, 1.87, 0.0, 0.0, -0.17]
                target_pose_attr = "target_pose_right"
                print(f"current_pose_right: {self.target_pose_right}")
            else:
                message = f"Invalid robot_arm: {robot_arm}"
                log.error(f"[{robot_arm}_go_home] {message}")
                return False
            
            if robot is None:
                message = "Robot not initialized"
                log.error(f"[{robot_arm}_go_home] {message}")
                return False

            log.info(f"[{robot_arm}_go_home] Starting home sequence...")

            # Step 1: 重置状态机
            log.info(f"[{robot_arm}_go_home] Resetting state machine...")
            try:
                robot.reset()
                time.sleep(1.0)
            except Exception as e:
                message = f"Failed to reset robot: {e}"
                log.error(f"[{robot_arm}_go_home] {message}")
                return False

            # Step 2: 切换到正常模式
            log.info(f"[{robot_arm}_go_home] Switching to normal mode...")
            try:
                robot.set_normal_mode()
                time.sleep(0.5)
            except Exception as e:
                message = f"Failed to set normal mode: {e}"
                log.error(f"[{robot_arm}_go_home] {message}")
                return False

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
                return False

            log.info(f"[{robot_arm}_go_home] Robot enabled successfully")

            # Step 4: 设置速度并运动到home位置
            try:
                robot.set_speed_percent(30)
                log.info(f"[{robot_arm}_go_home] Moving to home position: {home}")
                robot.move_j(home)
            except Exception as e:
                message = f"Failed to move to home: {e}"
                log.error(f"[{robot_arm}_go_home] {message}")
                return False

            # Step 5: 等待运动完成（使用 motion_status 检查）
            log.info(f"[{robot_arm}_go_home] Waiting for motion to complete...")
            timeout = 10.0
            start_t = time.monotonic()
            motion_in_progress = True
            
            while time.monotonic() - start_t < timeout:
                try:
                    status = robot.get_arm_status()
                    if status is not None and hasattr(status, 'msg'):
                        motion_status = status.msg.motion_status
                        if motion_status == 0:  # 0 表示没有运动
                            motion_in_progress = False
                            log.info(f"[{robot_arm}_go_home] Motion completed")
                            break
                        else:
                            log.debug(f"[{robot_arm}_go_home] Motion in progress, status={motion_status}")
                except Exception as e:
                    log.debug(f"[{robot_arm}_go_home] Failed to get arm status: {e}")
                
                time.sleep(0.1)
            
            if motion_in_progress:
                log.warning(f"[{robot_arm}_go_home] Motion timeout after {timeout}s, proceeding anyway")

            # Step 6: 更新目标位姿
            if robot is not None and ik_solver is not None:
                from pyAgxArm.utiles.tf import rot_to_rpy
                try:
                    current_joints = None
                    max_retries = 25
                    retry_interval = 0.1
                    
                    for retry in range(max_retries):
                        try:
                            ja = robot.get_joint_angles()
                            if ja is not None and hasattr(ja, 'msg'):
                                current_joints = ja.msg
                                if current_joints is not None and len(current_joints) == 7:
                                    log.info(f"[{robot_arm}_go_home] Successfully got joint angles at retry {retry + 1}")
                                    print(f"Current joints: {current_joints}")
                                    break
                        except Exception as e:
                            log.debug(f"[{robot_arm}_go_home] get_joint_angles attempt {retry + 1} failed: {e}")
                        
                        if retry < max_retries - 1:
                            time.sleep(retry_interval)
                    
                    if current_joints is not None:
                        q_current = np.array(current_joints, dtype=float)
                        T_fk = fk(q_current, ik_solver.nero_params)
                        fk_xyz = np.asarray(T_fk[:3, 3], dtype=float)
                        fk_rpy = np.asarray(rot_to_rpy(T_fk[:3, :3].tolist()), dtype=float)
                        target_pose = np.concatenate([fk_xyz, fk_rpy])
                        setattr(self, target_pose_attr, target_pose)
                        print(f"[{robot_arm}_go_home] Updated {target_pose_attr}: {target_pose}")
                    else:
                        log.warning(f"[{robot_arm}_go_home] Failed to get current joints after {max_retries} retries, using home position for FK")
                        try:
                            q_current = np.array(home, dtype=float)
                            T_fk = fk(q_current, ik_solver.nero_params)
                            fk_xyz = np.asarray(T_fk[:3, 3], dtype=float)
                            fk_rpy = np.asarray(rot_to_rpy(T_fk[:3, :3].tolist()), dtype=float)
                            target_pose = np.concatenate([fk_xyz, fk_rpy])
                            setattr(self, target_pose_attr, target_pose)
                            log.info(f"[{robot_arm}_go_home] Updated {target_pose_attr} using home position: {target_pose}")
                        except Exception as e:
                            message = f"Failed to update pose using home position: {e}"
                            log.error(f"[{robot_arm}_go_home] {message}")
                            return False
                except Exception as e:
                    message = f"Failed to update pose: {e}"
                    log.error(f"[{robot_arm}_go_home] {message}")
                    return False

            message = "Home sequence completed successfully"
            log.info(f"[{robot_arm}_go_home] {message}")
            # Step 6: 等待运动完成
            log.info(f"[{robot_arm}_go_home] Waiting for motion to complete...")
            # time.sleep(3.0)
            return True

        except Exception as e:
            message = f"Unexpected error during home sequence: {e}"
            log.error(f"[{robot_arm}_go_home] {message}", exc_info=True)
            return False

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
        # if self.left_robot is None or self.right_robot is None:
        #     return
        # self.left_robot_go_home()
        # self.right_robot_go_home()

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
        接收来自客户端的末端位姿控制指令（支持增量/绝对），通过任务队列执行。

        该函数将位姿控制任务加入任务队列，由任务执行器处理。

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
            
            task_id = self.task_queue_manager.add_task(
                task_type=TaskType.SERVO_P_OL,
                params={'robot_arm': robot_arm, 'pose': pose.tolist(), 'delta': delta},
                priority=TaskPriority.NORMAL
            )
            log.info(f"[SERVER] Servo P OL task queued: {task_id}")
            return True

        except Exception as e:
            log.error(f"[ERROR] servo_p_OL failed: {e}")
            return False
    
    def _servo_p_OL_internal(self, robot_arm: str, pose: list, delta: bool) -> bool:
        """
        内部位姿控制函数，由任务执行器调用
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
            log.error(f"[ERROR] _servo_p_OL_internal failed: {e}")
            return False  
    
    # ==================== Gripper (Placeholder) ====================

    def _gripper_goto(self, gripper, width: float, force: float = 1.0) -> bool:
        if not self.gripper_enabled or gripper is None:
            log.warning("[SERVER] Gripper not available")
            return False

        width = float(max(0.0, min(width, 0.1)))

        log.info(f"[SERVER] Gripper goto: width={width:.3f}, force={force}")

        try:
            gripper.move_gripper(width=width, force=force)
            return True
        except Exception as e:
            log.error(f"[SERVER] Gripper goto failed: {e}")
            return False

    def left_gripper_goto(self, width, force):
        """
        非阻塞设置左夹爪目标开度，通过任务队列执行
        
        Args:
            width: 夹爪开度 (0.0-0.1 m)
            force: 夹爪力 (0.0-1.0)
        
        Returns:
            bool: 成功返回True
        """
        task_id = self.task_queue_manager.add_task(
            task_type=TaskType.LEFT_GRIPPER_GOTO,
            params={'width': width, 'force': force},
            priority=TaskPriority.LOW
        )
        log.info(f"[SERVER] Left gripper goto task queued: {task_id}")
        return True

    def right_gripper_goto(self, width, force):
        """
        非阻塞设置右夹爪目标开度，通过任务队列执行
        
        Args:
            width: 夹爪开度 (0.0-0.1 m)
            force: 夹爪力 (0.0-1.0)
        
        Returns:
            bool: 成功返回True
        """
        task_id = self.task_queue_manager.add_task(
            task_type=TaskType.RIGHT_GRIPPER_GOTO,
            params={'width': width, 'force': force},
            priority=TaskPriority.LOW
        )
        log.info(f"[SERVER] Right gripper goto task queued: {task_id}")
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
    
    # ==================== Task Executor Loop ====================
    def task_executor_loop(self):
        """
        任务执行器循环线程，负责从任务队列中获取任务并执行
        """
        log.info("[TASK_EXECUTOR] Task executor loop started")
        
        while self.task_executor_running:
            try:
                # 获取下一个任务
                task = self.task_queue_manager.get_next_task()
                
                if task is not None:
                    # 执行任务
                    success = self.task_state_machine.execute_task(task)
                    
                    # 完成任务
                    self.task_queue_manager.complete_task(
                        task.task_id,
                        success=success,
                        error_message=None if success else "Task execution failed"
                    )
                else:
                    # 没有任务，短暂休眠
                    time.sleep(0.01)
                    
            except Exception as e:
                log.error(f"[TASK_EXECUTOR] Error in task executor loop: {e}", exc_info=True)
                time.sleep(0.1)
        
        log.info("[TASK_EXECUTOR] Task executor loop stopped")
    
    # ==================== Home Task Interface ====================
    def left_robot_go_home(self):
        """
        左机械臂回零，通过任务队列执行
        
        Returns:
            bool: 成功返回True
        """
        task_id = self.task_queue_manager.add_task(
            task_type=TaskType.LEFT_ROBOT_GO_HOME,
            params={},
            priority=TaskPriority.HIGH
        )
        log.info(f"[SERVER] Left robot go home task queued: {task_id}")
        return True
    
    def right_robot_go_home(self):
        """
        右机械臂回零，通过任务队列执行
        
        Returns:
            bool: 成功返回True
        """
        task_id = self.task_queue_manager.add_task(
            task_type=TaskType.RIGHT_ROBOT_GO_HOME,
            params={},
            priority=TaskPriority.HIGH
        )
        log.info(f"[SERVER] Right robot go home task queued: {task_id}")
        return True
    
    def robot_go_home(self):
        """
        双机械臂回零，通过任务队列执行
        
        Returns:
            bool: 成功返回True
        """
        task_id = self.task_queue_manager.add_task(
            task_type=TaskType.ROBOT_GO_HOME,
            params={},
            priority=TaskPriority.HIGH
        )
        log.info(f"[SERVER] Robot go home task queued: {task_id}")
        return True
    
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
        sync_interval = 20  # 每10个控制周期同步一次IK状态（100ms）
        
        while self.control_running:
            start = time.perf_counter()
            sync_counter += 1

            try:
                with self.home_flag_lock:
                    has_home_task = self.left_home_busy or self.right_home_busy
                
                # ===== 方案2：home期间彻底停control_loop动作 =====
                if self.left_state == "HOMING" or self.right_state == "HOMING":
                    time.sleep(self.control_dt)
                    continue
                
                # ===== LEFT SERVO =====
                if self.left_state != "IDLE":
                    pass
                else:
                    if self.target_pose_left is not None:
                        self.ik_counter += 1
                        self.cmd_counter += 1
                        
                        # IK降频
                        if self.ik_counter % self.ik_divider == 0:
                            try:
                                q_cmd = self.left_ik_solver.solve(self.target_pose_left)
                                if q_cmd is not None:
                                    if isinstance(q_cmd, np.ndarray):
                                        q_cmd = q_cmd.tolist()
                                    self.last_q_cmd_left = q_cmd
                            except Exception as e:
                                log.error(f"[CONTROL] Left arm IK solve failed: {e}")
                        
                        # ❗关键：move_js降频
                        if self.cmd_counter % self.cmd_divider == 0:
                            if self.last_q_cmd_left is not None:
                                try:
                                    self.left_robot.move_js(self.last_q_cmd_left)
                                except Exception as e:
                                    log.error(f"[CONTROL] Left arm move_js failed: {e}")

                # ===== RIGHT SERVO =====
                if self.right_state != "IDLE":
                    pass
                else:
                    if self.target_pose_right is not None:
                        self.ik_counter += 1
                        self.cmd_counter += 1
                        
                        # IK降频
                        if self.ik_counter % self.ik_divider == 0:
                            try:
                                q_cmd = self.right_ik_solver.solve(self.target_pose_right)
                                if q_cmd is not None:
                                    if isinstance(q_cmd, np.ndarray):
                                        q_cmd = q_cmd.tolist()
                                    self.last_q_cmd_right = q_cmd
                            except Exception as e:
                                log.error(f"[CONTROL] Right arm IK solve failed: {e}")
                        
                        # 关键：move_js降频
                        if self.cmd_counter % self.cmd_divider == 0:
                            if self.last_q_cmd_right is not None:
                                try:
                                    self.right_robot.move_js(self.last_q_cmd_right)
                                except Exception as e:
                                    log.error(f"[CONTROL] Right arm move_js failed: {e}")

                # Gripper和Home任务现在由任务执行器处理，不再在此处处理

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
                log.warning(f"[WARN] Control loop overrun: {dt*1000:.2f} ms")

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