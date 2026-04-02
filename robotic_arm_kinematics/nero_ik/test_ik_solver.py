#!/usr/bin/env python3
"""
测试 nero_ik.ik_solver 的 IK 解算能力
使用直线往复轨迹进行测试，不依赖真实机器人硬件
"""

import math
import time
import numpy as np
import matplotlib.pyplot as plt
from ik_solver import (
    fk,
    NeroParams,
    ContinuityParams,
    ContinuityRuntimeState,
    solve_pose_continuous_with_state,
    ik_arm_angle_with_report,
    pose_error,
)


def wrap_to_pi(angle):
    """将角度归一化到 [-pi, pi]"""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def generate_linear_reciprocal_trajectory(
    start_pose, amplitude=0.05, cycles=3, points_per_cycle=100
):
    """
    生成直线往复轨迹
    :param start_pose: 起始位姿 [x, y, z, roll, pitch, yaw]
    :param amplitude: 振幅 (米)
    :param cycles: 往复周期数
    :param points_per_cycle: 每个周期的点数
    :return: 位姿列表
    """
    total_points = cycles * points_per_cycle
    poses = []

    for i in range(total_points):
        t = i / points_per_cycle  # 归一化时间 [0, cycles]
        # 使用正弦波实现平滑往复
        offset = amplitude * math.sin(2 * math.pi * t)

        pose = list(start_pose)
        pose[2] += offset  # Z 轴方向往复
        poses.append(pose)

    return poses


def pose_to_matrix(pose):
    """将 6D pose [x, y, z, roll, pitch, yaw] 转换为 4x4 齐次变换矩阵"""
    T = np.eye(4, dtype=float)

    # 提取位置
    T[:3, 3] = np.array(pose[:3], dtype=float)

    # 提取 RPY 并转换为旋转矩阵
    roll, pitch, yaw = pose[3], pose[4], pose[5]

    # RPY 转 rotation matrix (ZYX 顺序)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    R = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )
    T[:3, :3] = R

    return T


def matrix_to_pose(T):
    """将 4x4 齐次变换矩阵转换为 6D pose [x, y, z, roll, pitch, yaw]"""
    pos = T[:3, 3].tolist()
    R = T[:3, :3]

    # Rotation matrix 转 RPY (ZYX 顺序)
    pitch = math.atan2(-R[2, 0], math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))

    if abs(math.cos(pitch)) > 1e-10:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        # 万向节锁
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])

    return pos + [roll, pitch, yaw]


def test_ik_single_pose():
    """测试单个位姿的 IK 求解"""
    print("\n" + "=" * 60)
    print("测试 1: 单个位姿 IK 求解")
    print("=" * 60)

    p = NeroParams.default()

    # 使用一个合理的初始关节角
    q_init = np.array([0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0], dtype=float)

    # FK 得到目标位姿
    T_target = fk(q_init, p)
    target_pose = matrix_to_pose(T_target)

    print(f"初始关节角 (rad): {q_init}")
    print(f"初始关节角 (deg): {np.rad2deg(q_init)}")
    print(f"目标位姿: pos={target_pose[:3]}, rpy={np.rad2deg(target_pose[3:])}")

    # IK 求解
    q_prev = np.zeros(7, dtype=float)
    q_ik, solutions, report = ik_arm_angle_with_report(T_target, p=p, q_prev=q_prev)

    if q_ik is not None:
        print(f"\n✅ IK 求解成功!")
        print(f"求解关节角 (rad): {q_ik}")
        print(f"求解关节角 (deg): {np.rad2deg(q_ik)}")
        print(f"位姿误差: {report['pose_err_best']:.6f}")
        print(f"候选解数量: {report['candidate_count']}")
        print(f"方法: {report['method']}")

        # 验证 FK
        T_verify = fk(q_ik, p)
        verify_pose = matrix_to_pose(T_verify)
        print(f"\n验证位姿: pos={verify_pose[:3]}, rpy={np.rad2deg(verify_pose[3:])}")

        pos_err = np.linalg.norm(np.array(target_pose[:3]) - np.array(verify_pose[:3]))
        rot_err = np.linalg.norm(
            wrap_to_pi(np.array(target_pose[3:]) - np.array(verify_pose[3]))
        )
        print(f"位置误差: {pos_err * 1000:.3f} mm")
        print(f"姿态误差: {np.rad2deg(rot_err):.3f} deg")
    else:
        print("❌ IK 求解失败!")


def test_ik_trajectory():
    """测试轨迹 IK 求解"""
    print("\n" + "=" * 60)
    print("测试 2: 直线往复轨迹 IK 求解")
    print("=" * 60)

    p = NeroParams.default()
    continuity = ContinuityParams(
        local_theta0_window=0.35,
        local_theta0_count=41,
        w_vel=1.0,
        w_acc=0.25,
        w_pose=0.1,
        w_theta0=0.15,
        hysteresis_margin=0.03,
        enable_global_fallback=True,
    )

    # 从一个合理的初始关节角开始
    q_init = np.array([0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0], dtype=float)
    T_init = fk(q_init, p)
    start_pose = matrix_to_pose(T_init)

    print(f"起始关节角 (deg): {np.rad2deg(q_init)}")
    print(f"起始位姿: pos={start_pose[:3]}, rpy={np.rad2deg(start_pose[3:])}")

    # 生成直线往复轨迹
    amplitude = 0.10  # 5cm 振幅
    cycles = 3
    points_per_cycle = 100
    poses = generate_linear_reciprocal_trajectory(
        start_pose, amplitude=amplitude, cycles=cycles, points_per_cycle=points_per_cycle
    )

    print(f"\n轨迹参数: 振幅={amplitude*100}cm, 周期数={cycles}, 总点数={len(poses)}")

    # 初始化状态
    state = ContinuityRuntimeState(q_prev=q_init)

    # 求解轨迹
    q_solutions = []
    pose_errors = []
    solve_times = []
    failed_count = 0

    print("\n开始求解...")
    start_time = time.time()

    for i, pose in enumerate(poses):
        T_target = pose_to_matrix(pose)

        t0 = time.perf_counter()
        q_cmd, report, state = solve_pose_continuous_with_state(
            T_target, state=state, p=p, n_psi=181, continuity=continuity
        )
        solve_time = time.perf_counter() - t0

        if q_cmd is None:
            failed_count += 1
            q_solutions.append(None)
            pose_errors.append(float("inf"))
        else:
            q_solutions.append(q_cmd)
            # 计算位姿误差
            T_actual = fk(q_cmd, p)
            err = np.linalg.norm(pose_error(T_actual, T_target))
            pose_errors.append(err)

        solve_times.append(solve_time)

        # 每 50 个点打印一次进度
        if (i + 1) % 50 == 0:
            print(
                f"  进度: {i+1}/{len(poses)}, 失败: {failed_count}, "
                f"平均误差: {np.mean(pose_errors[-50:])*1000:.2f}mm, "
                f"平均耗时: {np.mean(solve_times[-50:])*1000:.2f}ms"
            )

    total_time = time.time() - start_time

    # 统计结果
    valid_errors = [e for e in pose_errors if e != float("inf")]
    valid_times = solve_times

    print("\n" + "-" * 40)
    print("求解完成!")
    print(f"总耗时: {total_time:.2f}s")
    print(f"成功率: {(len(poses) - failed_count) / len(poses) * 100:.1f}%")
    print(f"失败次数: {failed_count}")
    print(f"平均位姿误差: {np.mean(valid_errors)*1000:.3f} mm")
    print(f"最大位姿误差: {np.max(valid_errors)*1000:.3f} mm")
    print(f"平均求解耗时: {np.mean(valid_times)*1000:.2f} ms")
    print(f"最大求解耗时: {np.max(valid_times)*1000:.2f} ms")

    return q_solutions, pose_errors, poses


def plot_results(q_solutions, pose_errors, poses):
    """绘制结果图表"""
    print("\n" + "=" * 60)
    print("测试 3: 绘制结果图表")
    print("=" * 60)

    # 过滤有效解
    valid_q = [q for q in q_solutions if q is not None]
    valid_errors = [e for e in pose_errors if e != float("inf")]

    if len(valid_q) == 0:
        print("没有有效解，无法绘图")
        return

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # 1. 关节角轨迹
    q_array = np.array(valid_q)
    time_steps = np.arange(len(valid_q))

    ax1 = axes[0]
    for i in range(7):
        ax1.plot(time_steps, np.rad2deg(q_array[:, i]), label=f"Joint {i+1}")
    ax1.set_xlabel("Time Step")
    ax1.set_ylabel("Joint Angle (deg)")
    ax1.set_title("Joint Angles over Time")
    ax1.legend(loc="upper right", ncol=4)
    ax1.grid(True, alpha=0.3)

    # 2. 位姿误差
    ax2 = axes[1]
    ax2.plot(np.arange(len(valid_errors)), np.array(valid_errors) * 1000, "b-")
    ax2.set_xlabel("Time Step")
    ax2.set_ylabel("Pose Error (mm)")
    ax2.set_title("Pose Error over Time")
    ax2.grid(True, alpha=0.3)

    # 3. 目标 Z 轨迹
    ax3 = axes[2]
    z_targets = [p[2] for p in poses[: len(valid_q)]]
    ax3.plot(time_steps, np.array(z_targets) * 1000, "g-")
    ax3.set_xlabel("Time Step")
    ax3.set_ylabel("Z Position (mm)")
    ax3.set_title("Target Z Position (Reciprocal Motion)")
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()

    # 保存图片
    output_path = "/home/geist/pyAgxArm/robotic_arm_kinematics/nero_ik/test_ik_results.png"
    plt.savefig(output_path, dpi=150)
    print(f"图表已保存到: {output_path}")

    plt.show()


def test_continuity():
    """测试连续性跟踪能力"""
    print("\n" + "=" * 60)
    print("测试 4: 连续性跟踪能力")
    print("=" * 60)

    p = NeroParams.default()
    continuity = ContinuityParams()

    # 从一个初始关节角开始
    q_init = np.array([0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0], dtype=float)
    T_init = fk(q_init, p)
    start_pose = matrix_to_pose(T_init)

    # 生成小幅度快速往复轨迹
    amplitude = 0.02  # 2cm
    cycles = 5
    points_per_cycle = 50
    poses = generate_linear_reciprocal_trajectory(
        start_pose, amplitude=amplitude, cycles=cycles, points_per_cycle=points_per_cycle
    )

    print(f"轨迹参数: 振幅={amplitude*100}cm, 周期数={cycles}, 总点数={len(poses)}")

    # 初始化状态
    state = ContinuityRuntimeState(q_prev=q_init)

    # 计算关节角变化率
    q_prev = q_init.copy()
    max_dq = 0.0
    total_dq = 0.0

    for i, pose in enumerate(poses):
        T_target = pose_to_matrix(pose)
        q_cmd, report, state = solve_pose_continuous_with_state(
            T_target, state=state, p=p, n_psi=181, continuity=continuity
        )

        if q_cmd is not None:
            dq = np.linalg.norm(wrap_to_pi(q_cmd - q_prev))
            max_dq = max(max_dq, dq)
            total_dq += dq
            q_prev = q_cmd.copy()

    avg_dq = total_dq / len(poses)

    print(f"\n关节角变化统计:")
    print(f"  最大单步变化: {np.rad2deg(max_dq):.2f} deg")
    print(f"  平均单步变化: {np.rad2deg(avg_dq):.2f} deg")
    print(f"  总变化量: {np.rad2deg(total_dq):.2f} deg")


def main():
    print("=" * 60)
    print("nero_ik.ik_solver 测试程序")
    print("=" * 60)

    # 测试 1: 单个位姿 IK
    test_ik_single_pose()

    # 测试 2: 轨迹 IK
    q_solutions, pose_errors, poses = test_ik_trajectory()

    # 测试 3: 绘制结果
    plot_results(q_solutions, pose_errors, poses)

    # 测试 4: 连续性跟踪
    test_continuity()

    print("\n" + "=" * 60)
    print("所有测试完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()
