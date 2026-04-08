# %%
from nero_interface_server import NeroDualArmServer
import pdb
# %%
import logging
log = logging.getLogger(__name__)

# %%
import numpy as np
import time

# %%
server = NeroDualArmServer(gripper_enabled=True)

# %%
server.left_robot_go_home()

left_joints = server.left_robot_get_joint_positions()
print("Left joints:", left_joints)

# # %%
# # 单步 servo_p 测试

# # 构造一个小的 delta
# cur_pose = [-0.226,0,0.4,-1.57,0,-3.14]
# # delta_pose = np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0])
# target_pose = np.array([-0.403, 0.03, 0.265, 1.57, -0.35, -0.07])

# # 调用 servo_p（delta 模式）
# ret = server.servo_p(
#     robot_arm="left_robot",
#     cur_pose=cur_pose,
#     # pose=delta_pose.tolist(),
#     # delta=True
#     pose=target_pose.tolist(),
#     delta=False
# )

# time.sleep(1.5)

# fp = server.left_robot.get_flange_pose()
# new_pose = np.array(fp.msg, dtype=float)
# print("新位姿:", new_pose)
# pdb.set_trace()
# %%
# 连续 servo_p 测试
steps = 50
dt = 0.02  # 20Hz
cur_pose = [-0.226,0,0.4,-1.57,0,-3.14]
for i in range(steps):
    cur_pose[0] = cur_pose[0]-0.001
    delta_pose = np.array([-0.001, 0.0, 0.0, 0.0, 0.0, 0.0])
    time1 = time.time()
    server.servo_p("left_robot",cur_pose, delta_pose.tolist(), delta=True)
    time2 = time.time()
    print(f"Step {i+1}/{steps}, servo_p time: {(time2 - time1) * 1000:.2f} ms")
    time.sleep(dt)
