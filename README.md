# AgxArm_teleop

Nero 机械臂双臂遥操作系统，基于 Agilex 机械臂 SDK 开发。

## 0 环境配置

### 0.1 创建 Conda 虚拟环境

```bash
# 创建名为 agxarm 的 Python 3.10 环境
conda create -n agxarm python=3.10 -y

# 激活环境
conda activate agxarm
```

### 0.2 克隆 lerobot 项目并安装 lerobot 框架
```bash
# 安装指定版本 0.3.4
# git checkout da5d2f3e9187fa4690e6667fe8b294cae49016d6
git clone https://github.com/huggingface/lerobot.git
cd lerobot
git checkout da5d2f3e9187fa4690e6667fe8b294cae49016d6
pip install -e .
```

### 0.3 克隆本项目

```bash
mkdir agile_ws && cd agile_ws
git clone https://github.com/Key-Zzs/agilex_teleop.git
cd agilex_teleop
```

### 0.4 安装项目依赖

#### 方式一：使用 requirements.txt（推荐）

```bash
# 安装所有依赖
pip install -r requirements.txt

# 安装项目（开发模式）
pip install -e .
```

#### 方式二：使用 pyproject.toml

```bash
# 基础安装
pip install -e .

# 包含仿真功能
pip install -e ".[sim]"

# 包含动力学功能
pip install -e ".[dynamics]"

# 全部功能
pip install -e ".[sim,dynamics]"
```

### 4. 安装 Pinocchio（可选）

Pinocchio 依赖较多，如果 `requirements.txt` 安装失败，可使用 conda-forge：

```bash
conda install -c conda-forge pinocchio eigenpy -y
```



## 1 快速开始

### 1.1 激活 CAN 设备

详见：
[docs/can_user.md](./docs/can_user.md#can-模块使用手册)

### 1.2 运行 nero 测试脚本

```bash
# nero 关节重置脚本
python nero/tests/reset.py
# nero 位置跟随 IK 测试脚本
python nero/tests/test_pos_flw_ik.py
```

## 2 启动遥操作服务

### 2.1 开启机械臂 server 服务端

```bash
# 开放端口 4242
udo iptables -I INPUT -p tcp --dport 4242 -j ACCEPT
# 启动机械臂 tcp 通信服务
python nero/teleop/interface/nero_interface_server.py --ip 0.0.0.0 --port 4242
# 确定 server 端 ip
ifconfig
```

### 2.2 遥操作端

## 注意事项

1. **安全警告**：运行重置脚本时，机械臂会瞬间失去力矩，务必用手扶稳！
2. **CAN 通信**：确保 CAN 接口正确配置（如 `can_left`, `can_right`）
3. **权限问题**：Linux 下可能需要设置 CAN 接口权限：
   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   ```

## 项目结构

```
AgxArm_teleop/
├── pyAgxArm/              # Agilex 机械臂 SDK
│   ├── api/               # API 接口
│   ├── protocols/         # 通信协议
│   └── utiles/            # 工具函数
├── nero/                  # Nero 双臂系统
│   ├── kinematics/        # 运动学
│   ├── teleop/            # 遥操作
│   └── tests/             # 测试脚本
├── requirements.txt       # 依赖列表
└── pyproject.toml         # 项目配置
```

## 开发人员资料

| 说明 | 文档 |
| --- | --- |
| ROS | [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) |
| CAN 模块手册 | [docs/can_user.md](./docs/can_user.md#can-模块使用手册) |
| Nero 首次使用 CAN 指南 | [docs/nero/first_time_user_guide_can.md](./docs/nero/first_time_user_guide_can.md#nero-首次使用指南can) |
| Nero API | [docs/nero/nero_api.md](./docs/nero/nero_api.md#nero-机械臂-api-使用文档) |
| AgxGripper API | [docs/effector/agx_gripper/agx_gripper_api.md](./docs/effector/agx_gripper/agx_gripper_api.md#agxgripper-夹爪-api-使用文档) |


## 来源

本仓库基于 [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm.git) 的代码开发，在此表示感谢。
