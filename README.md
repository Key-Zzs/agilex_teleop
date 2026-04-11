# AgxArm_teleop

Nero 机械臂双臂遥操作系统，基于 Agilex 机械臂 SDK 开发。

## 0 环境配置

### 0.1 创建 Conda 虚拟环境

```bash
# 创建名为 agilex_teleop 的 Python 3.10 环境
conda create -n agilex_teleop python=3.10 -y

# 激活环境
conda activate agilex_teleop
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
mkdir agilex_ws && cd agilex_ws
git clone --recursive https://github.com/Key-Zzs/agilex_teleop.git
cd agilex_teleop
```

如果忘记添加 `--recursive` 选项，需要手动克隆子模块：

```bash
cd agilex_ws/agilex_teleop

# 1. 初始化 submodule 配置
git submodule init

# 2. 拉取所有 submodule 的实际代码（递归，如果子模块还有子模块）
git submodule update --recursive
```

### 0.4 安装项目依赖

#### 方式一：使用 requirements.txt（推荐）

```bash
# 安装所有依赖
pip install -r requirements.txt

# 安装项目（开发模式）
pip install -e .
```

PS: 首次安装后运行 `pip install -r requirements.txt`，如果遇到冲突，可尝试升级 sympy 或降级 torch

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

#### 1.1.1 激活单个 CAN 模块

> 使用 `can_activate.sh` 脚本

1. 查看 USB 端口硬件地址

   拔掉所有 CAN 模块，只将连接到机械臂的 CAN 模块插入 PC，执行：

   ```shell
   bash find_all_can_port.sh
   ```

   记录下 `USB port` 的数值，例如 `3-1.4:1.0`。

2. 激活 CAN 设备

   假设上面的 `USB port` 数值为 `3-1.4:1.0`，执行：

   ```bash
   bash can_activate.sh can_piper 1000000 "3-1.4:1.0"
   ```

   含义：将硬件编码为 `3-1.4:1.0` 的 USB 端口上的 CAN 设备重命名为 `can_piper`，设定波特率为 `1000000`，并激活。

3. 检查是否激活成功

   执行 `ifconfig` 查看是否有 `can_piper`，如果有则 CAN 模块设置成功。

> 简化用法（单模块）
> 如果电脑只插入了一个 CAN 模块，可以直接执行：

```bash
bash can_activate.sh can0 1000000
```

#### 1.1.2 同时激活多个 CAN 模块

> 使用 `can_muti_activate.sh` 脚本

首先确定有多少个官方 CAN 模块被插入到电脑（以下示例假设为 2 个）。

> **提示：** 若当前电脑插入了 5 个 CAN 模块，也可以只激活指定的 CAN 模块。

1. 记录每个 CAN 模块对应的 USB 端口硬件地址

   逐个拔插 CAN 模块并一一记录每个模块对应的 USB 端口硬件地址。

   在 `can_muti_activate.sh` 中，`USB_PORTS` 参数中元素的数量为预激活的 CAN 模块数量。

   1. 将其中一个 CAN 模块单独插入 PC，执行：

      ```shell
      bash find_all_can_port.sh
      ```

      记录下 `USB port` 的数值，例如 `3-1.4:1.0`。

   2. 接着插入下一个 CAN 模块（**不可以**与上次插入的 USB 口相同），执行：

      ```shell
      bash find_all_can_port.sh
      ```

      记录下第二个 CAN 模块的 `USB port` 数值，例如 `3-1.1:1.0`。

      > **提示：** 如果未曾激活过，则第一个插入的 CAN 模块默认为 `can0`，第二个为 `can1`；若激活过，名字为之前激活过的名称。

2. 预定义 USB 端口、目标接口名称及波特率

   假设上面记录的 `USB port` 数值分别为 `3-1.4:1.0` 和 `3-1.1:1.0`，则将 `can_muti_activate.sh` 中的参数修改为：

   ```bash
   USB_PORTS["3-1.4:1.0"]="can_left:1000000"
   USB_PORTS["3-1.1:1.0"]="can_right:1000000"
   ```

   含义：`3-1.4:1.0` 端口的 CAN 设备重命名为 `can_left`，波特率 `1000000`，并激活。

3. 激活多个 CAN 模块

   执行：

   ```bash
   bash can_muti_activate.sh
   ```

4. 验证是否设置成功

   执行 `ifconfig` 查看是否有 `can_left` 和 `can_right`。


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
