# Robotic Arm SDK User Guide

[CN](README.MD)

| ![ubuntu18.04](https://img.shields.io/badge/Ubuntu-18.04-orange.svg) | ![ubuntu20.04](https://img.shields.io/badge/Ubuntu-20.04-orange.svg) | ![ubuntu22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg) |
| ------------------------------------------------------------------ | ------------------------------------------------------------------ | ------------------------------------------------------------------ |
| ![Pass](https://img.shields.io/badge/Pass-blue.svg)                  | ![Pass](https://img.shields.io/badge/Pass-blue.svg)                  | ![Pass](https://img.shields.io/badge/Pass-blue.svg)                  |

Test:

| ![python3.6](https://img.shields.io/badge/Python-3.6-blue.svg) | ![python3.8](https://img.shields.io/badge/Python-3.8-blue.svg) | ![python3.10](https://img.shields.io/badge/Python-3.10-blue.svg) |
| ------------------------------------------------------------ | ------------------------------------------------------------ | -------------------------------------------------------------- |
| ![Pass](https://img.shields.io/badge/Pass-blue.svg)            | ![Pass](https://img.shields.io/badge/Pass-blue.svg)            | ![Pass](https://img.shields.io/badge/Pass-blue.svg)              |

| Description                   | Document                                  |
| ----------------------------- | ----------------------------------------- |
| CAN module usage              | [can_user](./docs/can_user(EN).MD)               |
| Robotic Arm interface details | [docs](./docs)                               |
| Robotic Arm DEMO              | [demos](./pyAgxArm/demos/DEMO_README(ZH).MD) |
| CHANGELOG                     | [CHANGELOG](./CHANGELOG.MD)                  |
| Q&A                           | [Q&amp;A](./docs/Q&A.MD)                     |

<details open>
  <summary><h2 style="display:inline">1 Installation</h2></summary>
  <div>

## 1.1 Install CAN tools

Note: The version of python-can should be higher than 3.3.4

```shell
pip3 install python-can
```

## 1.2 Install pyAgxArm

Please check if it's installed in conda environment

```bash
which pip3
```

```shell
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
pip3 install .
```

## 1.3 Check the details of pyAgxArm

```shell
pip3 show pyAgxArm
```

## 1.4 Uninstall pyAgxArm

```shell
pip3 uninstall pyAgxArm
```

## 1.5 Update pyAgxArm

```shell
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
pip3 install .
```

</div>
</details>



<details open>
  <summary><h2 style="display:inline">2 Activate CAN module and build CAN communication</h2></summary>
  <div>

## Activate the can module


Please refer to：[can_user](./docs/can_user.MD)

</div>
</details>



<details open>
  <summary><h2 style="display:inline">3 Start Up</h2></summary>
  <div>

## Read joint angles

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory
cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
# Create a Communication Instance and Start the Reading Thread
robot.connect()
while True:
    ja = robot.get_joint_angles()
    # If the feedback value is not None — note that the feedback value will be None initially when the program starts, as no data has been received yet.
    if ja is not None:
        print(ja.msg) # list
        print(ja.hz, ja.timestamp)
    time.sleep(0.005)# 200hz
```

</div>
</details





## Precautions

* When using the CAN protocol, you need to **activate the CAN module first** and set the **correct baud rate** before you can read manipulator messages or control the robotic arm.
* `create_agx_arm_config`: The activated CAN device name can be passed through the `channel` parameter during instantiation, and this name can be obtained via the `ifconfig` command.
* **The MIT protocol for the arm to control single-joint motors is an advanced function. Caution must be exercised when using it — improper use of this protocol may cause damage to the arm!!!**

## Contact US

You can  submit an issue on GitHub .

or join discord to communicate: [https://discord.gg/wrKYTxwDBd](https://discord.gg/wrKYTxwDBd)
