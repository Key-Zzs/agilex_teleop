import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

def main():
    print("==================================================")
    print("⚠️ 安全警告：执行重置会导致机械臂瞬间失去力矩！")
    print("如果机械臂当前在半空中，它会【立刻掉落】。")
    print("==================================================")
    
    # 强制要求用户确认，防止误触导致砸机
    input("请【用手扶稳机械臂】，确认安全后按 Enter 键继续...")

    # 1. 初始化并连接
    print("\n[1/4] 正在连接机械臂...")
    cfg = create_agx_arm_config(robot="nero", comm="can", channel="can_left")
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()
    time.sleep(0.5)

    # 2. 切换回正常控制模式（防止之前卡在透传或主从模式）
    print("[2/4] 正在切换至正常控制模式...")
    robot.set_normal_mode()
    time.sleep(0.5)

    # 3. 发送重置/失能指令，清除底层控制器的报错与急停锁死状态
    print("[3/4] 正在发送重置指令，清除底层错误状态...")
    robot.disable() 
    # 注意：根据文档，reset() 会重置模式并立刻失电。
    # 如果遇到 reset() 无法清除的顽固错误，也可以将其替换为 robot.disable()。
    time.sleep(1.0) # 给驱动器一点时间完成状态重置

    # 4. 尝试重新使能，验证是否真正修好
    print("[4/4] 尝试重新使能，验证通信与驱动器状态...")
    start_t = time.monotonic()
    is_enabled = False
    
    while time.monotonic() - start_t < 5.0: # 5秒超时
        if robot.enable():
            is_enabled = True
            break
        time.sleep(0.1)

    if is_enabled:
        print("\n✅ 重置成功！机械臂已恢复正常并重新获得力矩。")
        print("您可以按 Ctrl+C 退出此脚本，然后继续运行您的运动控制程序。")
        
        # 保持脚本运行，以便维持使能状态。您可以根据需要决定是否让它在这里挂起
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n退出重置脚本。正在安全放下机械臂...")
            robot.electronic_emergency_stop() # 退出时依然保持优雅的阻尼落下
    else:
        print("\n❌ 重置失败！未能重新使能。")
        print("可能原因：")
        print("1. 物理急停按钮（大红拍钮）被按下了，请旋转松开它。")
        print("2. 硬件发生了严重保护（如过温、过流），请尝试给控制箱彻底断电重启。")

if __name__ == "__main__":
    main()
