# -*- coding: utf-8 -*-
import time
import math
from servo_controller import ServoController
from led_controller import LedIndicator

def parse_pose_data_line(line):
    """
    尝试手动解析一行 POSE 报文，
    假设格式为：
      POSE,<s0>,<s1>,...,<s15>,<timestamp>,...<其他数据>
    其中前 16 项是舵机原始角度
    """
    if not line.startswith("POSE,"):
        return None

    parts = line.strip().split(',')
    if len(parts) < 17:
        return None

    try:
        # 取前 16 个舵机角度
        raw_servo = [int(x) for x in parts[1:17]]
        timestamp = int(parts[17])
        return {
            'servo_raw': raw_servo,
            'timestamp': timestamp
        }
    except:
        return None


def map_raw_to_q(raw_angles, THETA_MID, S):
    """
    把原始舵机角度映射成 q 空间角度
    Args:
        raw_angles: list of 8 原始角度
        THETA_MID: mid 向量
        S: 方向系数
    """
    q_list = []
    for i in range(len(raw_angles)):
        q = S[i] * (raw_angles[i] - THETA_MID[i])
        q_list.append(q)
    return q_list


def main():
    # 创建控制器实例 (注意: 下位机波特率已升级为 460800)
    ctrl = ServoController(port='/dev/ttyTHS1', baudrate=460800)
    
    # 尝试初始化 LED，如果失败则忽略（防止没有该硬件时报错）
    try:
        led = LedIndicator(pin=12)
    except:
        led = None

    if not ctrl.connect():
        print("无法连接串口，退出")
        return

    try:
        print("=== 1. 初始化: 慢速复位到中位 (1.0s) ===")
        # 使用 duration=1000 让舵机慢速归位，避免上电猛冲
        ctrl.go_to_mid(duration=1000)
        time.sleep(1.2)

        print("\n=== 2. 高精度正弦波控制测试 (50Hz) ===")
        print("提示: 利用 STM32 新固件的浮点插值能力，实现丝滑运动")
        
        # --- 核心控制参数 ---
        FREQ = 50.0             # 控制频率 50Hz (每秒发送50次指令)
        DT = 1.0 / FREQ         # 控制周期 0.02s (20ms)
        DURATION_MS = int(DT * 1000) # 20ms，与发送周期严格匹配，实现无缝插值
        
        # 记录开始时间
        start_time = time.time()
        
        # 运行一个持续 10 秒的动态测试
        while True:
            t = time.time() - start_time
            if t > 10.0: break # 10秒后结束测试

            # --- A. 生成高精度浮点轨迹 ---
            # 生成一个 0.5Hz 的正弦波，幅度 +/- 30.5 度
            # 注意：这里计算出的是浮点数 (如 15.342 度)，
            # 现在的下位机可以精确执行这个角度，而不会被截断为 15 度
            angle_float = 30.5 * math.sin(2 * math.pi * 0.5 * t)
            
            # --- B. 构造批量指令 ---
            # 格式: (id, angle_float, duration_ms)
            # 我们让左侧腿 (0,2,4,6) 和右侧腿 (1,3,5,7) 反相运动
            batch = []
            for sid in range(8):
                if sid % 2 == 0:
                    target = angle_float       # 左侧
                else:
                    target = -angle_float      # 右侧
                
                # 将 (ID, 浮点角度, 20ms) 加入列表
                batch.append((sid, target, DURATION_MS))
            
            # --- C. 发送指令 ---
            ctrl.send_servo_batch(batch)
            
            # --- D. 状态指示 (LED 闪烁) ---
            if led:
                # 每秒闪烁一次
                if int(t) % 2 == 0: led.on()
                else: led.off()

            # --- E. 接收反馈 (可选) ---
            # 如果需要读取下位机回传的 POSE 数据，可以在这里调用 ctrl.receive_data()
            # if ctrl.serial_port.in_waiting > 0:
            #     line = ctrl.serial_port.readline().decode('utf-8', errors='ignore')
            #     data = parse_pose_data_line(line)
            #     if data: print(f"FB: {data['servo_raw'][0]}")

            # --- F. 精确控频 ---
            # 计算本次循环已消耗时间，sleep 剩余时间以维持 50Hz
            elapsed = time.time() - (start_time + t)
            sleep_time = DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        print("\n=== 测试结束: 恢复中位 ===")
        ctrl.go_to_mid(duration=1000)
        time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n[用户中断] 正在停止...")
    
    finally:
        # 安全退出前确保复位
        time.sleep(0.1)
        if led: led.off()
        ctrl.disconnect()
        print("[结束] 程序已退出")

if __name__ == "__main__":
    main()
