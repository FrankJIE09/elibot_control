import socket
import json
import time
import cv2
import numpy as np
from JoystickReaderProject.joystick_reader import Joystick  # 导入 read_joystick_data 函数


# 连接到机器人控制器
def connectETController(ip, port=8055):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ip, port))
        return (True, sock)
    except Exception as e:
        sock.close()
        return (False, None)


# 断开与机器人控制器的连接
def disconnectETController(sock):
    if sock:
        sock.close()
    return None


# 发送命令到机器人并接收响应
def sendCMD(sock, cmd, params=None, id=1):
    if not params:
        params = []
    else:
        params = json.dumps(params)

    sendStr = "{{\"jsonrpc\":\"2.0\",\"method\":\"{0}\",\"params\":{1},\"id\":{2}}}".format(cmd, params, id) + "\n"

    try:
        sock.sendall(bytes(sendStr, "utf-8"))
        ret = sock.recv(1024)
        jdata = json.loads(str(ret, "utf-8"))
        if "result" in jdata.keys():
            return (True, jdata["result"], jdata["id"])
        elif "error" in jdata.keys():
            return (False, jdata["error"], jdata["id"])
        else:
            return (False, None, None)
    except Exception as e:
        return (False, None, None)


# 示例：执行所有API
def main():
    robot_ip = "192.168.11.8"  # 机器人IP地址
    conSuc, sock = connectETController(robot_ip)
    port = "/dev/ttyUSB0"  # 根据实际连接更改 /dev/ttyUSB0 或 COM5
    num_axes = 3  # 操纵杆轴数量 (2 或 3)
    joystick = Joystick(port, num_axes)
    joystick.connect()\

    if conSuc:
        print("Connected to robot controller.")

        # 获取机器人当前伺服状态
        suc, servo_status, _ = sendCMD(sock, "getServoStatus")
        if suc:
            print("Current Servo Status:", servo_status)

            # 如果伺服没有启用，设置伺服为开启状态
            if servo_status == "false" or servo_status == "0":
                print("Enabling servo...")
                suc, result, _ = sendCMD(sock, "set_servo_status", {"status": 1})  # 1表示启用伺服
                if suc:
                    print("Servo enabled successfully.")
                else:
                    print("Failed to enable servo.")
                    return
            else:
                print("Servo is already enabled.")

        # 获取操纵杆数据并控制机器人
        while True:
            joystick_data = joystick.read_data()
            if joystick_data:
                if joystick_data[-1] != 0:
                    break
                x, y, z = np.array(joystick_data)[0:3] - 512
                print(f"Joystick data: X={x}, Y={y}, Z={z}")

                # 将操纵杆数据映射为机器人控制命令
                speed = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 机器人速度数组

                # 根据操纵杆数据调整机器人运动
                speed[0] = x * 0.1  # X轴速度调整
                speed[1] = y * 0.1  # Y轴速度调整
                if z is not None:
                    speed[2] = z * 0.1  # Z轴速度调整

                print(f"Sending movement command with speed: {speed}")
                acc = 100  # 加速度
                arot = 10  # 姿态加速度
                t = 0.1  # 执行时间
                suc, result, _ = sendCMD(sock, "moveBySpeedl", {"v": list(speed), "acc": acc, "arot": arot, "t": t})
                if suc:
                    print("Movement command sent successfully.")
                else:
                    print("Failed to send movement command.")
            else:
                print("No joystick data received.")

        # 断开连接
        disconnectETController(sock)
        joystick.close()
    else:
        print("Failed to connect to robot controller.")


if __name__ == "__main__":
    main()
