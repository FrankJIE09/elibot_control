import socket
import json
import time
import numpy as np
import cv2


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


# 以下是所有API封装

### MovementService APIs ###
def moveBySpeedl(sock, speed_l, acc, arot, t, id=1):
    params = {"v": speed_l, "acc": acc, "arot": arot, "t": t}
    return sendCMD(sock, "moveBySpeedl", params, id)


# 键盘控制函数（使用cv2的窗口进行显示）
def keyboardControl(sock):
    # 创建窗口
    cv2.namedWindow('Robot Control', cv2.WINDOW_NORMAL)

    # 初始化速度为0
    speed = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [X, Y, Z, Roll, Pitch, Yaw]

    # 定义不同轴的速度变化步长
    step_size_xyz = 10  # XYZ轴的步长
    step_size_rpy = 2  # RPY（Roll, Pitch, Yaw）轴的步长

    acc = 100  # 加速度
    arot = 10  # 姿态加速度
    t = 0.1  # 执行时间

    while True:
        # 显示当前速度
        img = np.zeros((300, 600, 3), dtype=np.uint8)
        cv2.putText(img, f"Speed: {speed}", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, "Control Keys:", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, "'w'/'s' -> X-axis", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "'a'/'d' -> Y-axis", (50, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "'q'/'e' -> Z-axis", (50, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "'r'/'f' -> Roll (R)", (50, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "'t'/'g' -> Pitch (P)", (50, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "'y'/'h' -> Yaw (Y)", (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "Press 'Esc' to exit", (50, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.imshow('Robot Control', img)

        # 等待用户输入
        key = cv2.waitKey(100) & 0xFF
        speed = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [X, Y, Z, Roll, Pitch, Yaw]

        # 控制XYZ速度
        if key == ord('w'):  # X轴正向
            speed[0] = step_size_xyz
        if key == ord('s'):  # X轴负向
            speed[0] = -step_size_xyz
        if key == ord('a'):  # Y轴负向
            speed[1] = -step_size_xyz
        if key == ord('d'):  # Y轴正向
            speed[1] = step_size_xyz
        if key == ord('q'):  # Z轴正向
            speed[2] = step_size_xyz
        if key == ord('e'):  # Z轴负向
            speed[2] = -step_size_xyz

        # 控制RPY速度
        if key == ord('r'):  # Roll正向
            speed[3] = step_size_rpy
        if key == ord('f'):  # Roll负向
            speed[3] = -step_size_rpy
        if key == ord('t'):  # Pitch正向
            speed[4] = step_size_rpy
        if key == ord('g'):  # Pitch负向
            speed[4] = -step_size_rpy
        if key == ord('y'):  # Yaw正向
            speed[5] = step_size_rpy
        if key == ord('h'):  # Yaw负向
            speed[5] = -step_size_rpy

        # 更新速度
        print(f"Current Speed: {speed}")

        # 发送速度命令到机器人
        suc, result, _ = moveBySpeedl(sock, list(speed), acc, arot, t)
        if suc:
            print("Movement command sent successfully.")
        else:
            print("Failed to send movement command.")

        # 按Esc键退出
        if key == 27:
            break

    # 关闭窗口
    cv2.destroyAllWindows()


# 示例：执行所有API
def main():
    robot_ip = "192.168.11.8"  # 机器人IP地址
    conSuc, sock = connectETController(robot_ip)

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

        # 使用键盘控制机器人
        keyboardControl(sock)

        # 断开连接
        disconnectETController(sock)
    else:
        print("Failed to connect to robot controller.")


if __name__ == "__main__":
    main()
