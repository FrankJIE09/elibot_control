import socket
import json
import time

import cv2
import numpy as np
from Jacobian.base_2_joint_velocity_elibot import calculate_joint_velocity


# 连接到机器人控制器
def connectETController(ip, port=8055):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ip, port))
        return (True, sock)
    except Exception as e:
        sock.close()
        return (False, None)


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

### ServoService APIs ###
def getServoStatus(sock):
    return sendCMD(sock, "getServoStatus")


def setServoStatus(sock, status):
    return sendCMD(sock, "set_servo_status", {"status": status})


def syncMotorStatus(sock):
    return sendCMD(sock, "syncMotorStatus")


def clearAlarm(sock):
    return sendCMD(sock, "clearAlarm")


def getMotorStatus(sock):
    return sendCMD(sock, "getMotorStatus")


### ParamService APIs ###
def getRobotState(sock):
    return sendCMD(sock, "getRobotState")


def getRobotMode(sock):
    return sendCMD(sock, "getRobotMode")


def getJointPos(sock):
    return sendCMD(sock, "get_joint_pos")


def getTcpPose(sock, coordinate_num=-1, tool_num=-1, unit_type=1):
    params = {"coordinate_num": coordinate_num, "tool_num": tool_num, "unit_type": unit_type}
    return sendCMD(sock, "get_tcp_pose", params)


def getMotorSpeed(sock):
    return sendCMD(sock, "get_motor_speed")


def getCurrentCoord(sock):
    return sendCMD(sock, "getCurrentCoord")


def getCycleMode(sock):
    return sendCMD(sock, "getCycleMode")


def getCurrentJobLine(sock):
    return sendCMD(sock, "getCurrentJobLine")


def getCurrentEncode(sock):
    return sendCMD(sock, "getCurrentEncode")


def getToolNumber(sock):
    return sendCMD(sock, "getToolNumber")


def setToolNumber(sock, tool_num):
    return sendCMD(sock, "setToolNumber", {"tool_num": tool_num})


def getUserNumber(sock):
    return sendCMD(sock, "getUserNumber")


def setUserNumber(sock, user_num):
    return sendCMD(sock, "setUserNumber", {"user_num": user_num})


def getMotorTorque(sock):
    return sendCMD(sock, "get_motor_torque")


def getPathPointIndex(sock):
    return sendCMD(sock, "getPathPointIndex")


def setCurrentCoord(sock, coord_mode):
    return sendCMD(sock, "setCurrentCoord", {"coord_mode": coord_mode})


### MovementService APIs ###
def moveByJoint(sock, targetPos, speed, block=True):
    # 发送目标位置和速度的命令
    suc, result, _ = sendCMD(sock, "moveByJoint", {"targetPos": targetPos, "speed": speed})

    if suc:
        print(f"Move command sent: Target position {targetPos} with speed {speed}")

        if block:
            # 如果是阻塞模式，持续检查机器人状态
            while True:
                suc, state, _ = getRobotState(sock)
                if suc and state == '0':  # 0表示机器人已停止（或已到达目标）
                    print("Robot reached the target position.")
                    break
                else:
                    print("1Robot is still moving...")
                    time.sleep(0.1)  # 每1秒检查一次，避免过多的请求
    else:
        print("Failed to send move command.")


# 直线运动（阻塞模式）
def moveByLine(sock, point):
    # 发送直线运动命令
    suc, result, _ = sendCMD(sock, "moveByLine", {
        "targetPos": point,
        "speed_type": 0,
        "speed": 200,
        "cond_type": 0,
        "cond_num": 7,
        "cond_value": 1
    })

    if suc:
        print(f"Line movement command sent to {point}")

        # 阻塞等待直到机器人到达目标点
        while True:
            suc, result, _ = sendCMD(sock, "getRobotState")
            if suc and result == "0":  # 0表示机器人已停止
                print(f"Robot reached target point: {point}")
                break
            else:
                print("Robot is still moving...")
                time.sleep(0.1)  # 每秒检查一次状态
    else:
        print(f"Failed to send line movement command to {point}")


def get_all_dh(sock, num_parameters):
    """
    获取所有DH参数
    :param sock: 控制器的socket连接
    :param num_parameters: DH参数的总数
    :return: 包含所有DH参数的列表
    """
    all_dh_parameters = []

    for index in range(num_parameters):
        ret, result, id = sendCMD(sock, "getDH", {"index": index})
        if ret:
            print(f"DH参数 (index {index}) = {result}")
            all_dh_parameters.append(result)
        else:
            print(f"获取DH参数失败 (index {index})，错误信息: {result.get('message', '未知错误')}")

    return all_dh_parameters


def moveByPath(sock):
    return sendCMD(sock, "moveByPath")


def clearPathPoint(sock):
    return sendCMD(sock, "clearPathPoint")


def addPathPoint(sock, wayPoint, moveType=0, speed=50, circular_radius=0):
    params = {"wayPoint": wayPoint, "moveType": moveType, "speed": speed, "circular_radius": circular_radius}
    return sendCMD(sock, "addPathPoint", params)


def moveBySpeedl(sock, speed_l, acc, arot, t, id=1):
    params = {"v": speed_l, "acc": acc, "arot": arot, "t": t}
    return sendCMD(sock, "moveBySpeedl", params, id)


def moveBySpeedj(sock, speed_j, acc=20, t=0.01, id=1):
    params = {"vj": speed_j, "acc": acc, "t": t}
    return sendCMD(sock, "moveBySpeedj", params, id)


### 示例：执行所有API ###
# 示例：执行所有API
def main2():
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

        # 获取机器人当前TCP位置
        suc, joint_pos, _ = sendCMD(sock, "getTcpPose")
        if suc:
            print("Current Joint Position:", joint_pos)

            # 确保 joint_pos 是列表类型
            if isinstance(joint_pos, str):
                joint_pos = json.loads(joint_pos)
            # joint_pos = np.array(joint_pos)
            # joint_pos[3:] = np.rad2deg(joint_pos[3:])
            # joint_pos = joint_pos.tolist()
            joint3 = float(joint_pos[2])

            # 增加Z轴10
            new_joint3 = joint3
            print(f"Moving joint3 from {joint3} to {new_joint3}")

            # 创建新的目标位置，将Z轴增加10
            new_joint_pos = joint_pos.copy()  # 使用copy()来生成新的列表
            new_joint_pos[2] = new_joint3
            suc, result, _ = sendCMD(sock, "inverseKinematic", {
                "targetPose": new_joint_pos,
                "referencePos": "P000"
            })
            # 移动机械臂到新的目标位置，并启用阻塞模式
            print("Moving to new joint position with Z+10 (blocking mode):")

            # 进行直线运动
            print("Starting line movements...")
            moveByLine(sock, new_joint_pos)

        # 断开连接
        disconnectETController(sock)
    else:
        print("Failed to connect to robot controller.")


def keyboardControl(sock):
    # 创建窗口
    cv2.namedWindow('Robot Control', cv2.WINDOW_NORMAL)

    # 初始化速度为0
    speed = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [X, Y, Z, Roll, Pitch, Yaw]

    # 定义不同轴的速度变化步长
    step_size_xyz = 3  # XYZ轴的步长
    step_size_rpy = 3  # RPY（Roll, Pitch, Yaw）轴的步长

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
        key = cv2.waitKey(2000) & 0xFF
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
        suc, theta, _ = getJointPos(sock)
        theta = eval(theta)

        speedj = calculate_joint_velocity(speed, theta)
        speedj = speedj / 10
        print(f"Current Speed: {speedj}")

        # 发送速度命令到机器人
        suc, result, _ = moveBySpeedj(sock, list(speedj), acc, t)
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
