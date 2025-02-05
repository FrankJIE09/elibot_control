import socket
import json
import time


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

# 新增直线匀速运动的API封装
def moveBySpeedl(sock, speed_l, acc, arot, t, id=1):
    params = {"v": speed_l, "acc": acc, "arot": arot, "t": t}
    return sendCMD(sock, "moveBySpeedl", params, id)


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
                    time.sleep(1)  # 每1秒检查一次，避免过多的请求
    else:
        print("Failed to send move command.")


def moveByPath(sock):
    return sendCMD(sock, "moveByPath")


def clearPathPoint(sock):
    return sendCMD(sock, "clearPathPoint")


def addPathPoint(sock, wayPoint, moveType=0, speed=50, circular_radius=0):
    params = {"wayPoint": wayPoint, "moveType": moveType, "speed": speed, "circular_radius": circular_radius}
    return sendCMD(sock, "addPathPoint", params)


### 示例：执行所有API ###
def main():
    robot_ip = "192.168.11.6"  # 机器人IP地址
    conSuc, sock = connectETController(robot_ip)

    if conSuc:
        print("Connected to robot controller.")

        # 获取机器人当前伺服状态
        suc, servo_status, _ = getServoStatus(sock)
        if suc:
            print("Current Servo Status:", servo_status)

            # 如果伺服没有启用，设置伺服为开启状态
            if servo_status == "false" or servo_status == "0":
                print("Enabling servo...")
                suc, result, _ = setServoStatus(sock, 1)  # 1表示启用伺服
                if suc:
                    print("Servo enabled successfully.")
                else:
                    print("Failed to enable servo.")
                    return
            else:
                print("Servo is already enabled.")

        # 获取机器人当前关节位置
        suc, joint_pos, _ = getJointPos(sock)
        i = 0
        while True:
            if i > 10:
                i = 0
            else:
                i = i + 1
            speed_l = [1.0, 1.0 * i, 0.0, 0.0, 0.0, 1.0]  # 速度值
            acc = 100  # 位移加速度
            arot = 10  # 姿态加速度
            t = 0.1  # 执行时间
            suc, result, _ = moveBySpeedl(sock, speed_l, acc, arot, t)
            if suc:
                print("Linear motion started successfully.")
            else:
                print("Failed to start linear motion.")

        # 断开连接
        disconnectETController(sock)
    else:
        print("Failed to connect to robot controller.")


if __name__ == "__main__":
    main()
