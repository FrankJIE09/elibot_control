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

# 发送命令到机器人并接收响应
def sendCMD(sock, cmd, params=None, id=1):
    if params:
        params = json.dumps(params)
    else:
        params = '{}'

    sendStr = f'{{"jsonrpc":"2.0", "method":"{cmd}", "params":{params}, "id":{id}}}\n'
    try:
        sock.sendall(sendStr.encode('utf-8'))
        response = sock.recv(1024)
        response_data = json.loads(response.decode('utf-8'))
        if "result" in response_data:
            return (True, response_data["result"], response_data["id"])
        elif "error" in response_data:
            return (False, response_data["error"], response_data["id"])
    except Exception as e:
        return (False, None, None)

if __name__ == "__main__":
    # 机器人IP地址
    robot_ip = "192.168.11.8"
    conSuc, sock = connectETController(robot_ip)
    points = [
        [0.0065, -103.9938, 102.2076, -88.2138, 90.0000, 0.0013],
        [-16.2806, -82.4996, 81.9848, -89.4851, 90.0000, -16.2858],
        [3.7679, -71.7544, 68.7276, -86.9732, 90.0000, 3.7627],
        [12.8237, -87.3028, 87.2361, -89.9333, 90.0000, 12.8185]
    ]
    if conSuc:
        # 设置机械臂伺服状态为ON
        suc, result, _ = sendCMD(sock, "set_servo_status", {"status": 1})
        time.sleep(1)
        for point in points:
            # 直线运动
            suc, result, _ = sendCMD(sock, "moveByLine", {
                "targetPos": point,
                "speed_type": 0,
                "speed": 200,
                "cond_type": 0,
                "cond_num": 7,
                "cond_value": 1
            })
            # 等待机器人到达目标点
            while True:
                suc, result, _ = sendCMD(sock, "getRobotState")
                if suc and result == 0:
                    break
                time.sleep(0.1)  # 适当延时，避免频繁请求
        # 断开连接
        sock.close()
