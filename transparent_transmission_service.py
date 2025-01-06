import json
import socket
import time


# 连接到机器人控制器
def connectETController(robot_ip, port=8055):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((robot_ip, port))
        return True, sock
    except Exception as e:
        print(f"Error connecting to {robot_ip}:{port} - {e}")
        return False, None


# 发送命令到机器人
def sendCMD(sock, method, params):
    # 构造JSON-RPC请求
    payload = {
        "jsonrpc": "2.0",
        "method": method,
        "params": params,
        "id": 1
    }
    try:
        # 发送命令
        msg = json.dumps(payload) + '\n'
        sock.sendall(msg.encode('utf-8'))

        # 接收响应
        response = sock.recv(1024)
        return True, json.loads(response), 1
    except Exception as e:
        print(f"Error sending command {method}: {e}")
        return False, str(e), 1


# 初始化透传服务
def transparent_transmission_init(sock, lookahead, t, smoothness, response_enable):
    params = {
        "lookahead": lookahead,
        "t": t,
        "smoothness": smoothness,
        "response_enable": response_enable
    }
    return sendCMD(sock, "transparent_transmission_init", params)


# 设置当前透传伺服目标关节点
def tt_set_current_servo_joint(sock, targetPos):
    params = {"targetPos": targetPos}
    return sendCMD(sock, "tt_set_current_servo_joint", params)


# 获取当前透传状态
def get_transparent_transmission_state(sock):
    return sendCMD(sock, "get_transparent_transmission_state", {})


# 添加透传伺服目标关节点信息到缓存
def tt_put_servo_joint_to_buf(sock, targetPos):
    params = {"targetPos": targetPos}
    return sendCMD(sock, "tt_put_servo_joint_to_buf", params)


# 清空透传缓存
def tt_clear_servo_joint_buf(sock):
    return sendCMD(sock, "tt_clear_servo_joint_buf", {})


if __name__ == "__main__":
    # 机器人IP地址
    robot_ip = "192.168.11.6"

    # 连接机器人控制器
    conSuc, sock = connectETController(robot_ip)
    if conSuc:
        # 初始化透传服务
        suc, result, id = transparent_transmission_init(sock, lookahead=400, t=10, smoothness=0.1, response_enable=0)
        print(f"Transparent Transmission Init: {suc}, {result}")

        # 设置透传伺服目标关节点
        P0 = [0, -90, 0, -90, 90, 0]  # 透传起始点
        suc, result, id = tt_set_current_servo_joint(sock, P0)
        print(f"Set Current Servo Joint: {suc}, {result}")

        # 获取当前透传状态
        suc, result, id = get_transparent_transmission_state(sock)
        print(f"Transparent Transmission State: {suc}, {result}")

        # 添加目标关节点到缓存
        suc, result, id = tt_put_servo_joint_to_buf(sock, P0)
        print(f"Put Servo Joint to Buffer: {suc}, {result}")

        # 清空透传缓存
        suc, result, id = tt_clear_servo_joint_buf(sock)
        print(f"Clear Servo Joint Buffer: {suc}, {result}")

    else:
        print("Failed to connect to the robot.")
