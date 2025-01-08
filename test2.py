import socket
import json


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
    # 参考点
    P000 = [0, -90, 90, -90, 90, 0]
    conSuc, sock = connectETController(robot_ip)
    if conSuc:
        # 获取机器人当前位姿信息
        suc, result_pose, id = sendCMD(sock, "get_tcp_pose")
        suc, result_joint, id = sendCMD(sock, "get_joint_pos")
        if isinstance(result_joint, str):
            result_joint = json.loads(result_joint)
        if isinstance(result_pose, str):
            result_pose = json.loads(result_pose)
        if suc:
            # 逆解函数2.0，带参考点位置逆解
            params = {"targetPose": result_pose,"referencePos":result_joint}
            suc, result, id = sendCMD(sock, "inverseKinematic", params)
            print(result)
        else:
            print("Failed to get TCP pose.")
