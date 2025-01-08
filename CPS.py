import socket
import json
import time


class CPSClient:
    def __init__(self, ip, port=8055):
        self.ip = ip
        self.port = port
        self.sock = None
        ret = self.connect()
        if not ret == True:
            RuntimeError


    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.ip, self.port))
            return True
        except Exception as e:
            if self.sock:
                self.sock.close()
            self.sock = None
            return False

    def disconnect(self):
        if self.sock:
            self.sock.close()
        self.sock = None

    def sendCMD(self, cmd, params=None, id=1):
        if not params:
            params = {}
        sendStr = json.dumps({
            "jsonrpc": "2.0",
            "method": cmd,
            "params": params,
            "id": id
        }) + "\n"
        try:
            self.sock.sendall(sendStr.encode('utf-8'))
            ret = self.sock.recv(1024)
            jdata = json.loads(ret.decode('utf-8'))
            if "result" in jdata:
                return True, jdata["result"], jdata["id"]
            elif "error" in jdata:
                return False, jdata["error"], jdata["id"]
            else:
                return False, None, None
        except Exception as e:
            return False, str(e), None

    def getJointPos(self):
        suc, joint_pose, id = self.sendCMD("get_joint_pos")
        if isinstance(joint_pose, str):
            joint_pose = json.loads(joint_pose)
        return joint_pose

    def getTcpPos(self, unit_type=0):
        params = {"unit_type": unit_type}
        suc, result_pose, _ = self.sendCMD("getTcpPose", params=params)
        if isinstance(result_pose, str):
            result_pose = json.loads(result_pose)
        return result_pose

    def moveByJoint(self, targetPos, speed, block=True):

        suc, result, _ = self.sendCMD("moveByJoint", {"targetPos": targetPos, "speed": speed})
        if suc:
            print(f"Move command sent: Target position {targetPos} with speed {speed}")
            if block:
                while True:
                    _, state, _ = self.sendCMD("getRobotState")
                    if state == '0':  # Assuming '0' means the robot has stopped
                        print("Robot reached the target position.")
                        break
                    time.sleep(0.1)
        else:
            print("Failed to send move command.")

    def moveBySpeedl(self, speed_l, acc, arot, t, id=1):
        params = {"v": speed_l, "acc": acc, "arot": arot, "t": t}
        return self.sendCMD("moveBySpeedl", params, id)

    def inverseKinematic(self, targetPose, unit_type=0):
        referencePos = self.getJointPos()
        params = {"targetPose": targetPose, "referencePos": referencePos, "unit_type": unit_type}
        _, iK_joint, _ = self.sendCMD("inverseKinematic", params)
        if isinstance(iK_joint, str):
            iK_joint = json.loads(iK_joint)
        return iK_joint

    def moveLine(self, target_pose):
        iK_joint = self.inverseKinematic(target_pose)
        self.moveByJoint(iK_joint, speed=10)

    def alignZAxis(self):
        # Get the current TCP pose
        current_pose = self.getTcpPos()
        current_pose[3:] = [180, 0, 0]
        target_pose = current_pose
        # Define a target pose with the desired orientation for Z-axis alignment
        # Here, we assume the orientation is given as Euler angles in degrees

        # Convert target orientation to the robot's required format if needed
        # This might involve converting Euler angles to quaternions, etc., depending on the robot

        # Calculate the joint positions needed to achieve this pose
        iK_joint = self.inverseKinematic(target_pose)

        # Move to the calculated joint positions
        self.moveByJoint(iK_joint, speed=10)


if __name__ == "__main__":
    robot_ip = "192.168.11.8"
    controller = CPSClient(robot_ip)
    if controller.connect():
        controller.alignZAxis()
