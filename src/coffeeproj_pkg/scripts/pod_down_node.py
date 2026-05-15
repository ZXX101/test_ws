#!/usr/bin/env python3
"""
pod_down.py
===========
TCP客户端节点：发送投放指令到192.168.100.108:2332
"""

import socket
import rospy

class PodDownClient:
    def __init__(self):
        self.host = rospy.get_param('~pod_host', '192.168.1.12')
        self.port = rospy.get_param('~pod_port', 2332)
        self.hex_data = bytes.fromhex(
            'A8 E5 48 00 01 00 00 D8 DC 00 00 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00 00 00 00 01 24 F2 DF 65 16 EE AA 16 A3 A0 00 00 0F B0 0C 06 15 E6 08 10 27 00 00 00 00 00 00 00 00 00 00 10 C8 7D'.replace(' ', '')
        )
        rospy.loginfo("[PodDown] Initialized, target: %s:%d", self.host, self.port)

    def send_command(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(5.0)
                s.connect((self.host, self.port))
                s.sendall(self.hex_data)
                response = s.recv(1024)
                rospy.loginfo("[PodDown] Command sent, response: %s", response.hex())
                return True
        except socket.error as e:
            rospy.logerr("[PodDown] Connection failed: %s", e)
            return False

if __name__ == '__main__':
    rospy.init_node('pod_down_node')
    client = PodDownClient()
    client.send_command()
    rospy.spin()