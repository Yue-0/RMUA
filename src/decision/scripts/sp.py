from sys import argv
from json import dumps
from socket import socket

import rospy
from sentry.msg import Position, Positions

__author__ = "YueLin"


class SentryClient:
    def __init__(self, *hp):
        self.socket = socket()
        self.socket.connect(hp)
        rospy.init_node("sentry")
        self.positions = Positions()
        self.publisher = rospy.Publisher(
            "sentry", Positions, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            "position", Position, self.subscribe, None, 1
        )
    
    def close(self) -> None:
        self.socket.close()
    
    def subscribe(self, position: Position) -> None:
        self.socket.sendall(dumps({
            "id": position.id,
            "location": [position.x, position.y, position.yaw]
        }).encode("utf-8"))
        ids, x, y, yaw = [], [], [], []
        positions = eval(self.socket.recv(0x400).decode("utf-8"))
        self.positions.len = len(positions)
        for robot in positions:
            ids.append(robot)
            p = positions[robot]
            x.append(p[0]), y.append(p[1]), yaw.append(p[2])
        self.positions.x = x
        self.positions.y = y
        self.positions.id = ids
        self.positions.yaw = yaw
        self.publisher.publish(self.positions)


if __name__ == "__main__":
    robot = SentryClient(*argv[1:])  # "192.168.1.11", 9999
    rospy.spin(), robot.close()
