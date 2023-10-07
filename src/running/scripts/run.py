import time
from os import path
from sys import argv
from random import choice

import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

RED, BLUE = tuple(range(2))


class Robot:
    def __init__(self, color: str):
        self.dt = 5e-2
        rospy.init_node("ai")
        self.color = eval(color.upper())
        self.publisher = rospy.Publisher(
            "move_base_simple/goal", PoseStamped, queue_size=1
        )
        publisher = rospy.Publisher(
            "initialpose", PoseWithCovarianceStamped, queue_size=1
        )
        rospy.Subscriber("no_enemy_time", Int8, self.callback)
        json = path.join(path.split(path.dirname(__file__))[0], "config")
        with open(path.join(json, "points.json")) as points:
            self.points = eval(points.read())
        time.sleep(5)
        init = PoseWithCovarianceStamped()
        x, y, w = self.points[color.lower()]["xyw"]
        rospy.wait_for_message("no_enemy_time", Int8)
        init.header.frame_id = "map"
        init.pose.pose.position.x = x
        init.pose.pose.position.y = y
        init.pose.pose.orientation.z = w
        init.pose.pose.orientation.w = (1 - w ** 2) ** 0.5
        init.pose.covariance = self.points[color.lower()]["cov"]
        self.points = self.points["points"]
        publisher.publish(init)

    def run(self) -> None:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.points[0][0]
        pose.pose.position.y = self.points[0][1]
        pose.pose.orientation.z = self.points[0][2]
        pose.pose.orientation.w = (1 - self.points[0][2] ** 2) ** 0.5
        self.publisher.publish(pose)
        rospy.spin()
    
    def callback(self, msg: Int8) -> None:
        t = time.time()
        if msg.data >= 2:
            pose = PoseStamped()
            point = choice(self.points)
            pose.header.frame_id = "map"
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.z = point[2]
            pose.pose.orientation.w = (1 - point[2] ** 2) ** 0.5
            self.publisher.publish(pose)
        time.sleep(max(0, self.dt - time.time() + t))


Robot(argv[1]).run()
