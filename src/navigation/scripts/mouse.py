import rospy
from tf.transformations import euler_from_quaternion as q2e

from geometry_msgs.msg import PoseStamped

from sentry import srv


class Controler:
    def __init__(self, node: str = "ctrl"):
        rospy.init_node(node)
        self.request = srv.planRequest()
        self.client = rospy.ServiceProxy("plan", srv.plan)
        self.subscriber = rospy.Subscriber(
            "move_base_simple/goal", PoseStamped, self.callback, None, 1
        )
        self.client.wait_for_service()
    
    def callback(self, position: PoseStamped):
        q = position.pose.orientation
        self.request.x = round(position.pose.position.x * 100)
        self.request.y = round(position.pose.position.y * 100)
        self.request.yaw = q2e([q.x, q.y, q.z, q.w])[-1]
        self.client.call(self.request)


if __name__ == "__main__":
    ctrl = Controler()
    rospy.spin()
