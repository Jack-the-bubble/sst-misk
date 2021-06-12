import rospy
from nav_msgs.msg import Odometry

class RobotObject:
    def __init__(self, topic_prefix) -> None:
        self.odom_callback = rospy.Subscriber(topic_prefix + '/odom', Odometry,
                                              self.odom_callback)
        self.current_odom = Odometry()
        rospy.wait_for_message(topic_prefix+'/odom', Odometry)

    def odom_callback(self, odom):
        self.current_odom = odom