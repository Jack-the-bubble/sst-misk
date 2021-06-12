import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from sst_interfaces.srv import SetGoal

class RobotObject:
    def __init__(self, topic_prefix) -> None:
        self.odom_callback = rospy.Subscriber(topic_prefix + '/odom', Odometry,
                                              self.odom_callback)
        self.new_goal_service = rospy.Service(topic_prefix+'/goal', SetGoal,
                                              self.new_goal_callback)
        self.current_odom = Odometry()
        self.new_goal = False  # used only to find a path (once)
        self.goalDriving = False  # used for driving to goal (continuously)
        self.target_num = 0
        self.rx = []  # to store path x coordinates
        self.ry = []  # to store path y coordinates
        self.service_name = topic_prefix + '/go_to_pose'
        self.new_point = Point()
        rospy.wait_for_message(topic_prefix+'/odom', Odometry)

    def odom_callback(self, odom):
        self.current_odom = odom

    def new_goal_callback(self, req):
        self.target_num = 0
        self.new_goal = True
        self.goalDriving = True
        self.new_point = Point(req.pose.position.x, req.pose.position.y, 0)
        return True

