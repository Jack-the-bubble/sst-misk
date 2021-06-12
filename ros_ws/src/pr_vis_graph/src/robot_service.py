from __future__ import print_function
import rospy
import matplotlib.pyplot as plt


from planning_utils import sendTargetService, show_animation, \
    prm_planning, scaleToMap

class RobotService():
    def __init__(self, robot, ox, oy) -> None:
        # rospy.init_node(robot_name+'_srv_node')
        self.robot = robot

        while not rospy.is_shutdown():
            if self.robot.goalDriving:
                if robot.new_goal:
                    position = robot.current_odom.pose.pose.position
                    sx, sy = scaleToMap(position.x, position.y)
                    gx, gy = scaleToMap(robot.new_point.x, robot.new_point.y)
                    robot_size = 100.0  # [px]



                    if show_animation:
                        plt.plot(ox, oy, ".k")
                        plt.plot(sx, sy, "^r")
                        plt.plot(gx, gy, "^c")
                        plt.grid(True)
                        plt.axis("equal")

                    robot.rx, robot.ry = prm_planning(sx, sy, gx, gy,
                                                        ox, oy, robot_size)

                    if not robot.rx:
                        print('COULD NOT FIND PATH')
                        robot.new_goal = False
                        robot.goalDriving = False
                        continue
                    # assert rx, 'Cannot found path'

                    if show_animation:
                        plt.plot(robot.rx, robot.ry, "-r")
                        plt.pause(0.00005)
                        plt.show()

                    print("path x, y", robot.rx.reverse(), robot.ry.reverse())
                    robot.new_goal = False

                # goalDriving = True
                if(robot.goalDriving):
                    robot.goalDriving = sendTargetService(robot.rx, 
                                                            robot.ry, robot)
                    if not robot.goalDriving:
                        robot.target_num = 0
                        print("DESTINATION REACHED")