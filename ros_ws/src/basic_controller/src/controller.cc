#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "sst_interfaces/SetGoal.h"

typedef sst_interfaces::SetGoal sst_srv;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

bool add(sst_interfaces::SetGoal::Request  &req,
         sst_interfaces::SetGoal::Response &res)
{
  res.response = true;
  return true;
}

class SimpleController: public ros::NodeHandle {
private:
    ros::NodeHandle n_;
    ros::ServiceServer server_;
    ros::Subscriber sub_;
    ros::Publisher cmd_pub_;
    std::shared_ptr<ros::ServiceServer> service_server_;

    nav_msgs::Odometry::ConstPtr current_odom_;

    bool srv_callback(sst_srv::Request &req, sst_srv::Response &res) {
        // calculate orientation


        double target_rotation = calculate_target_rotation(req.pose.position);
        
        rotate_to_target(target_rotation);
        
        
        move_to_target(req.pose.position);




        // rotate to traverse orientation

        // move to given point

        res.response = true;
        return true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        current_odom_ = msg;
        ROS_INFO("Current x pos is %f", current_odom_->pose.pose.position.x);
    }

    double get_rotation(const geometry_msgs::Quaternion orientation) const {
		tf2::Quaternion current_quat;
		tf2::fromMsg(orientation, current_quat);
		double r{}, p{}, y{};
  		tf2::Matrix3x3 m(current_quat);
  		m.getRPY(r, p, y);
		
		return y;
	}

    double calculate_pose_error(geometry_msgs::Point current_pose, geometry_msgs::Point target) {
        double x_diff = target.x - current_pose.x;
        double y_diff = target.y - current_pose.y;

        return std::sqrt(std::pow(x_diff, 2)+std::pow(y_diff, 2));
    }

    double calculate_target_rotation(geometry_msgs::Point target_point) {
        auto current_pose = current_odom_->pose.pose.position;

        double x_diff = target_point.x - current_pose.x;
        double y_diff = target_point.y - current_pose.y;

        double angle = std::atan2(y_diff, x_diff);


        return angle;
    }

    void rotate_to_target(double target_rotation, double error=0.01){
        const double max_speed = 0.5; 
        double modified_target = (target_rotation < 0) ? 3.14 + (3.14 - target_rotation*(-1)) : target_rotation;
        double angle_difference, angle_vel;
        geometry_msgs::Twist target_twist;

        while(true){
            ros::spinOnce();
            double current_rotation = get_rotation(current_odom_->pose.pose.orientation);
            current_rotation = (current_rotation < 0) ? 3.14 + (3.14 - current_rotation*(-1)) : current_rotation;
            
            angle_difference = modified_target - current_rotation;
            // angle_difference_2 = current_rotation - target_rotation;
			if (std::abs(angle_difference) < error){
				target_twist.angular.z = 0.0;
				cmd_pub_.publish(target_twist);
				return;
			}



            angle_vel = std::min<double>(max_speed, std::abs(max_speed*angle_difference))*sgn<double>(angle_difference);
			target_twist.angular.z = angle_vel;
            cmd_pub_.publish(target_twist);
        }
    }

    void move_to_target(geometry_msgs::Point target, double error=0.1) {
        const double max_speed = 0.5; 
        geometry_msgs::Point current_pose;
        geometry_msgs::Twist target_twist;
        double pose_error, lin_vel;

        while(true){
            ros::spinOnce();
            current_pose = current_odom_->pose.pose.position;
            pose_error = calculate_pose_error(current_pose, target);

            if (pose_error < error) {
                target_twist.linear.x = 0.0;
                cmd_pub_.publish(target_twist);
                return;
            }

            lin_vel = std::min<double>(max_speed, pose_error*max_speed);
            target_twist.linear.x = lin_vel;
            cmd_pub_.publish(target_twist);
        }
    }



public:
    SimpleController() {
        n_ = ros::NodeHandle();
        server_ = n_.advertiseService("go_to_pose", 
                                      &SimpleController::srv_callback, this);
        sub_ = n_.subscribe("/robot_0/odom", 10, 
                            &SimpleController::odomCallback, this);

        cmd_pub_ = n_.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 10);
    }
};

// #include "ros/ros.h"
// #include "sst_interfaces/SetGoal.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
//   ros::NodeHandle n;
  SimpleController controller;


//   ros::ServiceServer service = n.advertiseService("add_two_ints", add);
//   ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
