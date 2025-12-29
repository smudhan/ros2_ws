#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>

class MissionControl : public rclcpp::Node {
public:
    using Nav2 = nav2_msgs::action::NavigateToPose;
    using Arm = control_msgs::action::FollowJointTrajectory;

    MissionControl() : Node("mission_control_node") {
        nav_client_ = rclcpp_action::create_client<Nav2>(this, "navigate_to_pose");
        arm_client_ = rclcpp_action::create_client<Arm>(this, "/arm_controller/follow_joint_trajectory");
        timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&MissionControl::start_mission, this));
    }

private:
    void start_mission() {
        timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 to be active...");
        
        while (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
            if (!rclcpp::ok()) return;
            RCLCPP_WARN(this->get_logger(), "Nav2 not ready yet, retrying...");
        }

        RCLCPP_INFO(this->get_logger(), "Mission Started: Navigating to RED PICK BOX...");
        // Coordinate 1.1 ensures the robot is safely inside the costmap
        send_nav_goal(1.1, 0.15, 0.0, "PICK"); 
    }

    void send_nav_goal(float x, float y, float yaw, std::string stage) {
        auto goal = Nav2::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = rclcpp::Time(0);
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;

        // Convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.pose.pose.orientation.x = q.x();
        goal.pose.pose.orientation.y = q.y();
        goal.pose.pose.orientation.z = q.z();
        goal.pose.pose.orientation.w = q.w();

        auto options = rclcpp_action::Client<Nav2>::SendGoalOptions();
        options.result_callback = [this, stage](auto) {
            RCLCPP_INFO(this->get_logger(), "Arrived at %s station. Executing Arm...", stage.c_str());
            execute_arm_sequence(stage);
        };
        nav_client_->async_send_goal(goal, options);
    }

    void execute_arm_sequence(std::string stage) {
        auto goal = Arm::Goal();
        goal.trajectory.joint_names = {"open_manipulator_joint1", "open_manipulator_joint2", "open_manipulator_joint3", "open_manipulator_joint4"};
        
        trajectory_msgs::msg::JointTrajectoryPoint p;
        if (stage == "PICK") {
            p.positions = {0.0, -0.6, 0.5, 0.3}; 
        } else {
            p.positions = {0.0, -0.4, 0.2, 0.1}; 
        }
        
        p.time_from_start = rclcpp::Duration::from_seconds(3);
        goal.trajectory.points.push_back(p);

        auto options = rclcpp_action::Client<Arm>::SendGoalOptions();
        options.result_callback = [this, stage](auto) {
            if (stage == "PICK") {
                RCLCPP_INFO(this->get_logger(), "Pick complete. Moving to BLUE PLACE BOX...");
                // x=-1.2 is safely inside the new 30m costmap
                send_nav_goal(-1.2, 0.0, 3.14, "PLACE"); 
            } else {
                RCLCPP_INFO(this->get_logger(), "Mission Accomplished!");
            }
        };
        arm_client_->async_send_goal(goal, options);
    }

    rclcpp_action::Client<Nav2>::SharedPtr nav_client_;
    rclcpp_action::Client<Arm>::SharedPtr arm_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionControl>());
    rclcpp::shutdown();
    return 0;
}