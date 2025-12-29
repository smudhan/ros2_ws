#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>

class TeleopManager : public rclcpp::Node {
public:
    TeleopManager() : Node("teleop_manager") {
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);
        gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/gripper_controller/joint_trajectory", 10);
        
        joint_positions_ = {0.0, 0.0, 0.0, 0.0};
        joint_names_ = {"open_manipulator_joint1", "open_manipulator_joint2", "open_manipulator_joint3", "open_manipulator_joint4"};
        
        // UPDATED: Verified joint name for your OpenXManipulator
        gripper_names_ = {"open_manipulator_gripper_left_joint"};
        gripper_opened_ = true;
    }

    void move_base(double linear, double angular) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear;
        msg.angular.z = angular;
        twist_pub_->publish(msg);
    }

    void stop_robot() {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;

        // Publish multiple times to ensure the controller registers the zero-state
        for(int i = 0; i < 5; ++i) {
            twist_pub_->publish(msg);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
        
        RCLCPP_INFO(this->get_logger(), "EMERGENCY STOP: Velocity Zeroed");
    }

    void toggle_gripper() {
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.joint_names = gripper_names_;
        trajectory_msgs::msg::JointTrajectoryPoint p;
        
        // UPDATED: Using your verified limits: 0.019 (Open) and -0.010 (Closed)
        double target_pos = gripper_opened_ ? -0.010 : 0.019;
        p.positions = {target_pos};
        p.time_from_start = rclcpp::Duration::from_seconds(0.5);
        msg.points.push_back(p);
        
        gripper_pub_->publish(msg);
        gripper_opened_ = !gripper_opened_;
        
        std::cout << "Gripper sent to: " << (gripper_opened_ ? "OPEN (0.019)" : "CLOSED (-0.010)") << std::endl;
    }

    void move_arm(int joint_idx, double delta) {
        joint_positions_[joint_idx] += delta;
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.joint_names = joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions = joint_positions_;
        p.time_from_start = rclcpp::Duration::from_seconds(0.1);
        msg.points.push_back(p);
        arm_pub_->publish(msg);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
    std::vector<double> joint_positions_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> gripper_names_;
    bool gripper_opened_;
};

int getch() {
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopManager>();
    
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "W/A/S/D : Base (Reduced Speed)" << std::endl;
    std::cout << "1 - 8   : Arm Joints +/-" << std::endl;
    std::cout << "0       : Toggle Gripper (Open/Close)" << std::endl;
    std::cout << "x       : STOP ROBOT" << std::endl;
    std::cout << "q       : Quit" << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    while (rclcpp::ok()) {
        int c = getch();
        if (c == 'w') node->move_base(0.15, 0.0);
        else if (c == 's') node->move_base(-0.15, 0.0);
        else if (c == 'a') node->move_base(0.0, 0.4);
        else if (c == 'd') node->move_base(0.0, -0.4);
        else if (c == 'x') node->stop_robot();
        else if (c == '0') node->toggle_gripper();
        else if (c >= '1' && c <= '8') {
            int idx = (c - '1') / 2;
            double delta = ((c - '1') % 2 == 0) ? 0.05 : -0.05;
            node->move_arm(idx, delta);
        }
        else if (c == 'q') break;
        
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}