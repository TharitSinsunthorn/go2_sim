#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <chrono>

class JointTrajectoryPublisher : public rclcpp::Node {
public:
    JointTrajectoryPublisher()
        : Node("joint_trajectory_publisher") {

        // Set up the publisher for the joint trajectory topic for all legs
        publisher_FR_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/FRposition_trajectory_controller/joint_trajectory", 10);
        publisher_FL_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/FLposition_trajectory_controller/joint_trajectory", 10);
        publisher_RR_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/RRposition_trajectory_controller/joint_trajectory", 10);
        publisher_RL_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/RLposition_trajectory_controller/joint_trajectory", 10);

        // Set up a timer to periodically publish joint trajectories for all legs
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),  // Adjust the timer interval as needed
            std::bind(&JointTrajectoryPublisher::publish_trajectory, this));
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_FR_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_FL_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_RR_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_RL_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_trajectory() {
        // Create a new joint trajectory message for each leg
        auto create_trajectory = [](const std::vector<std::string>& joint_names, const std::vector<double>& positions) {
            trajectory_msgs::msg::JointTrajectory trajectory;

            trajectory.joint_names = joint_names;

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = positions;
            point.time_from_start = rclcpp::Duration(1, 0);  // Duration of 2 seconds

            trajectory.points.push_back(point);

            return trajectory;
        };

        // Publish trajectory for each leg
        publisher_FR_->publish(create_trajectory({"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"}, {0.0, 0.7, -1.4}));
        publisher_FL_->publish(create_trajectory({"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"}, {0.0, 0.7, -1.4}));
        publisher_RR_->publish(create_trajectory({"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"}, {0.0, 0.7, -1.4}));
        publisher_RL_->publish(create_trajectory({"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"}, {0.0, 0.7, -1.4}));

        RCLCPP_INFO(this->get_logger(), "Published joint trajectories for all legs.");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointTrajectoryPublisher>();
    rclcpp::spin(node);  // Keep the node running to handle callbacks
    rclcpp::shutdown();  // Clean up
    return 0;
}
