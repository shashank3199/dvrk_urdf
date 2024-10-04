#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class ECMJointController : public rclcpp::Node
{
public:
    ECMJointController() : Node("ecm_joint_controller")
    {
        // Create a publisher to send joint position commands
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

        // Create a publisher for the clock
        clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Create a system clock
        system_clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

        // Create a timer to periodically publish commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ECMJointController::timer_callback, this));

        // Initialize joint positions: [yaw, pitch, insertion, roll]
        joint_positions_.resize(4, 0.0);
        direction_ = 1;
    }

private:
    void timer_callback()
    {
        // Define joint limits
        double yaw_limit = 1.5;        // Approx ±85 degrees
        double pitch_limit = 1.0;      // Approx ±57 degrees
        double insertion_limit = 0.23; // Max insertion length
        double roll_limit = 3.14;      // Approx ±180 degrees

        // Update joint positions
        joint_positions_[0] += 0.001 * direction_;  // ecm_yaw
        joint_positions_[1] += 0.0005 * direction_; // ecm_pitch
        joint_positions_[2] += 0.0001 * direction_; // ecm_insertion
        joint_positions_[3] += 0.002 * direction_;  // ecm_roll

        // Reverse direction if limits are reached
        if (std::abs(joint_positions_[0]) > yaw_limit ||
            std::abs(joint_positions_[1]) > pitch_limit ||
            joint_positions_[2] > insertion_limit || joint_positions_[2] < 0 ||
            std::abs(joint_positions_[3]) > roll_limit)
        {
            direction_ *= -1;
        }

        // joint_names for ecm_base = {"ecm_yaw", "ecm_pitch", "ecm_insertion", "ecm_roll"};
        // joint_names for ecm = {"ecm_outer_yaw", "ecm_outer_pitch", "ecm_insertion", "ecm_outer_roll"};
        std_msgs::msg::Float64MultiArray msg;
        std_msgs::msg::MultiArrayDimension dim;
        dim.label = "position";
        dim.size = 4;
        dim.stride = 1;

        msg.layout.dim.push_back(dim);
        msg.data = joint_positions_;

        // Publish the message
        publisher_->publish(msg);

        // Publish the clock message
        auto current_time = system_clock_->now();
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = current_time;
        clock_publisher_->publish(clock_msg);

        // Log the joint positions
        RCLCPP_INFO(this->get_logger(), "Publishing joint positions: [%.2f, %.2f, %.2f, %.2f]",
                    joint_positions_[0], joint_positions_[1],
                    joint_positions_[2], joint_positions_[3]);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
    rclcpp::Clock::SharedPtr system_clock_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> joint_positions_;
    int direction_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ECMJointController>());
    rclcpp::shutdown();
    return 0;
}
