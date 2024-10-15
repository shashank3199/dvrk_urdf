#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class MTMSJointController : public rclcpp::Node
{
public:
    MTMSJointController() : Node("mtms_joint_controller")
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
            std::bind(&MTMSJointController::timer_callback, this));

        // Initialize Joint Positions
        joint_positions_.resize(14, 0.0);
        direction_ = 1;
    }

private:
    void timer_callback()
    {
        // Define joint limits
        double outer_yaw_limit = 1.5;       // Approx ±85 degrees
        double shoulder_pitch_limit = 0.5;  // Approx ±30 degrees
        double elbow_pitch_limit = 1.0;     // Approx ±57 degrees
        double wrist_platform_limit = 3.14; // Approx ±180 degrees
        double wrist_pitch_limit = 1.0;     // Approx ±57 degrees
        double wrist_yaw_limit = 1.5;       // Approx ±85 degrees
        double wrist_roll_limit = 1.0;      // Approx ±57 degrees

        // Update Joint Positions
        for (int i = 0; i < 14; i++)
        {
            joint_positions_[i] += 0.001 * direction_;
        }

        // Reverse direction if limits are reached
        double limits[14] = {outer_yaw_limit, shoulder_pitch_limit, elbow_pitch_limit, wrist_platform_limit, wrist_pitch_limit, wrist_yaw_limit, wrist_roll_limit, outer_yaw_limit, shoulder_pitch_limit, elbow_pitch_limit, wrist_platform_limit, wrist_pitch_limit, wrist_yaw_limit, wrist_roll_limit};
        for (int i = 0; i < 7; i++)
        {
            if (std::abs(joint_positions_[i]) > limits[i] || std::abs(joint_positions_[i + 8]) > limits[i])
            {
                direction_ *= -1;
                break;
            }
        }

        // Joint Names for MTM-S
        // MTML_outer_yaw, MTML_shoulder_pitch, MTML_elbow_pitch, MTML_wrist_platform,
        // MTML_wrist_pitch, MTML_wrist_yaw, MTML_wrist_roll, MTMR_outer_yaw

        // MTMR_outer_yaw, MTMR_shoulder_pitch, MTMR_elbow_pitch, MTMR_wrist_platform,
        // MTMR_wrist_pitch, MTMR_wrist_yaw, MTMR_wrist_roll
        std_msgs::msg::Float64MultiArray msg;
        std_msgs::msg::MultiArrayDimension dim;
        dim.label = "position";
        dim.size = 8;
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
        RCLCPP_INFO(this->get_logger(), "Publishing joint positions -");
        for (int i = 0; i < 14; i++)
        {
            RCLCPP_INFO(this->get_logger(), "Joint %d: %f", i, joint_positions_[i]);
        }
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
    rclcpp::spin(std::make_shared<MTMSJointController>());
    rclcpp::shutdown();
    return 0;
}
