#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class DaVinciCoreJointController : public rclcpp::Node
{
public:
    DaVinciCoreJointController() : Node("davinci_core_joint_controller")
    {
        // Define the number of joints
        num_joints = 14;

        // Create a publisher to send joint position commands
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

        // Create a publisher for the clock
        clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Create a system clock
        system_clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

        // Create a timer to periodically publish commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DaVinciCoreJointController::timer_callback, this));

        // Initialize joint positions: [yaw, pitch, insertion, roll]
        joint_positions_.resize(num_joints, 0.0);
        direction_ = 1;
    }

private:
    void timer_callback()
    {
        // Define joint limits
        // 6 and 13
        double primatic_limit = 0.1; // Max 10 cm
        double revolute_limit = 0.5; // Approx ±28.6 degrees

        // Update joint positions
        for (int i = 0; i < num_joints; i++)
        {
            double increment = ((i == 6) || (i == 13)) ? 0.001 : 0.01;
            joint_positions_[i] += increment * direction_;
        }

        // Reverse direction if limits are reached
        for (int i = 0; i < num_joints; i++)
        {
            double check_limit = ((i == 6) || (i == 13)) ? primatic_limit : revolute_limit;
            if (std::abs(joint_positions_[i]) > check_limit)
            {
                direction_ *= -1;
                break;
            }
        }

        // joint_names for DaVinci Core Arms
        // left_arm_outer_yaw_joint, left_arm_outer_pitch_base_joint, left_arm_outer_pitch_front_joint, left_arm_outer_pitch_bottom_joint,
        // left_arm_outer_pitch_top_joint, left_arm_outer_insertion_joint, left_arm_tool_insertion_joint

        // right_arm_outer_yaw_joint, right_arm_outer_pitch_base_joint, right_arm_outer_pitch_front_joint, right_arm_outer_pitch_bottom_joint
        // right_arm_outer_pitch_top_joint, right_arm_outer_insertion_joint, right_arm_tool_insertion_joint

        std_msgs::msg::Float64MultiArray msg;
        std_msgs::msg::MultiArrayDimension dim;
        dim.label = "position";
        dim.size = 14;
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
        for (int i = 0; i < num_joints; i++)
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
    int num_joints;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DaVinciCoreJointController>());
    rclcpp::shutdown();
    return 0;
}
