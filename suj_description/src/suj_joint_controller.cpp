#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class SUJJointController : public rclcpp::Node
{
public:
    SUJJointController() : Node("suj_joint_controller")
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
            std::bind(&SUJJointController::timer_callback, this));

        // Initialize joint positions
        num_joints = 19;
        joint_positions_.resize(num_joints, 0.0);
        direction_ = 1;
    }

private:
    void timer_callback()
    {
        // Define joint limits
        double primatic_limit = 0.1; // Max 10 cm
        double revolute_limit = 0.5; // Approx ±28.6 degrees

        // Update joint positions
        for (int i = 0; i < num_joints; i++)
        {
            double increment = ((i == 0) || ((i + 1) % 5 == 0)) ? 0.001 : 0.01;
            joint_positions_[i] += increment * direction_;
        }

        // Reverse direction if limits are reached
        for (int i = 0; i < num_joints; i++)
        {
            double check_limit = ((i == 0) || ((i + 1) % 5 == 0)) ? primatic_limit : revolute_limit;
            if (std::abs(joint_positions_[i]) > check_limit)
            {
                direction_ *= -1;
                break;
            }
        }

        // joint_names for SUJ -
        // ECM -
        // SUJ_ECM_J0, SUJ_ECM_J1, SUJ_ECM_J2, SUJ_ECM_J3
        //
        // PSM1 -
        // SUJ_PSM1_J0, SUJ_PSM1_J1, SUJ_PSM1_J2, SUJ_PSM1_J3, SUJ_PSM1_J4
        //
        // PSM2 -
        // SUJ_PSM2_J0, SUJ_PSM2_J1, SUJ_PSM2_J2, SUJ_PSM2_J3, SUJ_PSM2_J4
        //
        // PSM3 -
        // SUJ_PSM3_J0, SUJ_PSM3_J1, SUJ_PSM3_J2, SUJ_PSM3_J3, SUJ_PSM3_J4

        std_msgs::msg::Float64MultiArray msg;
        std_msgs::msg::MultiArrayDimension dim;
        dim.label = "position";
        dim.size = num_joints;
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
        RCLCPP_INFO(this->get_logger(), "Publishing - Prismatics Position: %3f and Revolute Joint Positions: %3f", joint_positions_[0], joint_positions_[1]);
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
    rclcpp::spin(std::make_shared<SUJJointController>());
    rclcpp::shutdown();
    return 0;
}
