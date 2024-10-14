#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class PatientCartJointController : public rclcpp::Node
{
public:
    PatientCartJointController() : Node("patient_cart_joint_controller")
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
            std::bind(&PatientCartJointController::timer_callback, this));
    }

    void initialize()
    {
        auto client = this->create_client<rcl_interfaces::srv::GetParameters>("/forward_position_controller/get_parameters");

        // Wait for the service to be available
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
        };

        auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
        request->names.push_back("joints");

        // Call the service
        auto result_future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Got response from service get_parameters");
            if (!response->values.empty())
            {
                joint_names_ = response->values[0].string_array_value;
                num_joints = joint_names_.size();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No joints parameter found.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service get_parameters");
        }

        // Initialize joint positions
        joint_positions_.resize(num_joints, 0.0);
        direction_ = 1;
    }

private:
    void timer_callback()
    {
        // Define joint limits
        double prismatic_limit = 0.1; // Max 10 cm
        double revolute_limit = 0.5;  // Approx ±28.6 degrees

        // Update joint positions
        for (int i = 0; i < num_joints; i++)
        {
            double increment = (joint_names_[i].find("insertion") != std::string::npos) ? 0.001 : 0.01;
            joint_positions_[i] += increment * direction_;
        }

        // Reverse direction if limits are reached
        for (int i = 0; i < num_joints; i++)
        {
            double check_limit = (joint_names_[i].find("insertion") != std::string::npos) ? prismatic_limit : revolute_limit;
            if (std::abs(joint_positions_[i]) > check_limit)
            {
                direction_ *= -1;
                break;
            }
        }

        // joint_names for patient cart (44 Joints) -
        // ECM -
        // SUJ_ECM_J0, SUJ_ECM_J1, SUJ_ECM_J2, SUJ_ECM_J3
        // ECM_pitch, ECM_insertion, ECM_roll
        // Total 7 joints
        //
        // PSM1 -
        // SUJ_PSM1_J0, SUJ_PSM1_J1, SUJ_PSM1_J2, SUJ_PSM1_J3, SUJ_PSM1_J4
        // PSM1_outer_yaw, PSM1_pitch, PSM1_outer_insertion, PSM1_outer_roll, PSM1_outer_wrist_pitch, PSM1_outer_wrist_yaw, PSM1_jaw, PSM2_outer_yaw
        // Total 13 joints
        //
        // PSM2 -
        // SUJ_PSM2_J0, SUJ_PSM2_J1, SUJ_PSM2_J2, SUJ_PSM2_J3, SUJ_PSM2_J4
        // PSM2_pitch, PSM2_outer_insertion, PSM2_outer_roll, PSM2_outer_wrist_pitch, PSM2_outer_wrist_yaw, PSM2_jaw
        // Total 11 joints
        //
        // PSM3 -
        // SUJ_PSM3_J0, SUJ_PSM3_J1, SUJ_PSM3_J2, SUJ_PSM3_J3, SUJ_PSM3_J4
        // PSM3_outer_yaw, PSM3_pitch, PSM3_outer_insertion, PSM3_outer_roll, PSM3_outer_wrist_pitch, PSM3_outer_wrist_yaw, PSM3_jaw, ECM_yaw
        // Total 13 joints

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
        RCLCPP_INFO(this->get_logger(), "Publishing Position");
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
    rclcpp::Clock::SharedPtr system_clock_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> joint_positions_;
    std::vector<std::string> joint_names_;
    int direction_;
    int num_joints;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatientCartJointController>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
