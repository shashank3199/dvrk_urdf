#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

class PSMJointController : public rclcpp::Node
{
public:
    PSMJointController() : Node("psm_joint_controller")
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
            std::bind(&PSMJointController::timer_callback, this));
    }

    void initialize()
    {
        // Get the joint names from the parameter server
        auto client = this->create_client<rcl_interfaces::srv::GetParameters>("/forward_position_controller/get_parameters");

        // Wait for the service to be available
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for parameter service to become available...");
        }

        // Create the request
        auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
        request->names.push_back("joints");

        // Call the service
        auto result_future = client->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Got response from service get_parameters");
            if (!response->values.empty())
            {
                // Get the joint names and number of joints
                auto joint_names = response->values[0].string_array_value;
                num_joints = joint_names.size();
                // Find the insertion joint index
                insertion_joint_index = 0;
                for (const auto &joint : joint_names)
                {
                    RCLCPP_INFO(this->get_logger(), "- %s", joint.c_str());
                    if (joint.find("insertion") != std::string::npos)
                    {
                        break;
                    }
                    insertion_joint_index++;
                }
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
        double primatic_limit = 0.1; // Max 10 cm
        double revolute_limit = 0.5; // Approx ±28.6 degrees

        // Update joint positions
        for (int i = 0; i < num_joints; i++)
        {
            double increment = (i == insertion_joint_index) ? 0.001 : 0.01;
            joint_positions_[i] += increment * direction_;
        }

        // Reverse direction if limits are reached
        for (int i = 0; i < num_joints; i++)
        {
            double check_limit = (i == insertion_joint_index) ? primatic_limit : revolute_limit;
            if (std::abs(joint_positions_[i]) > check_limit)
            {
                direction_ *= -1;
                break;
            }
        }

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
    int insertion_joint_index;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PSMJointController>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
