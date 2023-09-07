#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

class MotorSerialiser : public rclcpp::Node
{
    public:
        MotorSerialiser()
        : Node("motor_serialiser")
        {
            subscription_ =
                this->create_subscription<std_msgs::msg::UInt8>(
                    "/motors_pwm/motor0",
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::motor0_callback, this, std::placeholders::_1)
                );
        }

    private:
        void motor0_callback(const std_msgs::msg::UInt8 &msg) const
        {
            // Callback code goes here
            RCLCPP_INFO(this->get_logger(), "I heard: '%u'", msg.data);
        }
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    printf("RCLCPP Initialised\n");
    auto node = std::make_shared<MotorSerialiser>();
    printf("Node Created\n");
    rclcpp::spin(node);
    printf("Node Spun\n");
    rclcpp::shutdown();
    printf("RCLCPP Shutdown\n");
    return 0;
}
