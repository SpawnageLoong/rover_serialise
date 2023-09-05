#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

class MotorSerialiser : public rclcpp::Node
{
    public:
        MotorSerialiser()
        : Node("motor_serialiser")
        {
            subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
                "/motors_pwm/" + motor_num0, 10, std::bind(&MotorSerialiser::motor0_callback, this, std::placeholders::_1)
            );
        }

    private:
        char motor_num0;
        char motor_num1;
        char motor_num2;
        void motor0_callback(const std_msgs::msg::UInt8 &msg) const
        {
            // Callback code goes here
        }
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorSerialiser>());
    rclcpp::shutdown();
    return 0;
}
