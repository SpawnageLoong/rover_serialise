#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/uint8.msg"

class MotorSerialiser : public rclcpp::Node
{
    public:
        MotorSerialiser()
        : Node("motor_serialiser")
        {
            subscription0_ = this->create_subscription<std_msgs::msg::UInt8>(
                "/motors_pwm/" + this.motor_num0, 10, std::bind(&MotorSerialiser::motor0_callback, this, _1)
            );
            subscription1_ = this->create_subscription<std_msgs::msg::UInt8>(
                "/motors_pwm/" + this.motor_num1, 10, std::bind(&MotorSerialiser::motor1_callback, this, _1)
            );
            subscription2_ = this->create_subscription<std_msgs::msg::UInt8>(
                "/motors_pwm/" + this.motor_num2, 10, std::bind(&MotorSerialiser::motor2_callback, this, _1)
            );
        }

    private:
        char motor_num0, motor_num1, motor_num2;
        uint8 pwm0, pwm1, pwm2;
        void motor0_callback(const std_msgs::msg::UInt8 &msg) const
        {
            // Callback code goes here
        }
        void motor1_callback(const std_msgs::msg::UInt8 &msg) const
        {
            // Callback code goes here
        }
        void motor2_callback(const std_msgs::msg::UInt8 &msg) const
        {
            // Callback code goes here
        }
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription0_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription1_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription2_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorSerialiser>())
    rclcpp::shutdown();
    return 0;
}
