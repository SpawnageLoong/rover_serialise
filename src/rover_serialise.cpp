#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

class MotorSerialiser : public rclcpp::Node
{
    public:
        MotorSerialiser(char *side)
        : Node("motor_serialiser")
        {
            if (*side == 'R')
            {
                this->topic0_name = "/motors_pwm/motor3";
                this->topic1_name = "/motors_pwm/motor4";
                this->topic2_name = "/motors_pwm/motor5";
            }
            subscription_0 =
                this->create_subscription<std_msgs::msg::UInt8>(
                    topic0_name,
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::motor0_callback, this, std::placeholders::_1)
                );
            subscription_1 =
                this->create_subscription<std_msgs::msg::UInt8>(
                    topic1_name,
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::motor1_callback, this, std::placeholders::_1)
                );
            subscription_2 =
                this->create_subscription<std_msgs::msg::UInt8>(
                    topic2_name,
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::motor2_callback, this, std::placeholders::_1)
                );
        }

    private:
        std::string topic0_name = "/motors_pwm/motor0";
        std::string topic1_name = "/motors_pwm/motor1";
        std::string topic2_name = "/motors_pwm/motor2";
        void motor0_callback(const std_msgs::msg::UInt8 &msg) const
        {
            RCLCPP_INFO(this->get_logger(), "Motor 0 received: '%u'", msg.data);
        }
        void motor1_callback(const std_msgs::msg::UInt8 &msg) const
        {
            RCLCPP_INFO(this->get_logger(), "Motor 1 received: '%u'", msg.data);
        }
        void motor2_callback(const std_msgs::msg::UInt8 &msg) const
        {
            RCLCPP_INFO(this->get_logger(), "Motor 2 received: '%u'", msg.data);
        }
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_0;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_1;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_2;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    printf("RCLCPP Initialised\n");
    auto node = std::make_shared<MotorSerialiser>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    printf("RCLCPP Shutdown\n");
    return 0;
}
