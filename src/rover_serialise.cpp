#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

class MotorSerialiser : public rclcpp::Node
{
    public:
        MotorSerialiser(char *side)
        : Node("motor_serialiser")
        {
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
            subscription_3 =
                this->create_subscription<std_msgs::msg::UInt8>(
                    topic3_name,
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::motor3_callback, this, std::placeholders::_1)
                );
            subscription_4 =
                this->create_subscription<std_msgs::msg::UInt8>(
                    topic4_name,
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::motor4_callback, this, std::placeholders::_1)
                );
            subscription_5 =
                this->create_subscription<std_msgs::msg::UInt8>(
                    topic5_name,
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::motor5_callback, this, std::placeholders::_1)
                );
            timer =
                this->create_wall_timer(
                    std::chrono::milliseconds(1000),
                    std::bind(&MotorSerialiser::timer_callback, this)
                );
        }

    private:
        std::string topic0_name = "/motors_pwm/motor0";
        std::string topic1_name = "/motors_pwm/motor1";
        std::string topic2_name = "/motors_pwm/motor2";
        std::string topic3_name = "/motors_pwm/motor3";
        std::string topic4_name = "/motors_pwm/motor4";
        std::string topic5_name = "/motors_pwm/motor5";

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
        void motor3_callback(const std_msgs::msg::UInt8 &msg) const
        {
            RCLCPP_INFO(this->get_logger(), "Motor 3 received: '%u'", msg.data);
        }
        void motor4_callback(const std_msgs::msg::UInt8 &msg) const
        {
            RCLCPP_INFO(this->get_logger(), "Motor 4 received: '%u'", msg.data);
        }
        void motor5_callback(const std_msgs::msg::UInt8 &msg) const
        {
            RCLCPP_INFO(this->get_logger(), "Motor 5 received: '%u'", msg.data);
        }
        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "Timer callback");
            // TODO: Write the PWM values to the two byte arrays and send over serial
        }

        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_0;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_1;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_2;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_3;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_4;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_5;
        rclcpp::TimerBase::SharedPtr timer;
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
