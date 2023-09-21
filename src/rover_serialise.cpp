/**
 * @file rover_serialise.cpp
 * @author Richard Loong (richardloongcj@gmail.com)
 * @brief A ROS Node to serialise the target PWM values of the motors and send them over serial
 * @version 0.1
 * @date 2023-09-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <chrono>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

class MotorSerialiser : public rclcpp::Node
{
    public:
        MotorSerialiser()
        : Node("motor_serialiser")
        {
            this->declare_parameter("topic0_name", "/motors_pwm/motor0");
            this->declare_parameter("topic1_name", "/motors_pwm/motor1");
            this->declare_parameter("topic2_name", "/motors_pwm/motor2");
            this->declare_parameter("topic3_name", "/motors_pwm/motor3");
            this->declare_parameter("topic4_name", "/motors_pwm/motor4");
            this->declare_parameter("topic5_name", "/motors_pwm/motor5");

            subscription_0 =
                this->create_subscription<std_msgs::msg::UInt8>(
                    this->get_parameter("topic0_name").as_string(),
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::motor0_callback, this, std::placeholders::_1)
                );
            subscription_1 =
                this->create_subscription<std_msgs::msg::UInt8>(
                    this->get_parameter("topic1_name").as_string(),
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::motor1_callback, this, std::placeholders::_1)
                );
            subscription_2 =
                this->create_subscription<std_msgs::msg::UInt8>(
                    this->get_parameter("topic2_name").as_string(),
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::motor2_callback, this, std::placeholders::_1)
                );
            subscription_3 =
                this->create_subscription<std_msgs::msg::UInt8>(
                    this->get_parameter("topic3_name").as_string(),
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::motor3_callback, this, std::placeholders::_1)
                );
            subscription_4 =
                this->create_subscription<std_msgs::msg::UInt8>(
                    this->get_parameter("topic4_name").as_string(),
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::motor4_callback, this, std::placeholders::_1)
                );
            subscription_5 =
                this->create_subscription<std_msgs::msg::UInt8>(
                    this->get_parameter("topic5_name").as_string(),
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
        uint8_t motor0_pwm = 0;
        uint8_t motor1_pwm = 0;
        uint8_t motor2_pwm = 0;
        uint8_t motor3_pwm = 0;
        uint8_t motor4_pwm = 0;
        uint8_t motor5_pwm = 0;

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

class SerialPort {
    public:
        SerialPort(const char* portName, speed_t baudRate)
        {
            fd_ = open(portName, O_RDWR);
            if (fd_ == -1)
            {
                std::cout << "Error opening serial port" << std::endl;
                return;
            }

            struct termios tty;
            if (tcgetattr(fd_, &tty) != 0)
            {
                std::cout << "Error getting serial port attributes" << std::endl;
                return;
            }

            cfsetospeed(&tty, baudRate);
            cfsetispeed(&tty, baudRate);

            tty.c_cflag &= ~PARENB;
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;
            tty.c_cflag &= ~CREAD;
            tty.c_cflag |= CLOCAL;

            if (tcsetattr(fd_, TCSANOW, &tty) != 0)
            {
                std::cout << "Error setting serial port attributes" << std::endl;
                return;
            }
        }

        ~SerialPort()
        {
            if (isOpen())
            {
                close(fd_);
            }
        }

        bool isOpen()
        {
            return fd_ != -1;
        }

        bool WriteData(const char* data, size_t length)
        {
            if (!isOpen())
            {
                return false;
            }
            ssize_t bytesWritten = write(fd_, data, length);
            return bytesWritten == static_cast<ssize_t>(length);
        }

        size_t readData(char* buffer, size_t maxLength)
        {
            if (!isOpen())
            {
                return -1;
            }
            ssize_t bytesRead = read(fd_, buffer, maxLength);
            return bytesRead;
        }

    private:
        int fd_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorSerialiser>();
    std::cout << "MotorSerialiser started" << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
