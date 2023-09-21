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
#include "rover_interfaces/msg/pwm_array.hpp"

class MotorSerialiser : public rclcpp::Node
{
    public:
        MotorSerialiser()
        : Node("motor_serialiser")
        {
            this->declare_parameter("pwm_array_topic_name", "/motors_pwm");

            subscription_ =
                this->create_subscription<rover_interfaces::msg::PwmArray>(
                    this->get_parameter("pwm_array_topic_name").as_string(),
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::pwm_callback, this, std::placeholders::_1)
                );
            timer =
                this->create_wall_timer(
                    std::chrono::milliseconds(1000),
                    std::bind(&MotorSerialiser::timer_callback, this)
                );
        }

    private:
        int16_t motor0_pwm = 0;
        int16_t motor1_pwm = 0;
        int16_t motor2_pwm = 0;
        int16_t motor3_pwm = 0;
        int16_t motor4_pwm = 0;
        int16_t motor5_pwm = 0;

        void pwm_callback(const rover_interfaces::msg::PwmArray &msg) const
        {
            RCLCPP_INFO(this->get_logger(), "Array PWM0 received: '%u'", msg.pwm0);
        }
        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "Timer callback");
            // TODO: Write the PWM values to the two byte arrays and send over serial

        }

        rclcpp::Subscription<rover_interfaces::msg::PwmArray>::SharedPtr subscription_;
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

void timer(std::function<void(void)> func, unsigned int interval)
{
  std::thread([func, interval]()
  { 
    while (true)
    { 
      auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval);
      func();
      std::this_thread::sleep_until(x);
    }
  }).detach();
}

void sendPwmToSerial(SerialPort serialPortL, SerialPort serialPortR, MotorSerialiser motorSerialiser)
{
    uint32_t bytesL = 0x00000000;
    uint32_t bytesR = 0x00000000;
    uint8_t* bytesArrayL = reinterpret_cast<uint8_t*>(&bytes);
    uint8_t* bytesArrayR = reinterpret_cast<uint8_t*>(&bytes);
    bytesArrayL[0] = MotorSerialiser::motor0_pwm;
    bytesArrayL[1] = MotorSerialiser::motor1_pwm;
    bytesArrayL[2] = MotorSerialiser::motor2_pwm;
    bytesArrayL[3] = 0x00;
    bytesArrayR[0] = MotorSerialiser::motor3_pwm;
    bytesArrayR[1] = MotorSerialiser::motor4_pwm;
    bytesArrayR[2] = MotorSerialiser::motor5_pwm;
    bytesArrayR[3] = 0x00;
    serialPortL.WriteData(reinterpret_cast<const char*>(bytesArrayL), 4);
    serialPortR.WriteData(reinterpret_cast<const char*>(bytesArrayR), 4);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorSerialiser>();
    std::cout << "MotorSerialiser started" << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
