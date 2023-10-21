/**
 * @file rover_serialise.cpp
 * @author Richard Loong (richardloongcj@gmail.com)
 * @brief A ROS Node to serialise the target PWM values of the motors and send them over serial
 * @version 1.0
 * @date 2023-09-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <chrono>
#include <iostream>
#include <thread>
#include <bitset>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "rover_interfaces/msg/pwm_array.hpp"
#include "rover_interfaces/msg/camera_cmd.hpp"

class MotorSerialiser : public rclcpp::Node
{
    public:
        MotorSerialiser()
        : Node("motor_serialiser")
        {
            this->declare_parameter("pwm_array_topic_name", "/motors_pwm");
            this->declare_parameter("camera_cmd_topic_name", "/camera_cmd");
            this->declare_parameter("max_servo_angle", 120);

            cam_subscription = 
                this->create_subscription<rover_interfaces::msg::CameraCmd>(
                    this->get_parameter("camera_cmd_topic_name").as_string(),
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::camera_callback, this, std::placeholders::_1)
                );
            subscription_ =
                this->create_subscription<rover_interfaces::msg::PwmArray>(
                    this->get_parameter("pwm_array_topic_name").as_string(),
                    rclcpp::SensorDataQoS(),
                    std::bind(&MotorSerialiser::pwm_callback, this, std::placeholders::_1)
                );
        }

        int16_t getCameraPan() const
        {
            return camera_pan;
        }
        int16_t getCameraTilt() const
        {
            return camera_tilt;
        }
        int16_t getPanPwm() const
        {
            return pan_pwm;
        }
        int16_t getTiltPwm() const
        {
            return tilt_pwm;
        }

        int16_t getMotor0Pwm() const
        {
            return motor0_pwm;
        }
        int16_t getMotor1Pwm() const
        {
            return motor1_pwm;
        }
        int16_t getMotor2Pwm() const
        {
            return motor2_pwm;
        }
        int16_t getMotor3Pwm() const
        {
            return motor3_pwm;
        }
        int16_t getMotor4Pwm() const
        {
            return motor4_pwm;
        }
        int16_t getMotor5Pwm() const
        {
            return motor5_pwm;
        }

    private:
        int16_t motor0_pwm = 0;
        int16_t motor1_pwm = 0;
        int16_t motor2_pwm = 0;
        int16_t motor3_pwm = 0;
        int16_t motor4_pwm = 0;
        int16_t motor5_pwm = 0;

        int16_t camera_pan = 0;
        int16_t camera_tilt = 0;
        uint8_t pan_pwm = 127;
        uint8_t tilt_pwm = 127;

        void setMotor0Pwm(int16_t pwm)
        {
            motor0_pwm = pwm;
        }
        void setMotor1Pwm(int16_t pwm)
        {
            motor1_pwm = pwm;
        }
        void setMotor2Pwm(int16_t pwm)
        {
            motor2_pwm = pwm;
        }
        void setMotor3Pwm(int16_t pwm)
        {
            motor3_pwm = pwm;
        }
        void setMotor4Pwm(int16_t pwm)
        {
            motor4_pwm = pwm;
        }
        void setMotor5Pwm(int16_t pwm)
        {
            motor5_pwm = pwm;
        }

        void setCameraPan(int16_t pan)
        {
            camera_pan = pan;
        }
        void setCameraTilt(int16_t tilt)
        {
            camera_tilt = tilt;
        }
        void calcPanPwm(int16_t pan)
        {
            int16_t max_angle = this->get_parameter("max_servo_angle").as_int();
            double temp_pan = (double)pan/(double)max_angle * 127 + 127;
            pan_pwm = floor(temp_pan);
        }
        void calcTiltPwm(int16_t tilt)
        {
            int16_t max_angle = this->get_parameter("max_servo_angle").as_int();
            double temp_tilt = (double)tilt/(double)max_angle * 127 + 127;
            tilt_pwm = floor(temp_tilt);
        }

        void pwm_callback(const rover_interfaces::msg::PwmArray &msg)
        {
            RCLCPP_INFO(this->get_logger(), "PWM Callback, Array PWM0 received: '%i'", msg.pwm0);
            setMotor0Pwm(msg.pwm0);
            setMotor1Pwm(msg.pwm1);
            setMotor2Pwm(msg.pwm2);
            setMotor3Pwm(msg.pwm3);
            setMotor4Pwm(msg.pwm4);
            setMotor5Pwm(msg.pwm5);
        }

        void camera_callback(const rover_interfaces::msg::CameraCmd &msg)
        {
            RCLCPP_INFO(this->get_logger(), "Camera Cmd received: '%u', '%u'", msg.pan, msg.tilt);
            setCameraPan(msg.pan);
            setCameraTilt(msg.tilt);
            calcPanPwm(msg.pan);
            calcTiltPwm(msg.tilt);
        }

        rclcpp::Subscription<rover_interfaces::msg::PwmArray>::SharedPtr subscription_;
        rclcpp::Subscription<rover_interfaces::msg::CameraCmd>::SharedPtr cam_subscription;
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

            /*
            for (int i=0; i<length; i++)
            {
                std::cout << "hex " << i << ": " << std::hex << data[i] << std::endl;
            }
            */

            //std::cout << "Data Written: " << bytesWritten << " bytes" << data << std::endl; 
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
      func();
      std::this_thread::sleep_for(std::chrono::milliseconds(interval));
    }
  }).detach();
}

void writePwmArray(std::shared_ptr<MotorSerialiser> motorSerialiser, uint32_t &bytesL, uint32_t &bytesR)
{
    int16_t motor0_pwm = motorSerialiser->getMotor0Pwm();
    int16_t motor1_pwm = motorSerialiser->getMotor1Pwm();
    int16_t motor2_pwm = motorSerialiser->getMotor2Pwm();
    int16_t motor3_pwm = motorSerialiser->getMotor3Pwm();
    int16_t motor4_pwm = motorSerialiser->getMotor4Pwm();
    int16_t motor5_pwm = motorSerialiser->getMotor5Pwm();
    uint8_t bytesArrayL[4];
    uint8_t bytesArrayR[4];
    bytesArrayL[0] = abs(motor0_pwm);
    bytesArrayL[1] = abs(motor1_pwm);
    bytesArrayL[2] = abs(motor2_pwm);
    bytesArrayL[3] = ((uint8_t)(motor0_pwm < 0) << 5)  | ((uint8_t)(motor1_pwm < 0) << 4)  | ((uint8_t)(motor2_pwm < 0) << 3) |
                     ((uint8_t)(motor0_pwm == 0) << 2) | ((uint8_t)(motor1_pwm == 0) << 1) | ((uint8_t)(motor2_pwm == 0));
    bytesArrayR[0] = abs(motor3_pwm);
    bytesArrayR[1] = abs(motor4_pwm);
    bytesArrayR[2] = abs(motor5_pwm);
    bytesArrayR[3] = ((uint8_t)(motor3_pwm < 0) << 5)  | ((uint8_t)(motor4_pwm < 0) << 4)  | ((uint8_t)(motor5_pwm < 0) << 3) |
                     ((uint8_t)(motor3_pwm == 0) << 2) | ((uint8_t)(motor4_pwm == 0) << 1) | ((uint8_t)(motor5_pwm == 0));

    //std::cout << "Right 0: " << std::bitset<8>(bytesArrayR[0]) << std::endl;
    //std::cout << "Right 1: " << std::bitset<8>(bytesArrayR[1]) << std::endl;
    //std::cout << "Right 2: " << std::bitset<8>(bytesArrayR[2]) << std::endl;
    //std::cout << "Right 3: " << std::bitset<8>(bytesArrayR[3]) << std::endl;
    bytesL = (bytesArrayL[0] << 24) | (bytesArrayL[1] << 16) | (bytesArrayL[2] << 8) | bytesArrayL[3];
    bytesR = (bytesArrayR[0] << 24) | (bytesArrayR[1] << 16) | (bytesArrayR[2] << 8) | bytesArrayR[3];
    //std::cout << "Right Full: " << std::bitset<32>(bytesR) << std::endl;
}

int main(int argc, char* argv[])
{
    uint32_t bytesL;
    uint32_t bytesR;
    uint8_t pan_pwm;
    uint8_t tilt_pwm;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorSerialiser>();
    std::cout << "MotorSerialiser started" << std::endl;

    SerialPort serialPortL("/dev/ttyESPleft", B115200);
    std::cout << "Serial Port L opened" << std::endl;
    SerialPort serialPortR("/dev/ttyESPright", B115200);
    std::cout << "Serial Port R opened" << std::endl;
    
    std::thread timerThread([&]()
    {
        while (true)
        {
            pan_pwm = node->getPanPwm();
            tilt_pwm = node->getTiltPwm();
            writePwmArray(node, bytesL, bytesR);
            char dataL[6] = {0x0A, 0x24, (char)(bytesL >> 24), (char)(bytesL >> 16), (char)(bytesL >> 8), (char)bytesL};
            char dataR[6] = {0x0A, 0x24, (char)(bytesR >> 24), (char)(bytesR >> 16), (char)(bytesR >> 8), (char)bytesR};
            //std::cout << "Left: " << std::bitset<32>(bytesL) << std::endl;
            //std::cout << "Right: " << std::bitset<32>(bytesR) << std::endl;
            serialPortL.WriteData((char*)&dataL, 6);
            serialPortR.WriteData((char*)&dataR, 6);
            //std::cout << "right hex: " << std::hex << dataR[0] << dataR[1] << dataR[2] << dataR[3] << dataR[4] << dataR[5] << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
    timerThread.detach();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
