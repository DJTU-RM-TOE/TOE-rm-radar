/*
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iomanip>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "radar_serial/crc.hpp"

struct MyStruct
{
    uint8_t SOF = (uint8_t)0xA5;
    uint16_t data_length = 0x000A;
    uint8_t seq;
    uint8_t CRC8;

    uint16_t cmd_id = 0x0305U;

    uint16_t target_robot_ID = 0x0001U;
    float target_position_x = 1.0;
    float target_position_y = 1.0;

    uint16_t CRC16;
} __attribute__((packed));


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("serial_node");
    // 打开串口文件
    int serial_port = open("/dev/ttyUSB0", O_RDWR);
    if (serial_port < 0)
    {
        std::cout << "无法打开串口" << std::endl;
        return 1;
    }

    // 配置串口
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_port, &tty) != 0)
    {
        std::cout << "无法获取串口属性" << std::endl;
        return 1;
    }
    cfsetospeed(&tty, B115200); // 设置波特率为9600
    tty.c_cflag &= ~PARENB;     // 无校验位
    tty.c_cflag &= ~CSTOPB;     // 1个停止位
    tty.c_cflag &= ~CSIZE;      // 8个数据位
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;       // 禁用硬件流控制
    tty.c_cflag |= CREAD | CLOCAL; // 启用接收和本地模式

    // 应用新的串口设置
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        std::cout << "无法应用新的串口设置" << std::endl;
        return 1;
    }

    MyStruct myStruct;

    // 发送数据
    while (1)
    {
        if (myStruct.seq >= 256)
            myStruct.seq = (int8_t)0;
        else
            myStruct.seq++;

        myStruct.CRC8 = Get_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&myStruct), 4, 0xff);
        myStruct.CRC16 = Get_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&myStruct), sizeof(myStruct) - 2, 0xffff);

        int bytes_written = write(serial_port, &myStruct, sizeof(myStruct));
        if (bytes_written < 0)
        {
            std::cout << "无法发送数据" << std::endl;
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "接收到的数据: 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(myStruct.data_length) << std::endl;
        // printf("haha\n");
    }

    // 关闭串口
    close(serial_port);

    return 0;
}
*/

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <serial_driver/serial_port.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "radar_serial/crc.hpp"

struct SendPacket
{
    uint8_t SOF = (uint8_t)0xA5;
    uint16_t data_length = 0x000A;
    uint8_t seq;
    uint8_t CRC8;

    uint16_t cmd_id = 0x0305U;

    uint16_t target_robot_ID = 0x0001U;
    float target_position_x = 1.0;
    float target_position_y = 1.0;

    uint16_t CRC16;
} __attribute__((packed));

inline std::vector<uint8_t> toVector(const SendPacket &data)
{
    std::vector<uint8_t> packet(sizeof(SendPacket));
    std::copy(
        reinterpret_cast<const uint8_t *>(&data),
        reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
    return packet;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("serial_node");

    uint32_t baud_rate = 115200;

    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;

    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
printf("1");
    serial_driver_->init_port("/dev/ttyUSB0", *device_config_);

    // SendPacket packet;
    //
    // packet.CRC8 = Get_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&packet), 4, 0xff);
    // packet.CRC16 = Get_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet) - 2, 0xffff);
    //
    // std::vector<uint8_t> data = toVector(packet);
    //
    // serial_driver_->port()->send(data);
    //
    if (serial_driver_->port()->is_open())
    {
        
        serial_driver_->port()->close();
    }

    rclcpp::shutdown();
    return 0;
}