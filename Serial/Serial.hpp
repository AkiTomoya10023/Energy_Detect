#ifndef SERIAL_HPP_
#define SERIAL_HPP_

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdint.h>
#include <mutex>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <string>
#include <string.h>
#include <stdexcept>

#include <exception>
#include <unistd.h>
#include <fstream>
#include <memory>
#include <memory.h>
#include <iomanip>

#undef EOF

struct Test_send
{
    //  //uint8_t name;
    //  int id;
    //  float delaytime;
    //  //int distance;
    //  float yaw_angle;
    //  float pitch_angle;
    //  //uint16_t detail;
    uint8_t sof;
    float yaw_angle;
    float pitch_angle;
    // float distance;
    uint8_t eof;
};

struct Test_receive
{
    //  int ctrl_mode;
    //  int shoot_speed;
    //  //int task_mode;
    char sof;
    char robot_id;
    char mode;
    char eof;
};

struct Test_send_frame
{
    //    uint8_t sof;
    //    //uint8_t name;
    //    int id;
    //    float delaytime;
    //    //float delaytime;
    //    //int distance;
    //    float pitch_angle;
    //    float yaw_angle;
    //    //uint16_t detail;
    //    uint8_t eof;
    char sof;
    float yaw_angle;
    float pitch_angle;
    // float distance;
    char eof;
};

// Test_receive _test_receive;

class Serial
{

private:
    int _serialFd;

    enum
    {
        // JetsonCommSOF = (uint8_t)0x66,
        JetsonCommSOF = (uint8_t)0xAA,
        // JetsonCommEOF = (uint8_t)0x88
        JetsonCommEOF = (uint8_t)0x55
    };

public:
    int openPort(std::string portName);
    int closePort();
    int send(const Test_send &test_send);
    Test_receive receive();
    // Serial::Test_receive get_receive();
    Test_send_frame pack(const Test_send &ctrl);
    // static uint16_t TransShort(uint8_t low, uint8_t high);
    // static float TransFloat(uint8_t f1, uint8_t f2, uint8_t f3, uint8_t f4);
};

#endif // SERIAL_HPP_