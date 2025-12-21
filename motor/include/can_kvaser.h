#ifndef _CAN_KVASER_H
#define _CAN_KVASER_H


#include <canlib.h>
#include <iostream>
#include <cassert>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <unistd.h>

class CANController {
private:
    canHandle hnd;  // CAN 通道句柄
    int channel;    // 通道号
    int id;         // CAN ID

public:
    CANController(int channel = 0,int id = 1);
    ~CANController();

    bool initialize();
    bool sendCommand(unsigned char* data, size_t size);
    std::vector<uint8_t> revieveFeedBack();
    void close();
    void control(CANController canController(const int CAN_CHANNEL,const int DEVICE_ID));
};

int control_command(int channel, int id, uint8_t spinDir, uint16_t maxSpeed, uint32_t angleControl);



#endif

