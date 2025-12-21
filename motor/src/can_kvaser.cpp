#include "can_kvaser.h"


// CANController 类的成员函数定义

CANController::CANController(int channel,int id) : hnd(-1), channel(channel) ,id(id){
    canInitializeLibrary(); // 初始化 CANlib 库
}

CANController::~CANController() {
    if (hnd >= 0) {
        canBusOff(hnd);  // 关闭 CAN 总线
        canClose(hnd);   // 关闭 CAN 通道
    }
}

bool CANController::initialize() {
    hnd = canOpenChannel(channel, canOPEN_ACCEPT_VIRTUAL); // 打开 CAN 通道
    if (hnd < 0) {
        std::cerr << "打开 CAN 通道失败: " << hnd << std::endl;
        return false;
    }

    canStatus stat = canSetBusParams(hnd, canBITRATE_1M, 0, 0, 0, 0, 0); // 设置总线参数
    if (stat != canOK) {
        std::cerr << "设置总线参数失败: " << stat << std::endl;
        canClose(hnd);
        return false;
    }

    stat = canBusOn(hnd); // 启用 CAN 总线
    if (stat != canOK) {
        std::cerr << "启用总线失败: " << stat << std::endl;
        canClose(hnd);
        return false;
    }

    return true;
}

bool CANController::sendCommand(unsigned char* data, size_t size) {
    canStatus stat = canWrite(hnd, id, data, size, 0); // 发送命令
    if (stat != canOK) {
        std::cerr << "发送数据失败: " << stat << std::endl;
        return false;
    }

    stat = canWriteSync(hnd, 500); // 等待数据发送完成（500 毫秒）
    if (stat != canOK) {
        std::cerr << "等待数据发送完成失败: " << stat << std::endl;
        return false;
    }
    std::cerr << "数据发送完成成功" << std::endl;
    return true;
}

std::vector<uint8_t> CANController::revieveFeedBack(){
     uint8_t receivedMsg[8] = {0};
    long can_id = 0;
    unsigned int dlc = 0;
    
    // 使用正确的句柄hnd，设置合理超时(如1000ms)
    canStatus res = canReadWait(hnd, &can_id, receivedMsg, &dlc, nullptr, nullptr, 5000);
    
    std::vector<uint8_t> receiveData;
    
    if (res == canOK) {
        for (unsigned int i = 0; i < dlc; i++) {
            receiveData.push_back(receivedMsg[i]);
        }
    } else if (res != canERR_NOMSG) {
        std::cerr << "接收数据错误: " << res << std::endl;
    }
    
    return receiveData;
    
}

void CANController::close() {
    if (hnd >= 0) {
        canBusOff(hnd); // 关闭 CAN 总线
        canClose(hnd);  // 关闭 CAN 通道
    }
}

// 发送电机命令
int control_command(int channel, int id, uint8_t spinDir, uint16_t maxSpeed, uint32_t angleControl) {
    CANController canController(channel, id);
    std::cout << "正在初始化CAN控制器..." << std::endl;
    if (!canController.initialize()) {
        std::cerr << "CAN控制器初始化失败！" << std::endl;
        return -1;
    }

    unsigned char command[8];
    command[0] = 0xA6; // 命令头
    command[1] = spinDir & 0x01; // spinDirection
    command[2] = maxSpeed & 0xFF; // maxSpeed low
    command[3] = (maxSpeed >> 8) & 0xFF; // maxSpeed high
    command[4] = angleControl & 0xFF; // angleControl low
    command[5] = (angleControl >> 8) & 0xFF; // angleControl mid-low
    command[6] = (angleControl >> 16) & 0xFF; // angleControl mid-high
    command[7] = (angleControl >> 24) & 0xFF; // angleControl high

    if (!canController.sendCommand(command, 8)) {
        std::cerr << "命令发送失败！" << std::endl;
        canController.close();
        return -1;
    }
    std::cout <<id<<"命令发送成功！" << std::endl;
    
    std::cout <<angleControl<<std::endl;

    std::vector<uint8_t> feedback = canController.revieveFeedBack();
    if (!feedback.empty()) {
        std::cout << "接收到反馈数据(" << feedback.size() << "字节): ";
        for (auto byte : feedback) {
            std::cout << std::hex << static_cast<int>(byte) << " ";
        }
        std::cout << std::dec << std::endl;
    } else {
        std::cout << "未接收到反馈数据或接收超时" << std::endl;
    }

    canController.close();
    // sleep(1); // 降低延迟
    return 0;
}


class MotorBridge {
public:
    MotorBridge() {
        sub_ = nh_.subscribe("motor_commands", 4, &MotorBridge::commandCallback, this);
    }

    uint32_t coxa_spin = 0;
    uint32_t femur_spin = 0;
    uint32_t tibia_spin = 0;

    void commandCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // 调整后的基 CAN ID，确保无冲突
        std::vector<int> base_can_ids = {0x141, 0x144, 0x147, 0x14A}; // FL, RL, FR, RR
        int channel = 0;
        uint8_t spinDir[2]= {0x00,0x01}; // 方向选项
        uint16_t maxSpeed = 900; // 速度调节

           
    
        for (size_t i = 0; i < 1; ++i) {
            uint32_t coxa_angle = static_cast<uint32_t>(msg->position[i * 3]);
            uint32_t femur_angle = static_cast<uint32_t>(msg->position[i * 3 + 1]);
            uint32_t tibia_angle = static_cast<uint32_t>(msg->position[i * 3 + 2]);
           
            // 发送到 coxa 关节
            if(coxa_angle-coxa_spin>0 && coxa_angle-coxa_spin<=648000)//判断电机旋转方向
            control_command(channel, base_can_ids[i*3], spinDir[0], maxSpeed, coxa_angle);
            else
            control_command(channel, base_can_ids[i*3], spinDir[1], maxSpeed, coxa_angle);

            // 发送到 femur 关节
            if(femur_angle-femur_spin>0 && femur_angle-femur_spin<=648000)
            control_command(channel, base_can_ids[i*3]+1, spinDir[0], maxSpeed, femur_angle);
            else
            control_command(channel, base_can_ids[i*3]+1, spinDir[1], maxSpeed, femur_angle);

            // 发送到 tibia 关节
            if(tibia_angle-tibia_spin>0 && tibia_angle-tibia_spin<=648000)
            control_command(channel, base_can_ids[i*3]+2, spinDir[0], maxSpeed, tibia_angle);
            else
            control_command(channel, base_can_ids[i*3]+2, spinDir[1], maxSpeed, tibia_angle);


           coxa_spin = coxa_angle;
           femur_spin = femur_angle;
           tibia_spin = tibia_angle; 
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};


//电机归零
void set_zero(){
    for(int i=0 ; i<12 ;i++)
   control_command(0, 0x141+i, 1, 900, 0);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_bridge");
    set_zero();
    sleep(2);
    MotorBridge bridge;
    ros::spin();
    return 0;
}
