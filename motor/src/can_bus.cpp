#include "can_bus.h"

int dev = 0;
int cpot0 = 0;
int cpot1 = 1;
Can_Msg  txmsg[100];
Can_Msg  rxmsg[100];
bool can_initialized = false;  // 添加初始化状态标志


//初始化CAN模块
int all_can_init() {
    int devs,ret,plck;
    Can_Config cancfg;

    if(can_initialized) return 0; // 如果已经初始化，直接返回

    devs = CAN_ScanDevice();                //扫描设备
    // std::cout<<"设备个数 = "<<devs<<std::endl; 
    if(devs <= 0) {
        std::cout<<"没有找到设备!!"<<std::endl;
        return -1;
    }
    ret = CAN_OpenDevice(dev,cpot0);        //打开通道0(CAN1)
    if(ret != 0) {
        std::cerr << "CAN1初始化失败" << std::endl;
        return -1;
    }
    // std::cout <<"通道0打开成功!!" << std::endl;
    ret = CAN_OpenDevice(dev,cpot1);        //打开通道1(CAN2)
    if(ret != 0){
        std::cerr << "CAN2初始化失败" << std::endl;
        return -1;
    }
    // std::cout <<"通道1打开成功!!" << std::endl;
    
    plck = CAN_GetDevPlck(0);  //获取设备频率
    // std::cout<<"设备主频 = "<<plck<<"Hz"<<std::endl;

    cancfg.Model = 0;
    cancfg.Config = 0;
    cancfg.Baudrate = 1000000;  //设置波特率500k(500*1000)
    cancfg.Config |= 0x0001;  //接通内部匹配电阻
    cancfg.Config |= 0x0002;  //开启离线唤醒模式
    cancfg.Config |= 0x0004;  //开启自动重传

    ret = CAN_Init(dev,cpot0,&cancfg);         //初始化CAN1
    if(ret != 0) {
        std::cerr << "CAN1初始化失败" << std::endl;
        return -1;
    }

    ret = CAN_Init(dev,cpot1,&cancfg);     //初始化CAN2
    if(ret != 0) {
       std::cerr << "CAN2初始化失败" << std::endl;
        return -1;
    }

    can_initialized = true;  // 标记为已初始化
    return 0;
}


// 发送电机命令
int control_command( uint8_t mode,int cpot, int id, uint8_t spinDir, uint16_t maxSpeed, int32_t angleValue) {
// 确保已初始化，否则立即初始化
// if (!can_initialized) {
//     if (all_can_init() != 0) {
//         std::cerr << "CAN 未初始化，发送指令失败" << std::endl;
//         return -1;
//     }
// }

int32_t angleControl = static_cast<int32_t>(angleValue);

    txmsg[0].ID = id;
    txmsg[0].Data[0] = mode; // 命令头
    txmsg[0].Data[1] = spinDir; // spinDirection
    txmsg[0].Data[2] = maxSpeed & 0xFF; // maxSpeed low
    txmsg[0].Data[3] = (maxSpeed >> 8) & 0xFF; // maxSpeed high
    txmsg[0].Data[4] = angleControl & 0xFF; // angleControl low
    txmsg[0].Data[5] = (angleControl >> 8) & 0xFF; // angleControl mid-low
    txmsg[0].Data[6] = (angleControl >> 16) & 0xFF; // angleControl mid-high
    txmsg[0].Data[7] = (angleControl >> 24) & 0xFF; // angleControl high
    txmsg[0].DataLen = 8;

    
    if(CAN_Transmit(dev,cpot, txmsg, 10, 200) == -1){
        std::cerr << "发送失败 (ID=" << std::hex << id << ", ch=" << cpot << ")" << std::dec << std::endl;
        return -1;
    } // 发送命令到 CAN 总线
    // else {
    //     std::cout << "发送数据ID："<<txmsg[0].ID << std::endl;
    //     std::cout << "数据帧：";
    //     for(int i = 0 ; i < txmsg[0].DataLen; i++){
    //     printf("%02x ",(unsigned char)txmsg[0].Data[i]);
    //     }
    //     std::cout<<std::endl;
    // } // 打印发送数据

    if(CAN_Receive(dev, cpot, rxmsg, 100, 200) == -1) {
        std::cerr << "接收失败 (ID=" << std::hex << id << ", ch=" << cpot << ")" << std::dec << std::endl;
        return -1;
    } 
    else {
        // std::cout << "反馈数据ID："<<rxmsg[0].ID << std::endl;
        // std::cout << "数据帧：";
        // for(int i = 0 ; i < rxmsg[0].DataLen; i++){
        // printf("%02x ",(unsigned char)rxmsg[0].Data[i]);
        // }
        // std::cout<<std::endl;
    }// 接收反馈数据
   
    return 0;
}

void stand()
{
    std::vector<int> base_can_ids = {0x141, 0x144, 0x147, 0x14A}; // FL, RL, FR, RR
    std::vector<int> dir= {0x00, 0x00, 0x00};
    std::vector<int> stand = {17803,  204013,  355975, 
                              17803, -204013, -355975,
                             -17803, -204013, -355975,
                             -17803,  204013,  355975};
    // 站立姿态

    std::vector<int> stand_1 = {17803,  186435,  442834, 
                                17803, -186435, -442834,
                               -17803, -186435, -442834,
                               -17803,  186435,  442834};
    for(int i = 0; i < 4; ++i) 
    {
        control_command(0xA4 ,cpot0, base_can_ids[i], dir[i*3], 600, stand_1[i*3]); // coxa
        control_command(0xA4 ,cpot0, base_can_ids[i] + 1, dir[i*3+1], 600, stand_1[i*3+1]); // femur
        control_command(0xA4 ,cpot0, base_can_ids[i] + 2, dir[i*3+2], 600, stand_1[i*3+2]); // tibia
    } 
}

void sit_down()
{
    std::vector<int> base_can_ids = {0x141, 0x144, 0x147, 0x14A}; // FL, RL, FR, RR
    std::vector<int> dir= {0x00, 0x00, 0x00};
    std::vector<int> stand = {17803,  204013,  355975, 
                              17803, -204013, -355975,
                             -17803, -204013, -355975,
                             -17803,  204013,  355975};
    // 站立姿态

    std::vector<int> stand_1 = {17803,  186435,  442834, 
                                17803, -186435, -442834,
                               -17803, -186435, -442834,
                               -17803,  186435,  442834};
    for(int i = 0; i < 4; ++i) 
    {
        control_command(0xA4 ,cpot0, base_can_ids[i], dir[i*3], 600, stand[i*3]); // coxa
        control_command(0xA4 ,cpot0, base_can_ids[i] + 1, dir[i*3+1], 600, stand[i*3+1]); // femur
        control_command(0xA4 ,cpot0, base_can_ids[i] + 2, dir[i*3+2], 600, stand[i*3+2]); // tibia
    } 
}

class MotorBridge {
public:
    MotorBridge() {
      sub_ = nh_.subscribe("motor_commands", 12, &MotorBridge::commandCallback, this);
    }
    

    void commandCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // 调整后的基 CAN ID，确保无冲突
        std::vector<int> base_can_ids = {0x141, 0x144, 0x147, 0x14A}; // FL, RL, FR, RR
        std::vector<int> gpio = {33,31,35,29};
        int channel = 0;
        uint8_t spinDir= 0x00; // 方向选项
        uint16_t maxSpeed = 1200; // 速度调节

        
    
        for (size_t i = 0; i < 4; ++i) {
            int32_t coxa_angle = static_cast<int64_t>(msg->position[i * 3]);
            int32_t femur_angle = static_cast<int64_t>(msg->position[i * 3 + 1]);
            int32_t tibia_angle = static_cast<int64_t>(msg->position[i * 3 + 2]);
            int32_t coxa_velocity = static_cast<int32_t>(msg->velocity[i * 3]);
            int32_t femur_velocity = static_cast<int32_t>(msg->velocity[i * 3 + 1]);
            int32_t tibia_velocity = static_cast<int32_t>(msg->velocity[i * 3 + 2]);

            ROS_INFO("Leg %zu Command: Coxa Angle = %d, Femur Angle = %d, Tibia Angle = %d",
                     i, coxa_angle, femur_angle, tibia_angle);
            ROS_INFO("Leg %zu Command: Coxa Velocity = %d, Femur Velocity = %d, Tibia Velocity = %d",
                     i, coxa_velocity, femur_velocity, tibia_velocity);

            // 发送到 coxa 关节
            control_command(0xA4 ,channel, base_can_ids[i], spinDir, coxa_velocity, coxa_angle);
            

            // 发送到 femur 关节
            control_command(0xA4 ,channel, base_can_ids[i]+1, spinDir, femur_velocity, femur_angle);
            

            // 发送到 tibia 关节
            control_command(0xA4 ,channel, base_can_ids[i]+2, spinDir, tibia_velocity, tibia_angle);

        }
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};



int main(int argc, char** argv) {    
    ros::init(argc, argv, "motor_bridge");
    // 程序启动时初始化CAN
    // if (all_can_init() != 0) {
    //     std::cerr << "CAN初始化失败，程序退出!" << std::endl;
    //     return -1;
    // }
    // GPIO::setmode(GPIO::BOARD);
    // GPIO::setup({29,31,33,35}, GPIO::OUT, GPIO::HIGH);
    stand();
    sleep(2);
    MotorBridge bridge;
    ros::spin();

    // 程序退出时关闭设备
    if(can_initialized) {
        sit_down();
        // GPIO::cleanup();
        CAN_CloseDevice(dev, cpot0);
    }

    
    return 0;
}
