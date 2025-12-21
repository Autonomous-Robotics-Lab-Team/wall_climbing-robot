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
    std::cout<<"设备个数 = "<<devs<<std::endl; 
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



// 1.电机控制命令
    // unsigned char open_motor[8]={0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//启动电机
    // unsigned char close_motor[8]={0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//关闭电机
    // unsigned char stop_motor[8]={0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//停止电机
    // unsigned char motor_state[8]={0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//查看电机状态
    // unsigned char clean_motor[8]={0x9B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//清除电机异常标志
    // unsigned char position_30[8]={0xA6, 0x00, 0x08, 0x07, 0xE0, 0xA5, 0x01, 0x00};//电机转至30°位置
    // unsigned char begin_position[8]={0xA6, 0x01, 0x08, 0x07, 0x00, 0x00, 0x00, 0x00};//电机回原点位置
    // unsigned char position_zero[8]={0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//调节零点位置（不要多次使用）

//电机重置零位
int set_zero(int id){
    

    txmsg[0].ID = id; // 设置为第一个电机的CAN ID;
    txmsg[0].Data[0] = 0xA4; // 命令头
    txmsg[0].Data[1] = 0x00; // spinDirection
    txmsg[0].Data[2] = 0x08; // maxSpeed low
    txmsg[0].Data[3] = 0x07; // maxSpeed high
    txmsg[0].Data[4] = 0x00; // angleControl low
    txmsg[0].Data[5] = 0x00; // angleControl mid-low
    txmsg[0].Data[6] = 0x00; // angleControl mid-high
    txmsg[0].Data[7] = 0x00; // angleControl high
    txmsg[0].DataLen = 8;

  
    if(CAN_Transmit(dev,cpot0, txmsg, 1, 500) == -1){
        std::cerr << "发送失败" << std::endl;
        return -1;
       } 
    // 发送命令到 CAN 总线;

    if(CAN_Receive(dev, cpot0, rxmsg, 100, 500) == -1) {
        std::cerr << "接收失败" << std::endl;
        return -1;
    } 
    else {
        std::cout << "反馈数据ID："<<rxmsg[0].ID << std::endl;
        std::cout << "数据帧：";
        for(int i = 0 ; i < rxmsg[0].DataLen; i++){
        printf("%02x ",(unsigned char)rxmsg[0].Data[i]);
        }
        std::cout<<std::endl;
    }// 接收反馈数据

    // CAN_CloseDevice(dev,cpot0);            //关闭CAN1
    // CAN_CloseDevice(dev,cpot1);            //关闭CAN2
   
    return 0;
    
}


int main(int argc, char *argv[]) { 
    if (all_can_init() != 0) {
        std::cerr << "CAN初始化失败，程序退出!" << std::endl;
        return -1;
    } 
    for(int i=0;i<3;i++){
    // set_zero(0x141+i);
    // set_zero(0x144+i);
    // set_zero(0x147+i);
    set_zero(0x14A+i);
    }
     // 程序退出时关闭设备
    if(can_initialized) {
        CAN_CloseDevice(dev, cpot0);
    }

    return 0;
}