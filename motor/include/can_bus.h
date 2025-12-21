
#ifndef CANBUS_H
#define CANBUS_H

#include <stdio.h>
#include <string.h>
#include <unistd.h> 
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>


#if defined(_WIN32) || defined(__CYGWIN__) || defined(_WIN32_WCE)
#define LIBUSB_CALL __stdcall
#else
#define LIBUSB_CALL
#endif

typedef unsigned int u32 ;
typedef unsigned char u8 ;

typedef  struct  _Dev_Info {
    char            HW_Type[32]; //设备型号 字符串
    char            HW_Ser[32];  //设备序列号 字符串 
    char            HW_Ver[32];  //硬件版本 字符串
    char            FW_Ver[32];  //软件版本 字符串
    char            MF_Date[32]; //生产日期 字符串
} Dev_Info, *PDev_Info;

typedef struct _Can_Config {
    unsigned int    Baudrate;
    unsigned short  Pres;        // Pres,Tseg1,Tseg2如果与Baudrate不匹配，动态库会自动按
    unsigned char   Tseg1;       // Baudrate重新计算Pres,Tseg1,Tseg2,SJW并且采样点设置为
    unsigned char   Tseg2;       // 75%左右。一般情况可以只设置波特率，Pres,Tseg1,Tseg2填0
    unsigned char   SJW;
    unsigned char   Config;      //配置信息：0x01接通内部电阻 0x02离线唤醒 0x04自动重传
    unsigned char   Model;       //工作模式：0 正常模式,1 环回模式,2 静默模式,3 静默环回模式
    unsigned char   Reserved;    //保留
}Can_Config, *P_Can_Config;

typedef struct _CanFD_Config {
    unsigned int    NomBaud;     //常规波特率
    unsigned int    DatBaud;     //数据波特率
    unsigned short  NomPre;      // NomPre,NomTseg1,NomTseg2如果与NomBaud不匹配，动态库会自动按
    unsigned char   NomTseg1;    // NomBaud重新计算NomPre,NomTseg1,NomTseg2,NomSJW并且采样点设置为
    unsigned char   NomTseg2;    // 75%左右。这些值可借助波特率计算器计算后设置
    unsigned char   NomSJW;
    unsigned char   DatPre;      // DatPre,DatTseg1,DatTseg2如果与DatBaud不匹配，动态库会自动按
    unsigned char   DatTseg1;    // DatBaud重新计算DatPre,DatTseg1,DatTseg2,DatSJW并且采样点设置为
    unsigned char   DatTseg2;    // 75%左右。这些值可借助波特率计算器计算后设置
    unsigned char   DatSJW;
    unsigned char   Config;      //配置信息：0x01接通内部电阻 0x02离线唤醒 0x04自动重传
    unsigned char   Model;       //工作模式：0 正常模式,1 环回模式,2 静默模式,3 静默环回模式
    unsigned char   Cantype;     //CAN模式：0 CAN,1 IOS CANFD,2 Non-ISO CANFD
}CanFD_Config, *P_CanFD_Config;

typedef  struct  _Can_Msg {
    unsigned int    ID;          //报文ID
    unsigned int    TimeStamp;   //微秒级时间戳
    unsigned char   FrameType;   //帧类型
    unsigned char   DataLen;     //有效字节数
    unsigned char   ExternFlag;  //扩展帧标识：0标准帧,1扩展帧
    unsigned char   RemoteFlag;  //远程帧标识：0数据帧,1远程帧
    unsigned char   BusSatus;    //总线状态
    unsigned char   ErrSatus;    //错误状态
    unsigned char   TECounter;   //发送错误计数
    unsigned char   RECounter;   //接收错误计数
    unsigned char   Data[8];     //报文数据
}Can_Msg, *P_Can_Msg;

typedef  struct  _CanFD_Msg {
    unsigned int    ID;          //报文ID
    unsigned int    TimeStamp;   //微秒级时间戳
    unsigned char   FrameType;   //帧类型
    unsigned char   DLC;         //DLC不等于数据长度。最大值15，对于数据长度64
    unsigned char   ExternFlag;  //扩展帧标识：0标准帧,1扩展帧
    unsigned char   RemoteFlag;  //远程帧标识：0数据帧,1远程帧
    unsigned char   BusSatus;    //总线状态
    unsigned char   ErrSatus;    //错误状态
    unsigned char   TECounter;   //发送错误计数
    unsigned char   RECounter;   //接收错误计数
    unsigned char   Data[64];    //报文数据
}CanFD_Msg, *P_CanFD_Msg;

typedef struct _Can_Status {
    unsigned char   BusSatus;    //总线状态
    unsigned char   ErrSatus;    //错误状态
    unsigned char   TECounter;   //发送错误计数
    unsigned char   RECounter;   //接收错误计数
    unsigned int    TimeStamp;   //产生状态时的时间戳
}Can_Status, *P_Can_Status;

#ifdef __cplusplus
extern "C" {
#endif

// devNum表示设备索引号，chNum表示通道号  当插入一个双通道CAN设备时 devNum为0， chNum为0或者1；
//  当插入N个双通道CAN设备时 devNum为0至(N-1)， chNum为0或者1；

int LIBUSB_CALL   CAN_ScanDevice(void);                   //扫描CAN,CANFD设备
int LIBUSB_CALL   CAN_OpenDevice(u32 devNum, u32 chNum);    //打开CAN,CANFD设备
int LIBUSB_CALL   CAN_CloseDevice(u32 devNum, u32 chNum);   //关闭CAN,CANFD设备

int LIBUSB_CALL   CAN_GetDevType(u32 devNum);     //获取CAN,CANFD类型 0：常规CAN，1：CANFD
int LIBUSB_CALL   CAN_GetDevPlck(u32 devNum);     //获取CAN,CANFD主频
int LIBUSB_CALL   CAN_ReadDevInfo(u32 devNum, PDev_Info devinfo);  //读取CAN,CANFD设备信息

int LIBUSB_CALL   CAN_GetTimeStamp(u32 devNum, u32 chNum, u32 *timestamp);//获取CAN,CANFD时间戳
int LIBUSB_CALL   CAN_SetTimeStamp(u32 devNum, u32 chNum, u32 timestamp, char mode);//设置CAN,CANFD时间戳  mode 0：设置当前通道，1设置同硬件所有通道

int LIBUSB_CALL   CAN_GetDevID(u32 devNum, u32 chNum, u32 *id);//获取CAN,CANFD通道ID 多设备时便于区分CAN通道
int LIBUSB_CALL   CAN_SetDevID(u32 devNum, u32 chNum, u32 id);//设置CAN,CANFD通道ID

int LIBUSB_CALL   CAN_SetFilter(u32 devNum, u32 chNum, char namber, char type, u32 ftID, u32 ftMask, char enable); //设置CAN,CANFD硬件屏蔽
int LIBUSB_CALL   CAN_Reset(u32 devNum, u32 chNum);         //复位CAN,CANFD设备
int LIBUSB_CALL   CAN_GetStatus(u32 devNum, u32 chNum, P_Can_Status status);                  //获取CAN,CANFD设备状态
int LIBUSB_CALL   CAN_Init(u32 devNum, u32 chNum, P_Can_Config pInitConfig);   //初始化CAN设备
int LIBUSB_CALL   CAN_Transmit(u32 devNum, u32 chNum, P_Can_Msg canmsg, u32 items, int timeou);  //发送CAN报文
int LIBUSB_CALL   CAN_TransmitRt(u32 devNum, u32 chNum, P_Can_Msg canmsg, u32 items, u32 *txitems, int timeou);   //定时发送CAN报文
int LIBUSB_CALL   CAN_GetReceiveNum(u32 devNum, u32 chNum); //获取接收缓冲区中接收到但尚未被读取的帧数量
int LIBUSB_CALL   CAN_Receive(u32 devNum, u32 chNum, P_Can_Msg canmsg, int Len, int timeou);  //接收CAN报文

int LIBUSB_CALL   CANFD_Init(u32 devNum, u32 chNum, P_CanFD_Config pInitConfig);
int LIBUSB_CALL   CANFD_Transmit(u32 devNum, u32 chNum, P_CanFD_Msg canmsg, u32 items, int timeout);
int LIBUSB_CALL   CANFD_TransmitRt(u32 devNum, u32 chNum, P_CanFD_Msg canmsg, u32 items, u32 *txitems, int timeou);   //定时发送CAN报文
int LIBUSB_CALL   CANFD_GetReceiveNum(u32 devNum, u32 chNum);//获取接收缓冲区中接收到但尚未被读取的帧数量
int LIBUSB_CALL   CANFD_Receive(u32 devNum, u32 chNum, P_CanFD_Msg canmsg, int Len, int timeout); //接收CANFD报文	

#ifdef __cplusplus
}
#endif

#endif

