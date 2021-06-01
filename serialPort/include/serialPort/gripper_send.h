#ifndef SRC_UART_TUTORIAL_INCLUDE_GRIPPER_REC_H_
#define SRC_UART_TUTORIAL_INCLUDE_GRIPPER_REC_H_
 
#include <ros/ros.h>                           // 包含ROS的头文件
#include <iostream>
#include <queue>
 
constexpr int RX_USB_SIZE = 150;   //设置临时存储的字节容量
 
struct RecStruct {   //设置一个结构体，包括布尔值，临时存储的数组，计数器
    bool isValue = false;
    char buffer[16];
    uint16_t buff_ptr = 0;
};
 
class DataGripper {  
 public:
    DataGripper(uint32_t baund);  //构造函数，判断串口是否成功打开和初始化
    ~DataGripper(); //析构函数
    bool DataReceive();   //判断是否接受到信号
    void DataDecode();    //判断帧头帧尾，将中间的有效数据存在rec_decode_.buffer中
    void GetData(char *effective_data);  //对信号进行解码
 
    short motor1_state_Value_[3];  //三个电机的状态
    short motor2_state_Value_[3];
    short motor3_state_Value_[3];
 private:
    char rcv_buf_[RX_USB_SIZE];  
    std::queue<char> rec_que_;
    char slide_windows_[4];  //滑动窗口
    int fd_ = -1;  //文件描述符
    int uart_open_err_ = false;
    struct RecStruct rec_decode_;  
    
};
 
#endif  // SRC_UART_TUTORIAL_INCLUDE_GRIPPER_REC_H_