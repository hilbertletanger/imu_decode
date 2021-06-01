#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <my_msgs/ChassisMsg.h>
// #include <serialPort/ChassisMsg.h>
#include "../include/serialPort/gripper_send.h"
#define S_TIMEOUT 1

unsigned int total_length = 0 ;
int UART0_Recv(int fd, char *data, int datalen)
{
    int len=0, ret = 0;
    fd_set fs_read;
    struct timeval tv_timeout;
      
    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);

#ifdef S_TIMEOUT    
    tv_timeout.tv_sec = (10*20/230400+2);
    tv_timeout.tv_usec = 0;
    ret = select(fd+1, &fs_read, NULL, NULL, NULL);
#else
    ret = select(fd+1, &fs_read, NULL, NULL, tv_timeout);
#endif

// printf("ret = %d\n", ret);
    //如果返回0，代表在描述符状态改变前已超过timeout时间,错误返回-1
     
    if (FD_ISSET(fd, &fs_read)) {
        len = read(fd, data, datalen);
    total_length += len ;
        printf("total len = %d\n", total_length);
        return len;
    } else {
        perror("select");
        return -1;
    }
      
    return 0;
}

DataGripper::DataGripper(uint32_t baund)
{
  serial::Serial ser; //声明串口对象
  try 
  { 
  //设置串口属性，并打开串口 
  ser.setPort("/dev/ttyUSB0"); 
  ser.setBaudrate(baund); 
  serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
  ser.setTimeout(to); 
  ser.open(); 
  } 
  catch (serial::IOException& e) 
  { 
  ROS_ERROR_STREAM("Unable to open port "); 
  } 
  //检测串口是否已经打开，并给出提示信息 
  if(ser.isOpen()) 
  { 
  ROS_INFO_STREAM("Serial Port initialized"); 
  } 
}

DataGripper::~DataGripper()
{}

bool DataGripper::DataReceive() {
  int len = 0;
    len = UART0_Recv(fd_, rcv_buf_, 100);
 
  if (len > 0) {
    for (int i = 0; i < len; i++) {
      rec_que_.push(rcv_buf_[i]);
    }
    return true;
  }
  return false;
}

void DataGripper::DataDecode() {
  char rec_temp;
  while (!rec_que_.empty()) {
    rec_temp = rec_que_.front();
    rec_que_.pop();
    for (int i = 0; i < 3; i++)
      slide_windows_[i] = slide_windows_[i + 1];
    slide_windows_[3] = rec_temp;
    for(int i=0; i<4; ++i)
       slide_windows_[i] = static_cast<uint8_t>(slide_windows_[i]);
 
    while (1) {
      if (static_cast<uint8_t>(slide_windows_[0]) != 0xa0) break;
      if (static_cast<uint8_t>(slide_windows_[1]) != 0xb8) break;
      if (static_cast<uint8_t>(slide_windows_[2]) != 0x21) break;
      if (static_cast<uint8_t>(slide_windows_[3]) != 0x0a) break;
      rec_decode_.buff_ptr = 0;
      rec_decode_.isValue = true;
      break;
    }
    if (rec_decode_.isValue == true) {
      rec_decode_.buffer[rec_decode_.buff_ptr] = rec_temp;
      if (rec_decode_.buff_ptr == 13) {
        if (static_cast<uint8_t>(rec_decode_.buffer[12]) == 0x59 && static_cast<uint8_t>(rec_decode_.buffer[13]) == 0xA6) {
          GetData(&rec_decode_.buffer[2]);
          //std::cout<<motor1_state_Value[0]<<" "<<motor1_state_Value[1]<< " "<<motor2_state_Value[0]<<" "<<motor3_state_Value[0]<<std::endl;
        }
        rec_decode_.buff_ptr = 0;
        rec_decode_.isValue = false;
      } else {
        rec_decode_.buff_ptr++;
      }
    }
  }
}

void DataGripper::GetData(char *effective_data)
{
  short pos_long_h = 0;
  short pos_long_l = 0;
 
  pos_long_h = effective_data[0];
  pos_long_h = (pos_long_h << 8 ) | effective_data[1];
 
  pos_long_l = effective_data[2];
  pos_long_l = (pos_long_l << 8 ) | effective_data[3];
 
  long motor1_pos = pos_long_h;
  motor1_pos = (motor1_pos << 16) | pos_long_l;
 
  short motor1_cur = effective_data[4];
  motor1_cur = (motor1_cur << 8) | effective_data[5];
 
  short motor2_pos = effective_data[6];
  motor2_pos = (motor2_pos << 8) | effective_data[7];
 
  short motor3_pos = effective_data[8];
  motor3_pos = (motor3_pos << 8) | effective_data[9];
 
  motor1_state_Value_[0] = static_cast<short>(motor1_pos);
  motor1_state_Value_[1] = motor1_cur;
 
  motor2_state_Value_[0] = motor2_pos;
  motor3_state_Value_[0] = motor3_pos;
}


int main(int argc, char** argv) 
{
    ROS_INFO("[info] init begin");

    ros::init(argc, argv, "virtual_serial");       //初始化节点
    ros::NodeHandle n;                             //启动节点
    ros::Publisher chatter_pub = n.advertise<my_msgs::ChassisMsg>("inss_send", 1000);      //定义发布消息的名称及sulv
    ros::Rate loop_rate(100);  //控制发布速度，与sleep配合使用，每0.02秒休息一次
    DataGripper data_gripper(230400);  //构造函数，看串口是否打开并初始化
   
    while (ros::ok()) {                            //检查节点是否关闭
        //read uart data
        data_gripper.DataReceive();  //接收数据
        data_gripper.DataDecode();  //解码数据
 
        //write to msg;
        my_msgs::ChassisMsg msg;
        msg.fLatitude = data_gripper.motor1_state_Value_[0];
        msg.fLongitude = data_gripper.motor2_state_Value_[0];
        msg.fAltitude = data_gripper.motor3_state_Value_[0];
        msg.fAccX = data_gripper.motor1_state_Value_[1];
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

