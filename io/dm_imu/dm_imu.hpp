#ifndef IO__Dm_Imu_HPP
#define IO__Dm_Imu_HPP

#include <math.h>
#include <serial/serial.h>

#include <Eigen/Geometry>
#include <array>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <thread>

#include "tools/thread_safe_queue.hpp"

namespace io
{
//6轴IMU传感器
struct __attribute__((packed)) IMU_Receive_Frame
{
  /*
  0x55 │ 0xAA │ 0x01 │  reg │  data...    │ CRC16│ 帧尾 │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
帧头    标志    从机ID  寄存器  4字节数据   2字节校验  帧尾
  数据帧格式
  总长：57字节
  */
  //加速度计
  uint8_t FrameHeader1;
  uint8_t flag1;
  uint8_t slave_id1;
  uint8_t reg_acc;//加速度计寄存器地址
  uint32_t accx_u32;
  uint32_t accy_u32;
  uint32_t accz_u32;
  uint16_t crc1;//CRC校验
  uint8_t FrameEnd1;
  //陀螺仪
  uint8_t FrameHeader2;
  uint8_t flag2;
  uint8_t slave_id2;
  uint8_t reg_gyro;//陀螺仪寄存器地址
  uint32_t gyrox_u32;
  uint32_t gyroy_u32;
  uint32_t gyroz_u32;
  uint16_t crc2;
  uint8_t FrameEnd2;
  //欧拉角，内置算法算出，并没有磁力计
  uint8_t FrameHeader3;
  uint8_t flag3;
  uint8_t slave_id3;
  uint8_t reg_euler;  //r-p-y//欧拉角寄存器地址
  uint32_t roll_u32;
  uint32_t pitch_u32;
  uint32_t yaw_u32;
  uint16_t crc3;
  uint8_t FrameEnd3;
};
//解析后的数据
typedef struct
{
  float accx;
  float accy;
  float accz;
  float gyrox;
  float gyroy;
  float gyroz;
  float roll;
  float pitch;
  float yaw;
} IMU_Data;

class DM_IMU
{
public:
  DM_IMU();
  ~DM_IMU();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

private:
  struct IMUData
  {
    Eigen::Quaterniond q;//四元数
    std::chrono::steady_clock::time_point timestamp;
  };

  void init_serial();//初始化串口
  void get_imu_data_thread();

  serial::Serial serial_;
  std::thread rec_thread_;

  tools::ThreadSafeQueue<IMUData> queue_;
  IMUData data_ahead_, data_behind_;

  std::atomic<bool> stop_thread_{false};
  IMU_Receive_Frame receive_data{};  //receive data frame
  IMU_Data data{};//解析后的数据
};

}  // namespace io

#endif
