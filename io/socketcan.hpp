#ifndef IO__SOCKETCAN_HPP
#define IO__SOCKETCAN_HPP

#include <linux/can.h>
#include <net/if.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <thread>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

constexpr int MAX_EVENTS = 10;
/*
Linux SocketCAN 架构
户空间程序
    │
    ▼
Socket API: socket() → bind() → read()/write()
    │
    ▼
Linux CAN 协议栈 (net/can/)
    │
    ▼
硬件驱动 (USB2CAN / 虚拟串口)
    │
    ▼
CAN 总线收发器 (TJA1050 等)
核心机制
┌───────────────────────────────────────────┐
│              SocketCAN                    │
│                                           │
│  初始化: socket() → ioctl() → bind()       │
│          → epoll_create()                 │
│                                           │
│  接收线程: epoll_wait() → recv() → 回调     │
│                                           │
│  守护线程: 每100ms检查，断线自动重连           │
│                                           │
│  写操作: ::write(socket_fd_, frame)        │
└───────────────────────────────────────────┘
socket(PF_CAN, SOCK_RAW, CAN_RAW);
PF_CAN: 套接字协议族,代表CAN 协议  AF_INET: IPv4 协议
SOCK_RAW: 原始套接字,不用高级加密  SOCK_STREAM: TCP 套接字
SOCK_DGRAM: UDP 套接字
CAN_RAW: 原始帧,不包含任何协议层信息

获取接口索引
ioctl(socket_fd_, SIOCGIFINDEX, &ifr);
bind() 绑定接口

CBoard 构造函数                                               │
│                                                             │
│  1. can_("can0", &CBoard::callback)  ← 创建SocketCAN       │
│                              ↓                              │
│  2. SocketCAN 构造函数                                       │
│     ├── try_open() → open()                                  │
│     │   ├── socket() → 创建原始CAN套接字                      │
│     │   ├── ioctl() → 获取 can0 的接口索引                    │
│     │   ├── bind() → 绑定到 can0                             │
│     │   ├── epoll_create() → 创建异步事件通知                  │
│     │   └── 启动 read_thread 【循环: epoll_wait→recv→回调】   │
│     └── 启动 daemon_thread 【每100ms检查,自动重连】            │
│                                                             │
│  3. 回调函数收到数据                                         │
│     ├── quaternion_canid_ → 解析四元数 → 推入队列             │
│     └── bullet_speed_canid_ → 更新弹速/模式                   │
│                                                             │
│  4. 主循环 (standard.cpp)                                    │
│     ├── cboard.imu_at(t) → 从队列取IMU数据 → 四元数插值       │
│     └── cboard.send(command) → SocketCAN::write()           │
│                          ↓                                   │
│                     ::write(socket_fd_, frame)                │
│                          ↓                                   │
│                    CAN 总线上传输到 STM32      





*/






namespace io
{
class SocketCAN
{
public:
  SocketCAN(const std::string & interface, std::function<void(const can_frame & frame)> rx_handler)
  : interface_(interface),//can总线接口
    socket_fd_(-1),
    epoll_fd_(-1),
    rx_handler_(rx_handler),
    quit_(false),
    ok_(false)
  {
    try_open();

    // 守护线程
    daemon_thread_ = std::thread{[this] {
      while (!quit_) {
        std::this_thread::sleep_for(100ms);

        if (ok_) continue;

        if (read_thread_.joinable()) read_thread_.join();

        close();
        try_open();
      }
    }};
  }

  ~SocketCAN()
  {
    quit_ = true;
    if (daemon_thread_.joinable()) daemon_thread_.join();
    if (read_thread_.joinable()) read_thread_.join();
    close();
    tools::logger()->info("SocketCAN destructed.");
  }

  void write(can_frame * frame) const
  {
    if (::write(socket_fd_, frame, sizeof(can_frame)) == -1)
      throw std::runtime_error("Unable to write!");
  }

private:
  std::string interface_;
  int socket_fd_;
  int epoll_fd_;
  bool quit_;
  bool ok_;
  std::thread read_thread_;
  std::thread daemon_thread_;
  can_frame frame_;
  epoll_event events_[MAX_EVENTS];
  std::function<void(const can_frame & frame)> rx_handler_;
  //初始化
  void open()
  {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);//
    if (socket_fd_ < 0) throw std::runtime_error("Error opening socket!");
//     ifreq 是 Linux 内核中专门用于网络接口配置请求的结构体（Interface Request）。

// 它内部包含一个联合体，用于存储接口名或配置参数。在这个场景中，主要使用它的 ifr_name 字段。
    ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    // SIOCGIFINDEX 是一个 ioctl 命令，用于获取指定网络接口的索引号。
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
      throw std::runtime_error("Error getting interface index!");

    sockaddr_can addr;
    std::memset(&addr, 0, sizeof(sockaddr_can));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, (sockaddr *)&addr, sizeof(sockaddr_can)) < 0) {
      ::close(socket_fd_);
      throw std::runtime_error("Error binding socket to interface!");
    }

    epoll_event ev;
    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ == -1) throw std::runtime_error("Error creating epoll file descriptor!");

    ev.events = EPOLLIN;//EPOLLIN表示对应的文件描述符可以读（包括对端SOCKET正常关闭）。
    ev.data.fd = socket_fd_;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, ev.data.fd, &ev))
      throw std::runtime_error("Error adding socket to epoll file descriptor!");

    // 接收线程
    read_thread_ = std::thread([this]() {
      ok_ = true;
      while (!quit_) {
        std::this_thread::sleep_for(10us);

        try {
          read();
        } catch (const std::exception & e) {
          tools::logger()->warn("SocketCAN::read() failed: {}", e.what());
          ok_ = false;
          break;
        }
      }
    });

    tools::logger()->info("SocketCAN opened.");
  }

  void try_open()
  {
    try {
      open();
    } catch (const std::exception & e) {
      tools::logger()->warn("SocketCAN::open() failed: {}", e.what());
    }
  }

  void read()
  {
    int num_events = epoll_wait(epoll_fd_, events_, MAX_EVENTS, 2);
    if (num_events == -1) throw std::runtime_error("Error wating for events!");

    for (int i = 0; i < num_events; i++) {
      ssize_t num_bytes = recv(socket_fd_, &frame_, sizeof(can_frame), MSG_DONTWAIT);
      if (num_bytes == -1) throw std::runtime_error("Error reading from SocketCAN!");

      rx_handler_(frame_);
    }
  }

  void close()
  {
    if (socket_fd_ == -1) return;
    epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, socket_fd_, NULL);
    ::close(epoll_fd_);
    ::close(socket_fd_);
  }
};

}  // namespace io

#endif  // IO__SOCKETCAN_HPP