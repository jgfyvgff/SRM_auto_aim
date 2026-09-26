#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_addr
#include <sys/socket.h>  // socket, sendto
#include <unistd.h>      // close

namespace tools
{
Plotter::Plotter(std::string host, uint16_t port)
{
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);//创建一个UDP套接字

  destination_.sin_family = AF_INET;
  destination_.sin_port = ::htons(port);
  destination_.sin_addr.s_addr = ::inet_addr(host.c_str());
}

Plotter::~Plotter() { ::close(socket_); }//析构函数关闭套接字

void Plotter::plot(const nlohmann::json & json)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto data = json.dump();//将json对象转换为字符串
  ::sendto(
    socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_),
    sizeof(destination_));//通过 UDP 套接字将数据发送到预先指定的目标地址。

    //参数：套接字、数据指针、数据长度、标志（0 为默认）、目标地址结构体指针、地址长度。
}

}  // namespace tools