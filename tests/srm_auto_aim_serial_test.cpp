#include "io/srm_auto_aim_transport.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

struct Options
{
  std::string port = "/dev/ttyACM0";
  int duration_seconds = 5;
  int period_ms = 100;
  bool send_commands = false;
};

void print_usage(const char * program)
{
  std::cout
    << "Usage: " << program << " [options]\n"
    << "  --port PATH          serial device, default /dev/ttyACM0\n"
    << "  --duration SECONDS  test duration, default 5\n"
    << "  --period-ms MS      send/poll period, default 100\n"
    << "  --send              send zero-angle, fire_flag=0 command frames\n"
    << "  --help              show this message\n";
}

int read_integer(const std::string & option, const std::string & value)
{
  try {
    std::size_t parsed = 0;
    const auto result = std::stoi(value, &parsed);
    if (parsed != value.size()) throw std::invalid_argument("trailing characters");
    return result;
  } catch (const std::exception &) {
    throw std::invalid_argument(option + " requires an integer, got: " + value);
  }
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--help") {
      print_usage(argv[0]);
      std::exit(0);
    }
    if (argument == "--send") {
      options.send_commands = true;
      continue;
    }

    if (i + 1 >= argc) {
      throw std::invalid_argument(argument + " requires a value");
    }
    const std::string value = argv[++i];
    if (argument == "--port") {
      options.port = value;
    } else if (argument == "--duration") {
      options.duration_seconds = read_integer(argument, value);
    } else if (argument == "--period-ms") {
      options.period_ms = read_integer(argument, value);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }

  if (options.port.empty() || options.duration_seconds <= 0 || options.period_ms <= 0) {
    throw std::invalid_argument("port must be non-empty and duration/period must be positive");
  }
  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse_options(argc, argv);

    io::srm_auto_aim::SerialConfig serial_config;
    serial_config.device = options.port;
    serial_config.timeout_ms = 20;
    io::srm_auto_aim::SrmAutoAimTransport transport(serial_config);

    std::cout << "Serial opened: " << options.port << '\n';
    if (options.send_commands) {
      std::cout << "TX enabled: yaw=0, pitch=0, fire_flag=0\n";
    } else {
      std::cout << "TX disabled: receive/open test only\n";
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(options.duration_seconds);
    std::size_t sent_count = 0;
    std::size_t received_count = 0;

    while (std::chrono::steady_clock::now() < deadline) {
      if (options.send_commands) {
        // 明确禁止开火；串口探针只验证协议链路，不代表自瞄控制逻辑。
        transport.send(io::srm_auto_aim::CommandFrame{0.0F, 0.0F, 0});
        ++sent_count;
      }

      const auto feedback = transport.poll_feedback();
      for (const auto & frame : feedback) {
        ++received_count;
        std::cout << "Feedback: yaw=" << frame.yaw_deg << " pitch=" << frame.pitch_deg
                  << " roll=" << frame.roll_deg << " mode=" << frame.mode
                  << " color=" << frame.color << " bullet_speed=" << frame.bullet_speed_mps
                  << "\n";
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(options.period_ms));
    }

    std::cout << "Serial test finished: TX frames=" << sent_count
              << ", RX feedback frames=" << received_count << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Serial test failed: " << error.what() << '\n';
    return 1;
  }
}
