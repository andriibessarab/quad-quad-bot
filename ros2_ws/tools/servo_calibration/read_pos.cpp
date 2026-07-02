// Script used to preview STS3215 positions in real
// time independent of ROS WS for calibration purposes.

#include <scservo/SCServo.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

static volatile bool running = true;

static void handle_sigint(int) { running = false; }

static void print_usage(const char *prog) {
  std::cerr << "Usage: " << prog
            << " <servo_id> [--port <dev>] [--baud <rate>]\n"
            << "  servo_id   Motor ID to read (e.g. 11 for fr_haa_joint)\n"
            << "  --port     Serial device (default: /dev/ttyUSB0)\n"
            << "  --baud     Baud rate     (default: 1000000)\n";
}

int main(int argc, char **argv) {
  if (argc < 2) {
    print_usage(argv[0]);
    return 1;
  }

  int id = std::atoi(argv[1]);
  if (id <= 0 || id > 253) {
    std::cerr << "Error: servo_id must be 1-253\n";
    return 1;
  }

  std::string port = "/dev/ttyUSB0";
  int baud = 1000000;

  for (int i = 2; i < argc; ++i) {
    if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      port = argv[++i];
    } else if (std::strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
      baud = std::atoi(argv[++i]);
    } else {
      std::cerr << "Unknown argument: " << argv[i] << "\n";
      print_usage(argv[0]);
      return 1;
    }
  }

  std::signal(SIGINT, handle_sigint);

  SMS_STS servo;
  if (!servo.begin(baud, port.c_str())) {
    std::cerr << "Failed to open " << port << " at " << baud << " baud\n";
    return 1;
  }

  if (servo.Ping(static_cast<uint8_t>(id)) == -1) {
    std::cerr << "Servo ID " << id << " did not respond on " << port << "\n";
    servo.end();
    return 1;
  }

  if (servo.EnableTorque(static_cast<uint8_t>(id), 0) == 0) {
    std::cerr << "Warning: failed to disable torque on servo " << id
              << " — move with care\n";
  }

  std::cout << "Servo " << id << " ready — torque disabled, move joint freely\n"
            << "Press Ctrl+C to exit\n\n";

  constexpr double kRadPerTick = 2.0 * M_PI / 4096.0;

  while (running) {
    if (servo.FeedBack(static_cast<uint8_t>(id)) != 0) {
      int ticks = servo.ReadPos(-1);
      double rad = ticks * kRadPerTick;
      std::cout << "\rPos: " << ticks << " ticks  (" << rad << " rad)   "
                << std::flush;
    } else {
      std::cout << "\rRead failed — retrying..." << std::flush;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  std::cout << "\nDone.\n";
  servo.end();
  return 0;
}
