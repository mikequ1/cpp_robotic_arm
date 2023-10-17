/**
 * velocityControl.cpp
 *
 * A Minimal Script for Controlling the Robot using Joint Velocity Commands; the Robot sends back Position, Velocity,
 * Torque, and the Jacobian.
 */

#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include "franka/exception.h"
#include <franka/robot.h>
#include <franka/model.h>

#include <liborl/liborl.h>

class VelocityController {
  public:
    VelocityController(int port);
    std::array<double, 3> getEE(orl::Robot& robot, std::array<double, 3>& curpose);
  private:
    double time = 0.0;
    std::array<double, 100> buffer1;
    std::array<double, 100> buffer2;
    std::array<double, 100> buffer3;
    std::array<double, 100> buffer4;
    std::array<double, 100> buffer5;
    std::array<double, 100> buffer6;
    std::array<double, 100> buffer7;
    std::array<double, 100> bufferRate;
    std::array<double, 7> qdot;
    std::array<double, 3> ee_dest;
    double MovingAverage(std::array<double, 100>& buffer, double input);
    int sock = 0;
    int valread;
    struct sockaddr_in serv_addr;
    char buffer[200] = {0};
    int steps = 0;
};
VelocityController::VelocityController(int port) {
  for (int i = 0; i < 100; i++) {
    buffer1[i] = 0.0;
    buffer2[i] = 0.0;
    buffer3[i] = 0.0;
    buffer4[i] = 0.0;
    buffer5[i] = 0.0;
    buffer6[i] = 0.0;
    buffer7[i] = 0.0;	
    bufferRate[i] = 0.0;
  }
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    std::cout << "Socket Creation Error!" << std::endl;
  }
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(port);
  if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
    std::cout << "Invalid Address / Address Not Supported" << std::endl;
  }
  if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    std::cout << "Connection Failed" << std::endl;
  }
  int status = fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
  if (status == -1) {
    std::cout << "Failed to Make the Socket Non-Blocking" << std::endl;
  }
}
double VelocityController::MovingAverage(std::array<double, 100>& buffer, double input) {
  double filtered_input = 0.0;
  for (int i = 0; i < 100; i++) {
    filtered_input = filtered_input + buffer[i];
  }
  filtered_input = filtered_input / 100.0;
  for (int i = 0; i < 99; i++) {
    buffer[i] = buffer[i + 1];
  }
  buffer[99] = input;
  return filtered_input;
}
std::array<double, 3> VelocityController::getEE(orl::Robot& robot, std::array<double, 3>& curpose) {
  franka::RobotState robot_state = robot.get_franka_robot().readOnce();
  std::array<double, 7> joint_position = robot_state.q;
  std::array<double, 7> joint_velocity = robot_state.dq;
  std::array<double, 7> applied_torque = robot_state.tau_ext_hat_filtered;
  std::array<double, 16> ee_pose = robot_state.O_T_EE_c;
  curpose[0] = ee_pose[12];
  curpose[1] = ee_pose[13];
  curpose[2] = ee_pose[14];
  double send_rate = robot_state.control_command_success_rate;
  std::string state = "s,";
  for (int i = 0; i < 7; i++) {
    state.append(std::to_string(joint_position[i]));
    state.append(",");
  }
  for (int i = 0; i < 7; i++) {
    state.append(std::to_string(joint_velocity[i]));
    state.append(",");
  }
  for (int i = 0; i < 7; i++) {
    state.append(std::to_string(applied_torque[i]));
    state.append(",");
  }
  for (int i = 0; i < 16; i++) {
    state.append(std::to_string(ee_pose[i]));
    state.append(",");
  }
  char cstr[state.size() + 1];
  std::copy(state.begin(), state.end(), cstr);
  cstr[state.size()] = '\0';
  if (steps % 50 < 1) {
    valread = read(sock, buffer, 200);
    send(sock, cstr, strlen(cstr), 0);
    if (valread > 0) {
      printf(buffer);
      std::stringstream ss(buffer);
      bool first = false;
      while (not first) {
        std::string substr;
        getline(ss, substr, ',');
        if (substr[0] == 's') {
          first = true;
        }
      }
      for (int i = 0; i < 3; i++) {
        std::string substr;
        getline(ss, substr, ',');
        double term = std::stod(substr);
        ee_dest[i] = term;
      }
    } else {
      for (int i = 0; i < 3; i++) {
        ee_dest[i] = ee_pose[12+i];
      }
    }
  }
  return ee_dest;
}

int main(int argc, char** argv) {
  if (argc != 2){
    std::cerr << "2 arguments needed" << std::endl;
  }
  int port = std::stoi(argv[1]);
  orl::Robot robot("172.16.0.2");
  VelocityController* vc = new VelocityController(port);
  std::array<double, 3> curpose;
  while (true) {
    std::array<double, 3> eepos = vc->getEE(robot, curpose);
    robot.absolute_cart_motion(curpose[0] + eepos[0], curpose[1] + eepos[1], curpose[2] + eepos[2], 20);
  }
  return 0;
}
