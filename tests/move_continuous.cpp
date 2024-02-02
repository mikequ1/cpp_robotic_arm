// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <vector>
#include <queue>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <linux/joystick.h>

#include "franka/exception.h"
#include <franka/robot.h>
#include <franka/model.h>

#include <liborl/liborl.h>

#include "GamePad.h"

using namespace std;

/**
 * @example generate_cartesian_pose_motion.cpp
 * An example showing how to generate a Cartesian motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

static const std::vector<double> INIT_POS = {0.335, -0.02, -0.07};
static const double INIT_TIME = 10.0;
static const int S_TO_MS = 1000;
static const double MIN_Z = -0.075;
static const double MIN_DZ = -0.005;
static const double V_FACTOR = 0.05; // in m/s
static const char *DEVICE_PATH = "/dev/input/js0";


bool finished_traj(array<double, 3>& cur, array<double, 3>& dest, array<double, 3>& delta) {
  /**
   * Determines whether a trajectory in 3D space is finished.
   *
   * @param cur The current position in 3D space.
   * @param dest The destination position in 3D space.
   * @param delta The direction and magnitude of movement in each dimension.
   *
   * @return true if the current position has reached or surpassed the destination 
   * based on the movement direction for each dimension. false otherwise.
   *
   * @note The direction of movement in each dimension is determined by the sign of 
   * the corresponding value in the delta array. Positive indicates forward movement,
   * and negative indicates backward movement.
   */
  int x = delta[0] > 0 ? 1:0;
  int y = delta[1] > 0 ? 1:0;
  int z = delta[2] > 0 ? 1:0;

  x = abs(delta[0]) < 0.005 ? -1:x;
  y = abs(delta[1]) < 0.005 ? -1:y;
  z = abs(delta[2]) < 0.005 ? -1:z;

  int res = 0;
  if ((x == -1) || ((cur[0] > dest[0]) && x == 1) || ((cur[0] < dest[0]) && x == 0))
    res += 1;
  if ((y == -1) || ((cur[1] > dest[1]) && y == 1) || ((cur[1] < dest[1]) && y == 0))
    res += 1;
  if ((z == -1) || ((cur[2] > dest[2]) && z == 1) || ((cur[2] < dest[2]) && z == 0))
    res += 1;
  if (res == 3)
    return true;
  return false;
}


void getEE(std::array<double, 16>& ee_pose, std::array<double, 3>& nextpose, int sock, int bs, int& gw) {
  std::string state = "s,";
  for (int i = 0; i < 16; i++) {
    state.append(std::to_string(ee_pose[i]));
    state.append(",");
  }
  state.append(std::to_string(bs));
  state.append(",");
  char cstr[state.size() + 1];
  std::copy(state.begin(), state.end(), cstr);
  cstr[state.size()] = '\0';
  char buffer[200] = {0};
  cout << "sent state" << endl;
  send(sock, cstr, strlen(cstr), 0);

  int valread;
  while (true) {
    valread = read(sock, buffer, 200);
    std::this_thread::sleep_for(std::chrono::microseconds(50));
    if (valread > 0) break;
  }

  std::stringstream ss(buffer);
  std::string substr;
  double term;
  bool first = false;
  while (!first) {
    getline(ss, substr, ',');
    if (substr[0] == 's') {
      first = true;
    }
  }
  for (int i = 0; i < 3; i++) {
    getline(ss, substr, ',');
    term = std::stod(substr);
    nextpose[i] = term;
  }
  getline(ss, substr, ',');
  term = std::stod(substr);
  gw = term;
}


int connect2control(int PORT) {
  int sock = 0;
  struct sockaddr_in serv_addr;
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    std::cout << "Socket Creation Error!" << std::endl;
  }
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT);
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

  return sock;
}

int main(int argc, char** argv) {
  // Robot Connection
  if (argc != 2) {
    cerr << "two arguments required" << endl;
    return -1;
  }
  int port = std::stoi(argv[1]);

  // Joystick Connection
  GamePad *gp = new GamePad(DEVICE_PATH);
  gp->startThread();

  // Connecting to the robot and resetting to home position
  orl::Robot robot("172.16.0.2");
  std::cout << "robot connected" << std::endl;
  int sock = connect2control(port);

  // Robot movement
  double time = 0.0;
  double time_goal = 0.0;
  double time_state = 0.0;

  // Queue initialization for EE coordinates
  queue<array<double, 3>> q;

  double reset_time = 0.2;
  double velocity_factor = V_FACTOR;

  std::array<double, 16> pose;
  array<double, 3> delta;
  array<double, 3> delta_unit;
  array<double, 3> goal;
  array<double, 3> next_goal;
  array<double, 3> next_delta;
  array<double, 3> next_delta_unit;

  int state = 1;
  int rst = 1;
  int gw;
  while (true) {
    if (rst) {
      std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
      robot.joint_motion(q_goal, 0.2);
      robot.absolute_cart_motion(0.336, -0.023, -0.077, 3);
      rst = 0;
    } 

    robot.get_franka_robot().control([=, &time, &time_goal, &time_state, &state, &gw,
                                          &q, &delta, &next_delta, &delta_unit, &next_delta_unit,
                                          &goal, &next_goal, &pose, &gp, &rst](const franka::RobotState& robot_state,
                                          franka::Duration period) -> franka::CartesianVelocities {
      time += period.toSec();
      time_goal += period.toSec();
      time_state += period.toSec();

      int bs = gp->getButtonStateAtomic();
      array<double, 3> cur;
      franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

      pose = robot_state.O_T_EE_c; // pose[12] = x, pose[13] = y, pose[14] = z
      for (int i = 0; i < 3; i++) {
        cur[i] = pose[12+i];
      }
      // State = 1 computes the unit vector for the difference (delta) between the goal and current pose
      if (state == 1) {
        // If no button is pressed, do nothing
        // If enough time hasn't elapsed since the previous decoder reading, wait
        if (bs == 0 || time_goal < 0.1){
          return output;
        }
        // If reset button is pressed, reset the arm to original state
        if (bs == 16) {
          rst = 1;
          cout << "Resetting arm..." << endl;
          return franka::MotionFinished(output);
        }

        // Fetching the next point from the decoder
        array<double, 3> next;
        getEE(pose, next, sock, bs, gw);
        for (int i = 0; i < 3; i++) {
          next[i] += cur[i];
        }
        q.push(next);
        if (q.size() == 0) {
          return output;
        }
        goal = q.front();
        q.pop();
        double norm = 0;
        for (int i = 0; i < 3; i++) {
          delta[i] = goal[i] - pose[12+i];
          norm += (delta[i] * delta[i]);
        }
        norm = sqrt(norm);
        for (int i = 0; i < 3; i++) {
          delta_unit[i] = delta[i] / norm;
        }


        // If the next point is out of bounds, we ignore the point
        if ((goal[2] < MIN_Z) && (cur[2] < MIN_Z)) {
          cout << "End Effector is at z-level boundaries: goal = " << goal[2] << ", current = " << cur[2] << endl;
          time_goal = 0;
          return output;
        }
        // If the distance to the next point is too small, we ignore the point
        if (norm <= 0.01) {
          time_goal = 0;
          return output;
        }
        state = 2;
        time_state = 0;
      }

      printf("state: %d, %f \n - current position: %f, %f, %f \n - cur goal: %f, %f, %f \n - Joystick: %d | Gripper Width: %d\n", 
            state, time_state, cur[0], cur[1], cur[2], goal[0], goal[1], goal[2], bs, gw);

      // State = 2 speeds up the robot's motion based on a cosine function which ensures smooth acceleration. The robot moves to state 3 and stops accelerating after a specific time.
      if (state == 2) {
        if (time_state >= reset_time / 2) {
          state = 3;
          time_state = 0;
        } else {
          double v = velocity_factor / 2.0 * (1.0 - std::cos(2.0 * M_PI * time_state / reset_time)); // changed to 2pi since we want peak velocity to be at 0.5 progress instead of 1.0
          double vx = v * delta_unit[0];
          double vy = v * delta_unit[1];
          double vz = v * delta_unit[2];
          output = {{vx, vy, vz, 0.0, 0.0, 0.0}};
          return output;
        }
      }

      // State 3: The robot moves at a constant velocity. If the robot finishes its trajectory and there are no more points in the queue, it moves to state 4. 
      // If there are more points, it proceeds to state 5 that occurs at a constant velocity.
      if (state == 3) {
        // Robot finishes its trajectory and there are no more points in the queue
        if (bs == 0){
          state = 4;
          time_state = 0;
        }
        if (finished_traj(cur, goal, delta)) {
          printf("Reached point %f, %f, %f \n", cur[0], cur[1], cur[2]);
          array<double, 3> next;
          getEE(pose, next, sock, bs, gw);
          for (int i = 0; i < 3; i++) {
            next[i] = cur[i] + next[i];
          }
          q.push(next);
          if (q.size() == 0) {
            state = 4;
            time_state = 0;
          } else {
            state = 5;
            time_state = 0;
            next_goal = q.front();
            q.pop();

            // Adjusts the computed direction and distance to the next goal by accounting for anticipated motion 
            // (0.05 * reset_time * delta_unit[i]) in the robot's current movement direction over a specified time duration.
            double norm = 0;
            for (int i = 0; i < 3; i++) {
              next_delta[i] = next_goal[i] - pose[12+i] - (0.05 * reset_time * delta_unit[i]);
              norm += (next_delta[i] * next_delta[i]);
            }
            norm = sqrt(norm);
            for (int i = 0; i < 3; i++) {
              next_delta_unit[i] = next_delta[i] / norm;
            }
          }
        }
        // if out of bound, stop execution
        if ((goal[2] < MIN_Z) && (cur[2] < MIN_Z) && (delta[2] < MIN_DZ)) {
          cout << "Ending movement: out of bounds" << endl;
          state = 4;
          time_state = 0;
        }
        double v = velocity_factor / 2.0 * (1.0 - std::cos(2.0 * M_PI * (reset_time / 2) / reset_time)); // makes velocity go at a constant value
        double vx = v * delta_unit[0];
        double vy = v * delta_unit[1];
        double vz = v * delta_unit[2];
        output = {{vx, vy, vz, 0.0, 0.0, 0.0}};
        return output;
      }

      // Robot has finished its trajectory hence, velocity decreases
      if (state == 4) {
        if (time_state >= reset_time / 2) {
          state = 1;
          time_state = 0;
          return output;
        }
        // Adding the phase shift of reset_time/2, starting the function at the point where the velocity would be at its maximum.
        // The robot would start slowing down immediately, ensuring a smooth deceleration until it stops
        double v = velocity_factor / 2.0 * (1.0 - cos(2.0 * M_PI * (time_state + reset_time/2) / reset_time));
        double vx = v * delta_unit[0];
        double vy = v * delta_unit[1];
        double vz = v * delta_unit[2];
        output = {{vx, vy, vz, 0.0, 0.0, 0.0}};

        return output;
      }

      // state = 5: the robot smoothly transitions between reaching one point and accelerating towards the next point.
      if (state == 5) {
        if (time_state >= reset_time / 2) {
          state = 3;
          time_state = 0;

          goal = next_goal;
          delta = next_delta;
          delta_unit = next_delta_unit;
        }
        double v_slow = velocity_factor / 2.0 * (1.0 - cos(2.0 * M_PI  * (time_state + reset_time/2) / reset_time));
        double v_acc = velocity_factor / 2.0 * (1.0 - cos(2.0 * M_PI * time_state / reset_time));
        double vx = v_slow * delta_unit[0] + v_acc * next_delta_unit[0];
        double vy = v_slow * delta_unit[1] + v_acc * next_delta_unit[1];
        double vz = v_slow * delta_unit[2] + v_acc * next_delta_unit[2];

        output = {{vx, vy, vz, 0.0, 0.0, 0.0}};
        return output;
      }

      if (time >= 60) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    });
  }

  return 0;
}