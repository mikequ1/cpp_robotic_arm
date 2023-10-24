// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <vector>
#include <queue>

#include "franka/exception.h"
#include <franka/robot.h>
#include <franka/model.h>

#include <liborl/liborl.h>

#include "examples_common.h"

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

bool finished_traj(array<double, 3>& cur, array<double, 3>& dest, array<double, 3>& delta) {
  int x = delta[0] > 0 ? 1:0;
  int y = delta[1] > 0 ? 1:0;
  int z = delta[2] > 0 ? 1:0;
  int res = 0;
  if (((cur[0] > dest[0]) && x == 1) || ((cur[0] < dest[0]) && x == 0))
    res += 1;
  if (((cur[1] > dest[1]) && y == 1) || ((cur[1] < dest[1]) && y == 0))
    res += 1;
  if (((cur[2] > dest[2]) && z == 1) || ((cur[2] < dest[2]) && z == 0))
    res += 1;
  if (res == 3)
    return true;
  return false;
}

bool test(){
  return true;
}

int main() {
  try {
    orl::Robot robot("172.16.0.2");
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    // std::cout << "WARNING: This program will move the robot! "
    //           << "Please make sure to have the user stop button at hand!" << std::endl
    //           << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();
    robot.joint_motion(q_goal, 0.2);

    double time = 0.0;
    double time_goal = 0.0;
    double time_state = 0.0;

    robot.absolute_cart_motion(0.336, 0.023, -0.077, 3);

    queue<array<double, 3>> q;
    array<double, 3> pos1{{0.436, 0.023, 0.1}};
    array<double, 3> pos2{{0.536, 0.023, -0.05}};
    array<double, 3> pos3{{0.636, 0.023, 0.1}};
    array<double, 3> pos4{{0.736, 0.023, -0.05}};
    q.push(pos1);
    q.push(pos2);
    q.push(pos3);
    q.push(pos4);

    double reset_time = 0.3;
    double velocity_factor = 0.1;

    std::array<double, 16> pose;
    array<double, 3> delta;
    array<double, 3> delta_unit;
    array<double, 3> goal;
    array<double, 3> next_goal;
    array<double, 3> next_delta;
    array<double, 3> next_delta_unit;

    // 1: ready to start new point-to-point motion
    // 2: starting / speeding up new point-to-point motion
    // 3: moving at constant velocity
    int state = 1;
    robot.get_franka_robot().control([=, &time, &state, &time_goal, &time_state, 
                                          &q, &delta, &next_delta, &delta_unit, &next_delta_unit,
                                          &goal, &next_goal, &pose](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianVelocities {
      time += period.toSec();
      time_goal += period.toSec();
      time_state += period.toSec();

      array<double, 3> cur;
      franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

      pose = robot_state.O_T_EE_c;
      for (int i = 0; i < 3; i++) {
        cur[i] = pose[12+i];
      }

      if (state == 1) {
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
        for (int i = 0; i < delta.size(); i++) {
          delta_unit[i] = delta[i] / norm;
        }
        state = 2;
        time_goal = 0;
        time_state = 0;
      }

      // printf("state: %d, %f - current position: %f, %f, %f \n", state, time_state, cur[0], cur[1], cur[2]);
      if (state == 2) {
        if (time_state >= reset_time / 2) {
          state = 3;
          time_state = 0;
        } else {
          double v = velocity_factor / 2.0 * (1.0 - std::cos(2.0 * M_PI / reset_time * time_state));
          double vx = v * delta_unit[0];
          double vy = v * delta_unit[1];
          double vz = v * delta_unit[2];
          output = {{vx, vy, vz, 0.0, 0.0, 0.0}};
          return output;
        }
      }

      if (state == 3) {
        if (finished_traj(cur, goal, delta) && q.size() == 0) {
          state = 4;
          time_state = 0;
          printf("Reached point %f, %f, %f \n", cur[0], cur[1], cur[2]);
        } else if (finished_traj(cur, goal, delta) && q.size() != 0) {
          printf("Reached point %f, %f, %f \n", cur[0], cur[1], cur[2]);
          state = 5;
          time_state = 0;
          next_goal = q.front();
          q.pop();

          double norm = 0;
          for (int i = 0; i < 3; i++) {
            next_delta[i] = next_goal[i] - pose[12+i] - (0.05 * reset_time * delta_unit[i]);
            norm += (next_delta[i] * next_delta[i]);
          }
          norm = sqrt(norm);
          for (int i = 0; i < delta.size(); i++) {
            next_delta_unit[i] = next_delta[i] / norm;
          }
        }
        double v = velocity_factor / 2.0 * (1.0 - std::cos(2.0 * M_PI / reset_time * (reset_time / 2)));
        double vx = v * delta_unit[0];
        double vy = v * delta_unit[1];
        double vz = v * delta_unit[2];
        output = {{vx, vy, vz, 0.0, 0.0, 0.0}};
        return output;
      }

      if (state == 4) {
        if (time_state >= reset_time / 2) {
          state = 0;
          time_state = 0;
          return output;
        }
        double v = velocity_factor / 2.0 * (1.0 - cos(2.0 * M_PI / reset_time * (time_state + reset_time/2)));
        double vx = v * delta_unit[0];
        double vy = v * delta_unit[1];
        double vz = v * delta_unit[2];
        output = {{vx, vy, vz, 0.0, 0.0, 0.0}};

        // cout << "velocity_factor " <<  velocity_factor << endl;
        // cout << "reset_time " <<  reset_time << endl;
        // cout << "time_state " <<  time_state << endl;
        // cout << "reset_time " <<  reset_time << endl;

        return output;
      }

      if (state == 5) {
        if (time_state >= reset_time / 2) {
          state = 3;
          time_state = 0;

          goal = next_goal;
          delta = next_delta;
          delta_unit = next_delta_unit;
        }
        double v_slow = velocity_factor / 2.0 * (1.0 - cos(2.0 * M_PI / reset_time * (time_state + reset_time/2)));
        double v_acc = velocity_factor / 2.0 * (1.0 - cos(2.0 * M_PI / reset_time * (time_state)));
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
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
