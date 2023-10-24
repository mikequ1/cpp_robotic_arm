// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <math.h>
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

int main(int argc, char** argv) {
  try {
    orl::Robot robot("172.16.0.2");
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    // std::cout << "WARNING: This program will move the robot! "
    //           << "Please make sure to have the user stop button at hand!" << std::endl
    //           << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();
    robot.joint_motion(q_goal, 0.2);

    std::array<double, 16> initial_pose;
    double time = 0.0;

    robot.absolute_cart_motion(0.336, 0.023, -0.077, 3);

    queue<array<double, 3>> q;
    array<double, 3> pos1{{0.386, 0.023, 0.05}};
    array<double, 3> pos2{{0.436, 0.023, 0}};
    q.push(pos1);
    q.push(pos2);
    // robot.absolute_cart_motion(pos1[0], pos1[1], pos1[2], 3);
    // robot.absolute_cart_motion(pos2[0], pos2[1], pos2[2], 3);

    double max_time = 1;
    array<double, 3> delta;
    array<double, 3> accumulated_delta = {{0,0,0}};
    robot.get_franka_robot().control([=, &time, &initial_pose, &q, &delta, &accumulated_delta](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();

      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE;
        array<double, 3> dest = q.front();
        q.pop();
        for (int i = 0; i < 3; i++) {
          delta[i] = dest[i] - initial_pose[12+i];
        }
      } 

      std::array<double, 16> new_pose = robot_state.O_T_EE_d;
    
      double t = time/max_time;
      double speed_factor = (1 - std::cos(M_PI * t)) / 2.0;
      long double dist_factor = (1.0/2) * (t - M_1_PI * sin(M_PI * t));

      long double next_x = delta[0] * dist_factor;
      long double next_y = delta[1] * dist_factor;
      long double next_z = delta[2] * dist_factor;

      new_pose[12] += next_x - accumulated_delta[0];
      new_pose[13] += next_y - accumulated_delta[1];
      new_pose[14] += next_z - accumulated_delta[2];

      cout << dist_factor << endl;
      cout << t << " | diff | " << next_x - accumulated_delta[0] << ", " << next_y - accumulated_delta[1] << ", " << next_z - accumulated_delta[2] <<  endl;

      accumulated_delta[0] = next_x;
      accumulated_delta[1] = next_y;
      accumulated_delta[2] = next_z;

      cout << t << " | new pose | " << new_pose[12] << ", " << new_pose[13] << ", " << new_pose[14] <<  endl;
    

      if (time >= 10.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        array<double, 16> end_pose = robot_state.O_T_EE;
        cout << end_pose[12] << ", " << end_pose[13] << ", " << end_pose[14] <<  endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
