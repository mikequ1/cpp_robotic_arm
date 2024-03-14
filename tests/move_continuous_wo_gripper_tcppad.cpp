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
#include "TCPPad.h"

using namespace std;

/**
 * @example generate_cartesian_pose_motion.cpp
 * An example showing how to generate a Cartesian motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

static const std::vector<double> INIT_POS = {0.3, -0.02, -0.07};
static const double INIT_TIME = 10.0;
static const int S_TO_MS = 1000;
static const double MIN_Z = -0.07;
static const double MIN_DZ = -0.005;
static const double V_FACTOR = 0.05; // in m/s
// ref https://frankaemika.github.io/docs/control_parameters.html -> limits for panda
static const double MAX_SPEED = 0.05; // in m/s
static const double MAX_ACCEL = 0.05*13.0/1000.0; // delta of velocity. in m/s/tick
static const double MAX_JERK = 0.05*6500.0/1000000.0; // delta of accel. in m/s/tick/tick
static const char *DEVICE_PATH = "/dev/input/js0";


bool finished_traj(array<double, 3>& cur, array<double, 4>& dest, array<double, 3>& delta) {
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
    return true; //updated to false due to robotic arm's comms error
  return false;
}

bool reached_object(std::array<double, 3>& cur, std::array<double, 2>& obj, double radius) {
  cout << sqrt(pow(obj[0]-cur[0],2) + pow(obj[1]-cur[1],2)) << endl;
  if (sqrt(pow(obj[0]-cur[0],2) + pow(obj[1]-cur[1],2)) <= radius){
    return true;
  }
  return false;
}
double sum(std::array<double, 3> &arr) {
  double out = 0.0;
  for (int i=0; i<3; ++i) {
    out += arr[i];
  }
  return out;
}
double l2_norm(std::array<double, 3> &vec) {
  array<double, 3> vec_sq = {0.0, 0.0, 0.0};
  for (int i=0; i<3; ++i) {
    vec_sq[i] = pow(vec[i], 2);
  }
  return sqrt(sum(vec_sq));
}
double l2_distance(std::array<double, 3>& pt_a, std::array<double, 3>& pt_b) {
  array<double, 3> displacement = {0.0, 0.0, 0.0};
  for (int i=0; i<3; ++i) {
    displacement[i] = pt_b[i] - pt_a[i];
  }
  return l2_norm(displacement);
}

void sendState(std::array<double, 16>& ee_pose, int axis_x, int axis_y, int gs, int sock) {
  std::string state = "s,";
  for (int i = 0; i < 16; i++) {
    state.append(std::to_string(ee_pose[i]));
    state.append(",");
  }
  state.append(std::to_string(axis_x)); //Joystick X AXIS
  state.append(",");
  state.append(std::to_string(axis_y)); //Joystick Y AXIS
  state.append(",");
  state.append(std::to_string(gs));
  char cstr[state.size() + 1];
  std::copy(state.begin(), state.end(), cstr);
  cstr[state.size()] = '\0';
  cout << "sent state" << endl;
  send(sock, cstr, strlen(cstr), 0); // send arm state to python.
}

bool getEE(std::array<double, 4>& nextpose, std::array<double, 2>& nextObj, int sock) {
  char buffer[200] = {0};
  int valread;
  if (read(sock, buffer, 200) > 0){
    std::stringstream ss(buffer);
    bool first = false;
    while (!first) {
      std::string substr;
      getline(ss, substr, ',');
      if (substr[0] == 's') {
        first = true;
      }
    }
    // X,Y,and Z directions
    for (int i = 0; i < 3; i++) {
      std::string substr;
      getline(ss, substr, ',');
      cout << substr << endl;
      double term = std::stod(substr);
      nextpose[i] = term;
    }
    // Velocity Multiplier
    std::string substr;
    getline(ss, substr, ',');
    cout << substr << endl;
    double term = std::stod(substr);
    nextpose[3] = term;
    // Highest-scored object XY Coordinates
    for (int i = 0; i < 2; i++) {
      std::string substr;
      getline(ss, substr, ',');
      cout << substr << endl;
      double term = std::stod(substr);
      nextObj[i] = term;
    }
    return true;
  }
  return false;
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

  // if (argc != 2) {
  //   cerr << "two arguments required" << endl;
  //   return -1;
  // }
  // int port = std::stoi(argv[1]);
  int port = 53821;

  char* IP;
  if(argc > 1) {
      IP = argv[1];
  } else {
      IP = "10.0.1.8";
  }

  // Joystick Connection
  GamePad *gp = new GamePad(DEVICE_PATH);
  TCPPad *tp = new TCPPad(IP);
  gp->startThread();
  tp->startThread();

  // Connecting to the robot and resetting to home position
  orl::Robot robot("172.16.0.2");
  std::cout << "robot connected" << std::endl;
  int sock = connect2control(port);

  // Robot movement
  double time = 0.0;
  double time_goal = 0.0;
  double time_state = 0.0;

  // Queue initialization for EE coordinates
  queue<array<double, 4>> q;

  // previously: 0.17
  double reset_time = 0.24;
  double velocity_factor = V_FACTOR;
  double cur_velocity = 0;
  bool received_cmd = false;

  std::array<double, 16> pose;
  array<double, 3> delta = {0, 0, 0};
  array<double, 3> prev_delta = {0, 0, 0};
  array<double, 3> prev_accel = {0, 0, 0};
  array<double, 3> delta_unit;
  array<double, 4> goal;
  array<double, 4> next_goal;
  array<double, 3> next_delta;
  array<double, 3> next_delta_unit;
  array<double, 2> next_obj;

  // -1: default open
  // 1: gripper closed
  // 2: trying to grasp
  // 3: trying to release
  int gripper_state = -1;

  // 1: ready to start new point-to-point motion
  // 2: starting / speeding up new point-to-point motion
  // 3: moving at constant velocity
  int state = 1;
  int rst = 1;
  while (true) {
    time = 0.0;
    if (rst) {
      std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
      robot.joint_motion(q_goal, 0.2);
      robot.absolute_cart_motion(0.3, -0.023, 0.08, 3);
      robot.get_franka_gripper().homing();
      franka::GripperState gripper_read = robot.get_franka_gripper().readOnce();
      cout << "MAX gripper width is: " << gripper_read.max_width << endl;
      rst = 0;
    }
    cout << "Control loop starting" << endl;
    robot.get_franka_robot().control([=, &time, &state, &time_goal, &time_state, 
                                          &q, &delta, &prev_delta, &prev_accel, &next_delta, &delta_unit, &next_delta_unit, &next_obj,&gripper_state,
                                          &goal, &next_goal, &pose, &cur_velocity, &gp, &tp, &rst, &received_cmd](const franka::RobotState& robot_state,
                                          franka::Duration period) -> franka::CartesianVelocities {
      double delta_t = period.toSec();
      time += period.toSec();
      time_goal += period.toSec();
      time_state += period.toSec();

      int bs = gp->getButtonStateAtomic();
      int axis_x = tp->getAxisX();
      int axis_y = tp->getAxisY();
      //cout << axis_x << "," << axis_y << endl;
      array<double, 3> cur; // current position
      franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}; // xv, vy, vz, omega_x, omega_y, omega_z
      cout << endl;
      cout << "state " << state << endl;

      pose = robot_state.O_T_EE_c; // pose[12] = x, pose[13] = y, pose[14] = z
      for (int i = 0; i < 3; i++) {
        cur[i] = pose[12+i];
      }
      if (delta_t < 0.0001) {
        //return output;
        for (int i=0; i<3; ++i) {
          delta[i] = 0.0;
        }
      }
      //cout << "delta_t " << delta_t << endl;

      // State = 1 computes the unit vector for the difference (delta) between the goal and current pose
      if (state == 1) {
        // If no button is pressed, do nothing
        // If enough time hasn't elapsed since the previous decoder reading, wait
        if ((bs == 0 && axis_x == 0 && axis_y == 0)|| time_goal < 0.1){
          // for (int i=0; i<3; ++i) {
          //   delta[i] = 0.0;
          // }
          //double vx = prev;
          cout << "bs 0" << endl;
          //return output;
        } else {
          // If reset button is pressed, reset the arm to original state
          if (bs == 16) {
            rst = 1;
            // TODO: slow to zero first.
            cout << "Resetting arm..." << endl;
            return franka::MotionFinished(output);
          }

          // Fetching the next point from the decoder
          array<double, 4> next;
          sendState(pose, axis_x, axis_y, gripper_state, sock);
          while (true) {
            // read data from python/buffer
            if (getEE(next, next_obj, sock)) break;
            std::this_thread::sleep_for(std::chrono::microseconds(20));
          }
          for (int i = 0; i < 3; i++) {
            next[i] += cur[i];
          }
          q.push(next);
          if (q.size() == 0) {
            return output;
          }
          goal = q.front();
          q.pop();
          // distance to goal.
          double norm = 0;
          for (int i = 0; i < 3; i++) {
            delta[i] = goal[i] - pose[12+i];
            norm += (delta[i] * delta[i]);
          }
          

          cur_velocity = velocity_factor * goal[3];


          norm = sqrt(norm);
          for (int i = 0; i < 3; i++) {
            delta_unit[i] = delta[i] / norm; // this is a problem because it rescales all velocities. TO FIX
          }
          // If the next point z is out of bounds, we ignore the point
          // TODO: set bounds for x and y and upper bound for z.
          if ((goal[2] < MIN_Z) && (cur[2] < MIN_Z)) {
            cout << "End Effector is at z-level boundaries: goal = " << goal[2] << ", current = " << cur[2] << endl;
            time_goal = 0;
            //return output;
            for (int i=0; i<3; ++i) {
              delta[i] = 0.0;
            }
            cout << "oob" << endl;
          }
          // If the distance to the next point is too small, we ignore the point
          else if (norm <= 0.002) {
            time_goal = 0;
            //return output;
            for (int i=0; i<3; ++i) {
              delta[i] = 0.0;
            }
            cout << "too close to goal?" << endl;
          }
        }
        state = 2;
        time_state = 0;
      }
      
      if (state == 4) {
        // set velocity to zero. 
        // There's another state==4 routine later.
        for (int i=0; i<3; ++i) {
          delta[i] = 0.0;
        }
      }

      // delta corresponds to the desired velocity
      // prev_delta corresponds to the previous velocity.
      // Here we ensure that the new delta is within a radius of prev_delta
      // requires: delta, prev_delta, prev_accel
      // TODO: fix max accel and max v.
      array<double, 3> filtered_delta = {0, 0, 0};
      array<double, 3> desired_accel = {0, 0, 0}; // difference in delta. corresponds to desired accel
      array<double, 3> filtered_accel = {0, 0, 0}; // units: m/s/tick

      // filter based on Jerk.
      for (int i=0; i<3; ++i) {
        desired_accel[i] = delta[i] - prev_delta[i];
      }
      double jerk_magnitude = l2_distance(desired_accel, prev_accel);
      if (jerk_magnitude <= MAX_JERK) {
        for (int i=0; i<3; ++i) {
          filtered_accel[i] = desired_accel[i];
        }
      } else {
        double prop = MAX_JERK / jerk_magnitude;
        for (int i=0; i<3; ++i) {
          filtered_accel[i] = prop*prev_accel[i] + (1 - prop)*desired_accel[i];
        }
      }
      cout << "fa1 " << filtered_accel[0] << ", " << filtered_accel[1] << ", " << filtered_accel[2] << ", " << endl;
      double accel_magnitude = l2_norm(filtered_accel);
      if (accel_magnitude > MAX_ACCEL) {
        for (int i=0; i<3; ++i) {
          filtered_accel[i] = (MAX_ACCEL/accel_magnitude)*filtered_accel[i];
        }
      }
      cout << "fa2 " << filtered_accel[0] << ", " << filtered_accel[1] << ", " << filtered_accel[2] << ", " << endl;

      for (int i=0; i<3; ++i) {
        filtered_delta[i] = prev_delta[i] + filtered_accel[i];
        filtered_delta[i] = filtered_delta[i];
      }
      double delta_magnitude = l2_norm(filtered_delta);
      if (delta_magnitude > MAX_SPEED) {
        for (int i=0; i<3; ++i) {
          filtered_delta[i] = (MAX_SPEED/delta_magnitude)*filtered_delta[i];
        }
      }
      cout << "fd1 " << filtered_delta[0] << ", " << filtered_delta[1] << ", " << filtered_delta[2] << ", " << endl;

      cout << "delta accel " << l2_distance(filtered_accel, prev_accel) << endl;
      cout << "delta delta " << l2_distance(filtered_delta, prev_delta) << endl;



      // update cache
      for (int i=0; i<3; ++i) {
        prev_accel[i] = filtered_accel[i];
      }
      for (int i=0; i<3; ++i) {
        prev_delta[i] = filtered_delta[i];
      }

      // Set output based on filtered delta.
      // May need to modify behavior under special cases.
      double vx = filtered_delta[0];
      double vy = filtered_delta[1];
      double vz = filtered_delta[2];
      output = {{vx, vy, vz, 0.0, 0.0, 0.0}};

      cout << "         delta " << filtered_delta[0] << ", "<< filtered_delta[1] << ", " << filtered_delta[2] << endl;
      cout << "filtered delta " << filtered_delta[0] << ", "<< filtered_delta[1] << ", " << filtered_delta[2] << endl;

      printf("state: %d, %f, %f \n - current position: %f, %f, %f \n - cur goal: %f, %f, %f \n - v_multiplier: %f, Joystick: %d, Gripper: %d\n", 
            state, time_state, time, 
            cur[0], cur[1], cur[2], 
            goal[0], goal[1], goal[2], 
            goal[3], bs, gripper_state);

      // State = 2 speeds up the robot's motion based on a cosine function which ensures smooth acceleration. The robot moves to state 3 and stops accelerating after a specific time.
      if (state == 2) {
        if (time_state >= reset_time / 2) {
          state = 3;
          time_state = 0;
        } else {
          return output;
        }
      }

      // State 3: The robot moves at a constant velocity. If the robot finishes its trajectory and there are no more points in the queue, it moves to state 4. 
      // If there are more points, it proceeds to state 5 that occurs at a constant velocity.
      if (state == 3) {
        cout << "~~~~~~~~~~~STATE 3~~~~~~~~~~~" << endl;
        // Robot finishes its trajectory and there are no more points in the queue
        if (bs == 0 && axis_x == 0 && axis_y == 0){
          state = 4;
          time_state = 0;
          cout << "state 3 bs 0" << endl;
        }
        if (bs == 16) {
          rst = 1;
          state = 4;
          time_state = 0;
          cout << "Resetting arm..." << endl;
        }
        // If the gripper is open and we are close enough to an object, end control loop and engage gripper
        if (time>3.0 && (reached_object(cur, next_obj, 0.015)&& cur[2] < 0) && gripper_state == -1) {
          cout << "REACHED OBJECT for grasp" << endl;
          state = 4;
          time_state = 0;
          gripper_state = 2;
        // If the gripper is closed and we are close enough to an object, end control loop and release gripper
        } else if (time >3.0 && (reached_object(cur, next_obj, 0.015) && cur[2] < 0) && gripper_state == 1){
          cout << "REACHED OBJECT for release" << endl;
          state = 4;
          time_state = 0;
          gripper_state = 3;
        }
        // If we have reached the end of the current point-to-point movement, we begin slowing down and request
        // a new position from the python script
        if (finished_traj(cur, goal, delta)) {
          printf("Reached point %f, %f, %f \n", cur[0], cur[1], cur[2]);
          sendState(pose, axis_x, axis_y, gripper_state, sock);
          received_cmd = false;
          state = 5;
          time_state = 0;
          time_goal = 0;
        }
        // if out of bound, stop execution
        if (cur[2] < MIN_Z) {
          cout << "Ending movement: out of bounds" << endl;
          state = 4;
          time_state = 0;
        }

        /* CHANGED: limit velocities to a radius around the previous velocity. */

        double v = cur_velocity / 2.0 * (1.0 - std::cos(2.0 * M_PI * (reset_time / 2) / reset_time)); // makes velocity go at a constant value
        cout << "v: " << v << endl;
        // double vx = v * delta_unit[0];
        // double vy = v * delta_unit[1];
        // double vz = v * delta_unit[2];
        //double vx = filtered_delta[0];
        //double vy = filtered_delta[1];
        //double vz = filtered_delta[2];
        //cout << v << "| " << vx << ", " << vy << ", " << vz << endl;
        //cout << vx << ", " << vy << ", " << vz << endl;
        //output = {{vx, vy, vz, 0.0, 0.0, 0.0}};
        return output;
      }

      // Robot has finished its trajectory hence, velocity decreases
      if (state == 4) {
        if (time_state >= reset_time / 2) {
          if (gripper_state == 2 || gripper_state == 3) {
            cout << "attempting MotionFinished" << endl;

            //if (abs(vx) <= 0.01 && abs(vy) == 0.0 && abs(vz) == 0.0) {
            if (l2_norm(filtered_delta) < 0.001) {
              cout << "returning MotionFinished" << endl;
              state = 1;
              time_state = 0;
              return franka::MotionFinished(output);
            }
          } else {
            state = 1;
            time_state = 0;
          }
          cout << "@@@@@state 4, past reset@@@@@" << endl;
          return output;
        }
        return output;
      }

      // state == 5: the robot smoothly transitions between reaching one point and accelerating towards the next point.
      if (state == 5) {
        cout << "Performing state 5 if statement" << endl;
        // If we have not received back from python script, we do not keep track of new goal's time
        if (!received_cmd)
          time_goal = 0;
        // Slowing down the old velocity, unless it is done slowing down
        double v_slow = cur_velocity / 2.0 * (1.0 - cos(2.0 * M_PI  * (time_state + reset_time/2) / reset_time));
        if (time_state >= reset_time / 2)
          v_slow = 0;
        // Accelerate the new velocity, unless the new velocity hasn't been obtained yet
        double v_acc = (velocity_factor * next_goal[3]) / 2.0 * (1.0 - cos(2.0 * M_PI * time_goal / reset_time));
        if (!received_cmd)
          v_acc = 0;

        // Attempt to read from the python script
        array<double, 4> next;
        if (getEE(next, next_obj, sock)){
          received_cmd = true;
          cout << "Received Command" << endl;
          time_goal = 0;
          for (int i = 0; i < 3; i++) {
            next[i] = cur[i] + next[i];
          }
          q.push(next);
          if (q.size() == 0) {
            state = 4;
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
        
        // double vx = v_slow * delta_unit[0] + v_acc * next_delta_unit[0];
        // double vy = v_slow * delta_unit[1] + v_acc * next_delta_unit[1];
        // double vz = v_slow * delta_unit[2] + v_acc * next_delta_unit[2];

        //cout << v_slow << ", " << v_acc << "| " << vx << ", " << vy << ", " << vz << endl;

        // The new goal has completed acceleration, move to state 3 to travel at constant velocity
        if (time_goal >= reset_time / 2) {
          state = 3;
          time_state = 0;

          goal = next_goal;
          delta = next_delta;
          delta_unit = next_delta_unit;
          cur_velocity = velocity_factor * next_goal[3];
        }
        return output;
      }
      if (time >= 60) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      cout << "outputting default" << endl;
      return output;
    });
    cout << "Gripper state" << gripper_state << endl;

    if (gripper_state == 2) {
      cout << "!!attempting gripper" << endl;
      // original duration 1.2
      robot.absolute_cart_motion(next_obj[0],next_obj[1],-0.066,1.8);
      double force = 18;
      if (robot.get_franka_gripper().grasp(0.025, 0.03, force, 0.01, 0.015)){
        gripper_state = 1;
        cout << "Object Grasped" << endl;
      } else {
        gripper_state = -1;
        franka::GripperState gripper_read = robot.get_franka_gripper().readOnce();
        cout << "MAX gripper width is: " << gripper_read.max_width << endl;
        cout << "Object unable to be Grasped" << endl;
        robot.get_franka_gripper().move(gripper_read.max_width, 0.05);
      }
      // original duration 2.5
      robot.absolute_cart_motion(next_obj[0],next_obj[1],0.08,3.0);
    } else if (gripper_state == 3) {
      // original z -0.065
      robot.absolute_cart_motion(next_obj[0],next_obj[1],-0.06,1.8);
      franka::GripperState gripper_read = robot.get_franka_gripper().readOnce();
      if (robot.get_franka_gripper().move(gripper_read.max_width, 0.05)){
        gripper_state = -1;
        cout << "Object Released" << endl;
      } else {
        gripper_state = 1;
        cout << "Object unable to be Released" << endl;
      }
      robot.absolute_cart_motion(next_obj[0],next_obj[1],0.08,3.0);
    }

  }
  tp->shutdown();

  return 0;
}