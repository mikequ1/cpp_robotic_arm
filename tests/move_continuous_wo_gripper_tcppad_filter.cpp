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
static const double MAX_SPEED = 0.05; // in m/s
static const double MAX_ACCEL = 0.001; // derivative of velocity.
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
  array<double, 3> delta_unit;
  array<double, 4> goal;
  array<double, 4> next_goal;
  array<double, 3> next_delta;
  array<double, 3> next_delta_unit;
  array<double, 2> next_obj;
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
                                          &q, &delta, &prev_delta, &next_delta, &delta_unit, &next_delta_unit, &next_obj,&gripper_state,
                                          &goal, &next_goal, &pose, &cur_velocity, &gp, &tp, &rst, &received_cmd](const franka::RobotState& robot_state,
                                          franka::Duration period) -> franka::CartesianVelocities {
      time += period.toSec();
      time_goal += period.toSec();
      time_state += period.toSec();

      int bs = gp->getButtonStateAtomic();
      int axis_x = tp->getAxisX();
      int axis_y = tp->getAxisY();
      //cout << axis_x << "," << axis_y << endl;
      array<double, 3> cur; // current position
      franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}; // xv, vy, vz, omega_x, omega_y, omega_z

      pose = robot_state.O_T_EE_c; // pose[12] = x, pose[13] = y, pose[14] = z
      for (int i = 0; i < 3; i++) {
        cur[i] = pose[12+i];
      }
      // State = 1 computes the unit vector for the difference (delta) between the goal and current pose
      if (state == 1) {
        // If no button is pressed, do nothing
        // If enough time hasn't elapsed since the previous decoder reading, wait
        if ((bs == 0 && axis_x == 0 && axis_y == 0)|| time_goal < 0.1){
          return output;
        }
        // If reset button is pressed, reset the arm to original state
        if (bs == 16) {
          rst = 1;
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
          return output;
        }
        // If the distance to the next point is too small, we ignore the point
        if (norm <= 0.002) {
          time_goal = 0;
          return output;
        }
        state = 2;
        time_state = 0;
      }

      // delta corresponds to the desired velocity
      // prev_delta corresponds to the previous velocity.
      // Here we ensure that the new delta is within a radius of prev_delta
      // requires: delta and prev_delta
      array<double, 3> filtered_delta = {0, 0, 0};
      array<double, 3> ddelta = {0, 0, 0}; // difference in delta
      for (int i=0; i<3; ++i) {
        ddelta[i] = delta[i] - prev_delta[i];
      }
      double ddelta_magnitude = l2_norm(ddelta);
      if (ddelta_magnitude <= MAX_ACCEL) {
        for (int i=0; i<3; ++i) {
          filtered_delta[i] = delta[i];
        }
      } else {
        double prop = MAX_ACCEL / ddelta_magnitude; // proportion corresponding to prev_delta
        for (int i=0; i<3; ++i) {
          filtered_delta[i] = prop*prev_delta[i] + (1 - prop)*delta[i];
        }
      }
      for (int i=0; i<3; ++i) {
        prev_delta[i] = delta[i];
        filtered_delta[i] = 0.4*filtered_delta[i]; //todo: use goal[3] (i.e. velocity multiplier)
      }

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
          /* CHANGED: limit velocities to a radius around the previous velocity. */

          //double v = cur_velocity / 2.0 * (1.0 - std::cos(2.0 * M_PI * time_state / reset_time)); // changed to 2pi since we want peak velocity to be at 0.5 progress instead of 1.0
          //double vx = v * delta_unit[0];
          //double vy = v * delta_unit[1];
          //double vz = v * delta_unit[2];
          double vx = filtered_delta[0];
          double vy = filtered_delta[1];
          double vz = filtered_delta[2];
          output = {{vx, vy, vz, 0.0, 0.0, 0.0}};
          return output;
        }
      }

      // State 3: The robot moves at a constant velocity. If the robot finishes its trajectory and there are no more points in the queue, it moves to state 4. 
      // If there are more points, it proceeds to state 5 that occurs at a constant velocity.
      if (state == 3) {
        // Robot finishes its trajectory and there are no more points in the queue
        if (bs == 0 && axis_x == 0 && axis_y == 0){
          state = 4;
          time_state = 0;
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

        // double v = cur_velocity / 2.0 * (1.0 - std::cos(2.0 * M_PI * (reset_time / 2) / reset_time)); // makes velocity go at a constant value
        // double vx = v * delta_unit[0];
        // double vy = v * delta_unit[1];
        // double vz = v * delta_unit[2];
        double vx = filtered_delta[0];
        double vy = filtered_delta[1];
        double vz = filtered_delta[2];
        //cout << v << "| " << vx << ", " << vy << ", " << vz << endl;
        cout << vx << ", " << vy << ", " << vz << endl;
        output = {{vx, vy, vz, 0.0, 0.0, 0.0}};
        return output;
      }

      // Robot has finished its trajectory hence, velocity decreases
      if (state == 4) {
        if (time_state >= reset_time / 2) {
          if (gripper_state == 2 || gripper_state == 3) {
            return franka::MotionFinished(output);
          }
          state = 1;
          time_state = 0;
          return output;
        }
        // Adding the phase shift of reset_time/2, starting the function at the point where the velocity would be at its maximum.
        // The robot would start slowing down immediately, ensuring a smooth deceleration until it stops
        // double v = cur_velocity / 2.0 * (1.0 - cos(2.0 * M_PI * (time_state + reset_time/2) / reset_time));
        // double vx = v * delta_unit[0];
        // double vy = v * delta_unit[1];
        // double vz = v * delta_unit[2];
        double vx = filtered_delta[0];
        double vy = filtered_delta[1];
        double vz = filtered_delta[2];
        output = {{vx, vy, vz, 0.0, 0.0, 0.0}};

        return output;
      }

      // state = 5: the robot smoothly transitions between reaching one point and accelerating towards the next point.
      if (state == 5) {
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

        double vx = filtered_delta[0];
        double vy = filtered_delta[1];
        double vz = filtered_delta[2];

        output = {{vx, vy, vz, 0.0, 0.0, 0.0}};
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
      return output;
    });


    if (gripper_state == 2) {
      robot.absolute_cart_motion(next_obj[0],next_obj[1],-0.066,1.2);
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
      robot.absolute_cart_motion(next_obj[0],next_obj[1],0.08,2.5);
    } else if (gripper_state == 3) {
      robot.absolute_cart_motion(next_obj[0],next_obj[1],-0.065,1.2);
      franka::GripperState gripper_read = robot.get_franka_gripper().readOnce();
      if (robot.get_franka_gripper().move(gripper_read.max_width, 0.05)){
        gripper_state = -1;
        cout << "Object Released" << endl;
      } else {
        gripper_state = 1;
        cout << "Object unable to be Released" << endl;
      }
      robot.absolute_cart_motion(next_obj[0],next_obj[1],0.08,2.5);
    }

  }
  tp->shutdown();

  return 0;
}