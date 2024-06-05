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
#include <franka/rate_limiting.h>

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
static const double MAX_SPEED = 0.05;                     // in m/s
static const double MAX_ACCEL = 0.05 * 13.0 / 1000.0;     // delta of velocity. in m/s/tick
static const double MAX_JERK = 0.05 * 6500.0 / 1000000.0; // delta of accel. in m/s/tick/tick
// constants for limitRate
static const double LIMITS_SCALE = 0.1;
static const double FRANKA_MAX_VEL = LIMITS_SCALE * 1.7;
static const double FRANKA_MAX_ACCEL = LIMITS_SCALE * 13.0;
static const double FRANKA_MAX_JERK = LIMITS_SCALE * 6500.0;
static const double FRANKA_MAX_ROT_VEL = LIMITS_SCALE * 2.5;
static const double FRANKA_MAX_ROT_ACCEL = LIMITS_SCALE * 25.0;
static const double FRANKA_MAX_ROT_JERK = LIMITS_SCALE * 12500.0;

static const double GRASP_RADIUS = 0.0254;
static const double GRASP_COOLDOWN = 5.0;

array<double, 6> ROBOT_LIMITS_POLY_X = {0.3, 0.6, 0.75, 0.75, 0.6, 0.3}; // 0.7
array<double, 6> ROBOT_LIMITS_POLY_Y = {-0.31, -0.31, -0.15, 0.15, 0.31, 0.31};
// array<double, 6> ROBOT_LIMITS_POLY_X = {0.3, 0.6, 0.7, 0.7, 0.6, 0.3};
// array<double, 6> ROBOT_LIMITS_POLY_Y = {-0.2, -0.2, -0.15, 0.15, 0.20, 0.2};

static const char *DEVICE_PATH = "/dev/input/js0";

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename T>T clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

bool finished_traj(array<double, 3> &cur, array<double, 4> &dest, array<double, 3> &delta)
{
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
  int x = delta[0] > 0 ? 1 : 0;
  int y = delta[1] > 0 ? 1 : 0;
  int z = delta[2] > 0 ? 1 : 0;

  x = abs(delta[0]) < 0.005 ? -1 : x;
  y = abs(delta[1]) < 0.005 ? -1 : y;
  z = abs(delta[2]) < 0.005 ? -1 : z;

  int res = 0;
  if ((x == -1) || ((cur[0] > dest[0]) && x == 1) || ((cur[0] < dest[0]) && x == 0))
    res += 1;
  if ((y == -1) || ((cur[1] > dest[1]) && y == 1) || ((cur[1] < dest[1]) && y == 0))
    res += 1;
  if ((z == -1) || ((cur[2] > dest[2]) && z == 1) || ((cur[2] < dest[2]) && z == 0))
    res += 1;
  if (res == 3)
    return true; // updated to false due to robotic arm's comms error
  return false;
}

bool reached_object(std::array<double, 3> &cur, std::array<double, 2> &obj, double radius)
{
  //cout << sqrt(pow(obj[0] - cur[0], 2) + pow(obj[1] - cur[1], 2)) << endl;
  if (sqrt(pow(obj[0] - cur[0], 2) + pow(obj[1] - cur[1], 2)) <= radius)
  {
    return true;
  }
  return false;
}
double sum(std::array<double, 3> &arr)
{
  double out = 0.0;
  for (int i = 0; i < 3; ++i)
  {
    out += arr[i];
  }
  return out;
}
double l2_norm(std::array<double, 3> &vec)
{
  array<double, 3> vec_sq = {0.0, 0.0, 0.0};
  for (int i = 0; i < 3; ++i)
  {
    vec_sq[i] = pow(vec[i], 2);
  }
  return sqrt(sum(vec_sq));
}
double l2_norm(double *vec, int length)
{
  double accum = 0;
  for (int i = 0; i < length; ++i)
  {
    accum += pow(vec[i], 2);
  }
  return sqrt(accum);
}
double l2_distance(std::array<double, 3> &pt_a, std::array<double, 3> &pt_b)
{
  array<double, 3> displacement = {0.0, 0.0, 0.0};
  for (int i = 0; i < 3; ++i)
  {
    displacement[i] = pt_b[i] - pt_a[i];
  }
  return l2_norm(displacement);
}

bool is_within_2D_polygon(std::array<double, 2> pt, double *verts_x, double *verts_y, size_t n_verts)
{
  int winding_number = 0;
  for (int i = 0; i < n_verts; ++i)
  {
    int j = (i + 1) % n_verts;
    double v1x = verts_x[i];
    double v1y = verts_y[i];
    double v2x = verts_x[j];
    double v2y = verts_y[j];

    if (v1y == v2y)
    {
      continue;
    }
    double prop = (pt[1] - v1y) / (v2y - v1y);
    double delta = (prop >= 0) && (prop <= 1);
    delta = delta * ((v1x + prop * (v2x - v1x)) >= pt[0]); // 1 if the (positive) ray crosses the segment, 0 otherwise.
    double tmp1 = (v1y >= pt[1]) * (v2y < pt[1]);          // enter case
    double tmp2 = (v1y < pt[1]) * (v2y >= pt[1]);          // exit case
    winding_number += delta * (-1 * tmp1 + 1 * tmp2);
  }
  return (winding_number % 2) == 1;
}

// Projects a point onto the closest point along the polygon edges and returns the distance.
double project_onto_2D_polygon(std::array<double, 2> pt, double *verts_x, double *verts_y, size_t n_verts, std::array<double, 2> &out)
{
  if (n_verts == 0)
  {
    for (int i = 0; i < 2; ++i)
    {
      out[i] = pt[i];
    }
    return 0.0;
  }

  double min_dist = __DBL_MAX__;
  std::array<double, 2> closest_point = {__DBL_MAX__, __DBL_MAX__};
  for (int i = 0; i < n_verts; ++i)
  {
    //cout << endl
    //     << "clp" << closest_point[0] << "<" << closest_point[1] << "md " << min_dist << " | ";
    int j = (i + 1) % n_verts;
    double v1x = verts_x[i];
    double v1y = verts_y[i];
    double v2x = verts_x[j];
    double v2y = verts_y[j];

    // degenerate case
    if (v1x == v2x && v1y == v2y)
    {
      double dist = sqrt(pow(v1x - pt[0], 2) + pow(v1y - pt[1], 2));
      if (dist < min_dist)
      {
        min_dist = dist;
        closest_point[0] = v1x;
        closest_point[1] = v1y;
        //cout << dist << "a ";
      }
      continue;
    }

    // nondegenerate
    double ux = (v2x - v1x);                      // x of vector from v1 to v2
    double uy = (v2y - v1y);                      // y of vector from v1 to v2
    double unorm = sqrt(pow(ux, 2) + pow(uy, 2)); // l2 norm
    ux = ux / unorm;
    uy = uy / unorm;
    double uperpx = uy;
    double uperpy = -ux;

    double proj_prop = ((ux * pt[0] + uy * pt[1]) - (ux * v1x + uy * v1y)) / ((ux * v2x + uy * v2y) - (ux * v1x + uy * v1y));
    //cout << "pp |" << proj_prop << "||" << ux << "|" << uy << "||" << pt[0] << "|" << pt[1] << "||" << v1x << "|" << v1y << "||";

    // case where the closest point is one of the vertices
    if (proj_prop <= 0 || proj_prop >= 1)
    {
      double v1_dist = sqrt(pow(pt[0] - v1x, 2) + pow(pt[1] - v1y, 2));
      if (v1_dist < min_dist)
      {
        min_dist = v1_dist;
        closest_point[0] = v1x;
        closest_point[1] = v1y;
        //cout << v1_dist << "b ";
      }
      double v2_dist = sqrt(pow(pt[0] - v2x, 2) + pow(pt[1] - v2y, 2));
      if (v2_dist < min_dist)
      {
        min_dist = v2_dist;
        closest_point[0] = v2x;
        closest_point[1] = v2y;
        //cout << v2_dist << "c ";
      }
      continue;
    }

    // case where the closest point is in between the vertices
    double dist_to_line = (uperpx * pt[0] + uperpy * pt[1]) - (uperpx * v1x + uperpy * v1y); // signed distance in the direction of uperp

    double projx = pt[0] - dist_to_line * uperpx;
    double projy = pt[1] - dist_to_line * uperpy;
    //cout << "tmp" << (uperpx * projx + uperpy * projy) - (uperpx * v1x + uperpy * v1y) << "^";
    if (abs(dist_to_line) < min_dist)
    {
      min_dist = abs(dist_to_line);
      closest_point[0] = projx;
      closest_point[1] = projy;
      //cout << dist_to_line << "d ";
    }
  }

  out[0] = closest_point[0];
  out[1] = closest_point[1];
  //cout << "||out" << out[0] << ", " << out[1] << endl;
  return min_dist;
}

void sendState(std::array<double, 16> &ee_pose, int axis_x, int axis_y, int gs, int sock)
{
  std::string state = "s,";
  for (int i = 0; i < 16; i++)
  {
    state.append(std::to_string(ee_pose[i]));
    state.append(",");
  }
  state.append(std::to_string(axis_x)); // Joystick X AXIS
  state.append(",");
  state.append(std::to_string(axis_y)); // Joystick Y AXIS
  state.append(",");
  state.append(std::to_string(gs));
  char cstr[state.size() + 1];
  std::copy(state.begin(), state.end(), cstr);
  cstr[state.size()] = '\0';
  cout << "sent state" << endl;
  send(sock, cstr, strlen(cstr), 0); // send arm state to python.
}
void sendState(std::array<double, 16> &ee_pose, int axis_x, int axis_y, int gs, int rst, int sock)
{
  std::string state = "s,";
  for (int i = 0; i < 16; i++)
  {
    state.append(std::to_string(ee_pose[i]));
    state.append(",");
  }
  state.append(std::to_string(axis_x)); // Joystick X AXIS
  state.append(",");
  state.append(std::to_string(axis_y)); // Joystick Y AXIS
  state.append(",");
  state.append(std::to_string(gs));
  state.append(",");
  state.append(std::to_string(rst));
  char cstr[state.size() + 1];
  std::copy(state.begin(), state.end(), cstr);
  cstr[state.size()] = '\0';
  cout << "sent state" << endl;
  send(sock, cstr, strlen(cstr), 0); // send arm state to python.
}
void sendState(std::array<double, 16> &ee_pose, int axis_x, int axis_y, int gs, int rst, int start, int a_press, int b_press, 
              int gs_ret, int gs_at_ret, int zs, double zf, int sock)
{
  std::string state = "s,";
  for (int i = 0; i < 16; i++)
  {
    state.append(std::to_string(ee_pose[i]));
    state.append(",");
  }
  state.append(std::to_string(axis_x)); // Joystick X AXIS
  state.append(",");
  state.append(std::to_string(axis_y)); // Joystick Y AXIS
  state.append(",");
  state.append(std::to_string(gs));
  state.append(",");
  state.append(std::to_string(rst));
  state.append(",");
  state.append(std::to_string(start));
  state.append(",");
  state.append(std::to_string(a_press));
  state.append(",");
  state.append(std::to_string(b_press));
  state.append(",");
  state.append(std::to_string(gs_ret));
  state.append(",");
  state.append(std::to_string(gs_at_ret));
  state.append(",");
  state.append(std::to_string(zs));
  state.append(",");
  state.append(std::to_string(zf));
  char cstr[state.size() + 1];
  std::copy(state.begin(), state.end(), cstr);
  cstr[state.size()] = '\0';
  //cout << "sent state" << endl;
  send(sock, cstr, strlen(cstr), 0); // send arm state to python.
}

bool getEE(std::array<double, 4> &nextpose, std::array<double, 2> &nextObj, int sock)
{
  char buffer[200] = {0};
  int valread;
  if (read(sock, buffer, 200) > 0)
  {
    std::stringstream ss(buffer);
    bool first = false;
    while (!first)
    {
      std::string substr;
      getline(ss, substr, ',');
      if (substr[0] == 's')
      {
        first = true;
      }
    }
    // X,Y,and Z directions
    for (int i = 0; i < 3; i++)
    {
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
    for (int i = 0; i < 2; i++)
    {
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

bool getEE(std::array<double, 4> &nextpose, std::array<double, 2> &nextObj, int &valid, int sock)
{
  char buffer[300] = {0};
  int valread;
  if (read(sock, buffer, 300) > 0)
  {
    std::stringstream ss(buffer);
    bool first = false;
    while (!first)
    {
      std::string substr;
      getline(ss, substr, ',');
      if (substr[0] == 's')
      {
        first = true;
      }
    }
    // X,Y,and Z directions
    for (int i = 0; i < 3; i++)
    {
      std::string substr;
      getline(ss, substr, ',');
      //cout << substr << endl;
      double term = std::stod(substr);
      nextpose[i] = term;
    }
    // Velocity Multiplier
    std::string substr;
    getline(ss, substr, ',');
    //cout << substr << endl;
    double term = std::stod(substr);
    nextpose[3] = term;
    // Highest-scored object XY Coordinates
    for (int i = 0; i < 2; i++)
    {
      std::string substr;
      getline(ss, substr, ',');
      //cout << substr << endl;
      double term = std::stod(substr);
      nextObj[i] = term;
    }
    getline(ss, substr, ',');
    //cout << substr << endl;
    term = std::stod(substr);
    valid = term;
    return true;
  }
  return false;
}

void filter_vel(std::array<double, 3> &prev_v, std::array<double, 3> &desired_v, std::array<double, 3> &filtered_v, std::array<double, 3> &prev_a, std::array<double, 3> &desired_a, std::array<double, 3> &filtered_a)
{
  // filter based on Jerk.
  // changes here will change the inputs.
  for (int i = 0; i < 3; ++i)
  {
    desired_a[i] = desired_v[i] - prev_v[i];
  }
  double jerk_magnitude = l2_distance(desired_a, prev_a);
  if (jerk_magnitude <= MAX_JERK)
  {
    for (int i = 0; i < 3; ++i)
    {
      filtered_a[i] = desired_a[i];
    }
  }
  else
  {
    double prop = MAX_JERK / jerk_magnitude;
    for (int i = 0; i < 3; ++i)
    {
      filtered_a[i] = prop * prev_a[i] + (1 - prop) * desired_a[i];
    }
  }

  // should this go before the previous step?
  double accel_magnitude = l2_norm(filtered_a);
  if (accel_magnitude > MAX_ACCEL)
  {
    for (int i = 0; i < 3; ++i)
    {
      filtered_a[i] = (MAX_ACCEL / accel_magnitude) * filtered_a[i];
    }
  }

  for (int i = 0; i < 3; ++i)
  {
    filtered_v[i] = prev_v[i] + filtered_a[i];
  }
  //cout << "fa " << filtered_a[0] << ", " << filtered_a[1] << ", " << filtered_a[2] << endl;
  double v_magnitude = l2_norm(filtered_v);
  if (v_magnitude > MAX_SPEED)
  {
    for (int i = 0; i < 3; ++i)
    {
      filtered_v[i] = (MAX_SPEED / v_magnitude) * filtered_v[i];
    }
  }
  return;
}

int connect2control(int PORT)
{
  int sock = 0;
  struct sockaddr_in serv_addr;
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    std::cout << "Socket Creation Error!" << std::endl;
  }
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT);
  if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
  {
    std::cout << "Invalid Address / Address Not Supported" << std::endl;
  }
  if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
  {
    std::cout << "Connection Failed" << std::endl;
  }
  int status = fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
  if (status == -1)
  {
    std::cout << "Failed to Make the Socket Non-Blocking" << std::endl;
  }

  return sock;
}

int main(int argc, char **argv)
{
  // Robot Connection

  // if (argc != 2) {
  //   cerr << "two arguments required" << endl;
  //   return -1;
  // }
  // int port = std::stoi(argv[1]);
  int port = 53821;

  char *IP;
  if (argc > 1)
  {
    IP = argv[1];
  }
  else
  {
    IP = "10.0.1.8";
  }

  // Joystick Connection
  GamePad *gp = new GamePad(DEVICE_PATH);
  TCPPad *tp = new TCPPad(IP);
  gp->startThread();
  tp->startThread();

  // Connecting to the robot and resetting to home position
  orl::Robot robot("172.16.0.2");
  robot.get_franka_robot().setCollisionBehavior(
                            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                            {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},
                            {{5.0, 5.0, 5.0, 5.0, 5.0, 5.0}},
                            {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}});
  std::cout << "robot connected" << std::endl;
  int sock = connect2control(port);

  // Robot movement
  double time = 0.0;
  double time_goal = 0.0;
  double time_state = 0.0;
  double last_grasp_time = 0.0;
  bool set_last_grasp_time = 0.0;

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
  int pickplace_is_valid = 1;
  int pickplace_valid_buffer = 1;
  

  // -1: default open
  // 1: gripper closed
  // 2: trying to grasp
  // 3: trying to release
  int gripper_state = -1;
  int gripper_state_buffer = -1; // Used to make sure grasp and release gripper states are always sent eventually.
  int gripper_retval = -1; // Using -1 as null, 0 as fail, 1 as success
  int gripper_state_at_ret = -1; // gripper_state at the time that gripper_retval is computed.

  array<double, 2> grasp_at = {NAN, NAN};

  // 1: ready to start new point-to-point motion
  // 2: starting / speeding up new point-to-point motion
  // 3: moving at constant velocity
  int state = 1;
  int prev_start_bs = 0;
  int prev_A_bs = 0;
  int prev_B_bs = 0;
  int start_press_buffer = 0;
  int a_press_buffer = 0;
  int b_press_buffer = 0;
  bool allow_movement = false;
  int rst = 1;
  int finished_rst_buffer = rst;
  bool reset_initiated = false;
  // 0: move as normal
  // 1: move up
  // -1: move down
  // 2: down movement finished
  // 3: moving toward goal
  int z_state = 0;
  double z_contact = 0.0;

  // send state once at the beginning to initiate communication.
  // sendState(pose, 0, 0, gripper_state, sock);
  sendState(pose, 0, 0, gripper_state, 0, sock);

  // Open gripper at start.
  franka::GripperState gripper_read = robot.get_franka_gripper().readOnce();
  robot.get_franka_gripper().move(gripper_read.max_width, 0.05);
  while (true)
  {
    time = 0.0;
    if (rst)
    {
      cout << "Resetting to home position." << endl;
      std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
      robot.joint_motion(q_goal, 0.2);
      robot.absolute_cart_motion(0.3, -0.023, 0.08, 3);
      robot.get_franka_gripper().homing();
      franka::GripperState gripper_read = robot.get_franka_gripper().readOnce();
      cout << "MAX gripper width is: " << gripper_read.max_width << endl;
      rst = 0;
      finished_rst_buffer = 1;
      allow_movement = false;
    }


    // robot.get_franka_gripper().grasp(0.005, 0.03, 15, 0.01, 0.015); // if uncommented, grip something at the start

    cout << "Control loop starting" << endl;
    robot.get_franka_robot().control([=, &time, &state, &time_goal, &time_state, &last_grasp_time, &set_last_grasp_time,
                                      &reset_initiated, &allow_movement, &prev_start_bs, &prev_A_bs, &prev_B_bs,
                                      &a_press_buffer, &b_press_buffer, &start_press_buffer,
                                      &pickplace_is_valid, &pickplace_valid_buffer, &gripper_retval, &gripper_state_at_ret,
                                      &grasp_at,
                                      &z_state, &z_contact,
                                      &q, &delta, &prev_delta, &prev_accel, &next_delta, &delta_unit, &next_delta_unit, &next_obj, &gripper_state, &gripper_state_buffer,
                                      &goal, &next_goal, &pose, &cur_velocity, &gp, &tp, &rst, &finished_rst_buffer, &received_cmd](const franka::RobotState &robot_state,
                                                                                                                                    franka::Duration period) -> franka::CartesianVelocities
                                     {
      bool send_finished = false;                                      
      double delta_t = period.toSec();
      time += period.toSec();
      time_goal += period.toSec();
      time_state += period.toSec();

      if (set_last_grasp_time) {
        last_grasp_time = time;
        set_last_grasp_time = false;
      }

      int bs = gp->getButtonStateAtomic();
      int reset_bs = gp->getButtonState(4);
      int start_bs = gp->getButtonState(9);
      int A_bs = gp->getButtonState(1);
      int B_bs = gp->getButtonState(2);
      int X_bs = gp->getButtonState(0);
      int Y_bs = gp->getButtonState(3);
      int RB_bs = gp->getButtonState(5);
      
      // cout << "bs all ";
      // for (int i=0; i<10; i++) {
      //   cout << gp->getButtonState(i) << ", ";
      // }
      // cout << endl;

      int axis_x = tp->getAxisX();
      int axis_y = tp->getAxisY();
      array<double, 3> cur; // current position
      franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}; // xv, vy, vz, omega_x, omega_y, omega_z

      // cout << "pose: ";
      // for (int i=0; i<16; ++i) {
      //   cout << pose[i] << ", ";
      // }
      // cout << endl;

      pose = robot_state.O_T_EE_c; // pose[12] = x, pose[13] = y, pose[14] = z
      std::array<double, 6> last_commanded_accel = robot_state.O_ddP_EE_c;
      //cout << "last commanded accel " << last_commanded_accel[0] << ", " << last_commanded_accel[1] << ", " << last_commanded_accel[2] << ", " << endl;

      for (int i = 0; i < 3; i++) {
        cur[i] = pose[12+i];
      }
      if (delta_t < 0.0001) {
        for (int i=0; i<3; ++i) {
          delta[i] = 0.0;
        }
      }

      // Fetching the next point from the decoder
      array<double, 4> next;
      bool got_new_control = false;
      if (getEE(next, next_obj, pickplace_is_valid, sock)) {
        //cout << "READ NEW COMMAND" << endl;
        got_new_control = true;
        // Slow to zero if velocity scale is zero.
        if (next[3] == 0.0) {
          for (int i=0; i<3; ++i) {
            next[i] = 0.0;
          }
        }

        for (int i = 0; i < 3; i++) {
          next[i] += cur[i];
        }
        q.push(next);
        goal = q.front();
        q.pop();

        // store desired velocity.
        // possible change: copy based only on state.
        for (int i = 0; i < 3; i++) {
          delta[i] = goal[i] - pose[12+i];
        }
        if(X_bs == 1 && robot_state.cartesian_contact[2] == 0) {
          delta[2] = 0.05;
        }
        if(Y_bs == 1 && robot_state.cartesian_contact[2] == 0) {
          delta[2] = -0.05;
        }
      }

      // cout << "collision: ";
      // for(int i=0; i<6; i++) {
      //   cout << robot_state.cartesian_collision[i] << "|";
      // }
      // cout << endl;
      // cout << "contact: ";
      // for(int i=0; i<6; i++) {
      //   cout << robot_state.cartesian_contact[i] << "|";
      // }
      // cout << endl;
      // cout << "O_F: ";
      // for(int i=0; i<6; i++) {
      //   cout << robot_state.O_F_ext_hat_K[i] << "|";
      // }
      // cout << endl;
      // // normally under 6N during free motion.
      // double contact_force = sqrt(pow(robot_state.O_F_ext_hat_K[0], 2.0) + pow(robot_state.O_F_ext_hat_K[1], 2.0) + pow(robot_state.O_F_ext_hat_K[2], 2.0));
      // cout << "force " << contact_force << endl;
      // // cout << "K_F: ";
      // // for(int i=0; i<6; i++) {
      // //   cout << robot_state.K_F_ext_hat_K[i] << "|";
      // // }
      // // cout << endl;
      
      

      
      // Check if EE within range of goal
      // If so, set delta to 0.0
      //   after it slows down,
      //   set gripper_state (can also be done before)
      //   then return franka::MotionFinished(output)
      //   where output is zero.

      // // Don't try to pick or place within 5mm of the safety polygon.
      // std::array<double, 2> cur_xy = {cur[0], cur[1]};
      // std::array<double, 2> cur_xy_projection = {0.0, 0.0};
      // double cur_dist_to_polygon = project_onto_2D_polygon(cur_xy, &ROBOT_LIMITS_POLY_X[0], &ROBOT_LIMITS_POLY_Y[0], ROBOT_LIMITS_POLY_X.size(), cur_xy_projection);
      // bool cur_is_in_bounds = is_within_2D_polygon(cur_xy, &ROBOT_LIMITS_POLY_X[0], &ROBOT_LIMITS_POLY_Y[0], ROBOT_LIMITS_POLY_X.size());
      // if (!(cur_is_in_bounds && (cur_dist_to_polygon > 0.005))) {
      //   last_grasp_time = time;
      // }

      //bool goal_reached = ((time - last_grasp_time >= GRASP_COOLDOWN) && reached_object(cur, next_obj, GRASP_RADIUS));
      bool goal_reached = ((time - last_grasp_time >= GRASP_COOLDOWN) && (pickplace_is_valid == 1));
      if (goal_reached && isnan(grasp_at[0])) {
        grasp_at = next_obj;
      }
      // For debugging only.
      if (RB_bs == 1) {
        goal_reached = true;
        next_obj = {cur[0], cur[1]};
      }

      //cout << "grasp_at " << grasp_at[0] << ", " << grasp_at[1] << endl;
      if (goal_reached && (z_state == 0 || z_state == 3)) {
        //cout << "moving to grasp location" << endl;
        // Set desired speed to point toward target if xy is within GRASP_RADIUS
        if (gripper_state == 1) {
          double dx = next_obj[0] - cur[0];
          double dy = next_obj[1] - cur[1];
          double dz = -0.03 - cur[2];
          double dr = sqrt(pow(dx, 2) + pow(dy, 2));
          delta[0] = 8.0*clip(dr/0.010, 0.0, 1.0)*dx;
          delta[1] = 8.0*clip(dr/0.010, 0.0, 1.0)*dy;
          delta[2] = 0.0; //1.0*dz;
        } else if (gripper_state == -1) {
          double dx = next_obj[0] - cur[0];
          double dy = next_obj[1] - cur[1];
          double dz = -0.03 - cur[2];
          double dr = sqrt(pow(dx, 2) + pow(dy, 2));
          delta[0] = 8.0*clip(dr/0.010, 0.0, 1.0)*dx;
          delta[1] = 8.0*clip(dr/0.010, 0.0, 1.0)*dy;
          delta[2] = 0.0; //1.0*dz;
        } else {
          cout << "unknown gripper state for goal reached" << endl;
        }
        z_state = 3;
      }
      // cout << "force ";
      // for(int i=0; i<6; i++) {
      //   cout << robot_state.O_F_ext_hat_K[i] << ", ";
      // }
      // cout << endl;

      //cout << "!z " << z_state << "," << z_contact << endl;
      double z_force = robot_state.O_F_ext_hat_K[2];
      if (z_state == -1) {
        // Moving down after setting goal.
        delta[0] = 0;
        delta[1] = 0;
        if (abs(z_force) >= 5.0) {
          // Contact.
          cout << "Contact!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
          z_state = 1; // move up
          z_contact = cur[2];
        } else if (cur[2] < -0.065) {
          delta[2] = 0.0;
          z_state = 2; // stop moving down.
        } else {
          delta[2] = -0.04;
        }
      } else if (z_state == 1) {
        delta[0] = 0;
        delta[1] = 0;
        delta[2] = (z_contact + 0.030) - cur[2];
        if (cur[2] > z_contact + 0.025) {
          z_state = 0;
          last_grasp_time = time;
          cout << "setting last grasp time" << endl;
        }
      } else if (z_state == 2) {
        delta[0] = 0.0;
        delta[1] = 0.0;
        delta[2] = 0.0;
      }
      cout << "zz " << z_state << "," << z_contact << "," << goal_reached << endl;

      if (start_bs == 1 && prev_start_bs == 0) {
        allow_movement = (!allow_movement);

        // Stop sending finished_reset when you allow movement.
        // This is used so that the Python process knows when the robot is
        // reset but has not yet commenced movement.
        // THIS WILL FAIL IF YOU HOLD DOWN START DURING RESET.
        // i.e. unpressed -> reset started -> pressed -> reset finished (-> unpressed)
        // will cause the finished_reset signal to never be sent.
        if (allow_movement) {
          finished_rst_buffer = 0;
        }
        start_press_buffer = 1;
      }
      if (!allow_movement) {
        // Used to enable the cooldown after enabling movement.
        last_grasp_time = time;
      }
      prev_start_bs = start_bs; // must be after every reference to prev_start_bs
      int is_A_press = (A_bs==1)*(prev_A_bs==0);
      int is_B_press = (B_bs==1)*(prev_B_bs==0);
      if (is_A_press) {
        a_press_buffer = 1;
      }
      if (is_B_press) {
        b_press_buffer = 1;
      }
      prev_A_bs = A_bs;
      prev_B_bs = B_bs;

      if (!allow_movement) {
        for (int i=0; i<3; ++i) {
          delta[i] = 0.0;
        }
      }

      // reset behavior. This must occur after every modifier of delta
      //cout << "bs" << bs << endl;
      if (reset_bs == 1) {
        reset_initiated = true;
      }
      if (reset_initiated) {
        for (int i=0; i<3; ++i) {
          delta[i] = 0.0;
        }
      }
      // e.g.: timeout condition is reached from the python side.
      if (pickplace_is_valid == -1) {
        for (int i=0; i<3; ++i) {
          delta[i] = 0.0;
        }
      }


      // delta corresponds to the desired velocity
      // prev_delta corresponds to the previous velocity.
      // Here we ensure that the new delta is within a radius of prev_delta
      // requires: delta, prev_delta, prev_accel
      // TODO: functionize this.
      array<double, 3> filtered_delta = {0, 0, 0};
      array<double, 3> desired_accel = {0, 0, 0}; // difference in delta. corresponds to desired accel
      array<double, 3> filtered_accel = {0, 0, 0}; // units: m/s/tick

      for (int i=0; i<3; ++i){
        prev_accel[i] = last_commanded_accel[i] / 1000.0;
      }

      // Copy vel
      // Rotation correction using Tait-Bryan XYZ angles 
      // https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix
      double xtheta = std::atan2(-pose[6], -pose[10]);
      double ytheta = std::asin(-pose[2]);
      double ztheta = std::atan2(pose[1], pose[0]);
      //std::cout << "angles" << xtheta << ", " << ytheta << ", " << ztheta << ", " << endl;
      //const std::array<double, 6> des_vel = {delta[0], delta[1], delta[2], -xtheta, -ytheta, -ztheta};
      std::array<double, 6> des_vel = {delta[0], delta[1], delta[2], -xtheta, -ytheta, -ztheta};
      //const std::array<double, 6> des_vel = {delta[0], delta[1], delta[2], 0.0, 0.0, 0.0};
      filter_vel(prev_delta, delta, filtered_delta, prev_accel, desired_accel, filtered_accel);
      //cout << "filtered_accel " << 1.0e6*filtered_accel[0] << ", " << 1.0e6*filtered_accel[1] << ", " << 1.0e6*filtered_accel[2] << ", " << endl;

      // Ensure that velocity places the next point within the safety polygon (xy only)
      double vel_extension_factor = 0.1; // currently scaled by euler's number e times 1ms.
      std::array<double, 2> next_xy = {cur[0] + vel_extension_factor*des_vel[0], cur[1] + vel_extension_factor*des_vel[1]};
      bool within_poly = is_within_2D_polygon(next_xy, &ROBOT_LIMITS_POLY_X[0], &ROBOT_LIMITS_POLY_Y[0], ROBOT_LIMITS_POLY_X.size());
      //cout << "within poly" << within_poly << endl;
      std::array<double, 2> next_xy_projection;
      if (!within_poly) {
        double dist_to_polygon = project_onto_2D_polygon(next_xy, &ROBOT_LIMITS_POLY_X[0], &ROBOT_LIMITS_POLY_Y[0], ROBOT_LIMITS_POLY_X.size(), next_xy_projection);
        //cout << "projected point " << next_xy_projection[0] << ", " << next_xy_projection[1] << endl;
        
        double new_vel_x = (next_xy_projection[0] - cur[0])/vel_extension_factor;
        double new_vel_y = (next_xy_projection[1] - cur[1])/vel_extension_factor;
        des_vel[0] = new_vel_x;
        des_vel[1] = new_vel_y;
      }


      std::array<double, 6> limited_vel = franka::limitRate(FRANKA_MAX_VEL, FRANKA_MAX_ACCEL, FRANKA_MAX_JERK, 
        FRANKA_MAX_ROT_VEL, FRANKA_MAX_ROT_ACCEL, FRANKA_MAX_ROT_JERK,
        des_vel,
        robot_state.O_dP_EE_c,
        robot_state.O_ddP_EE_c);
      

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

      /**/

      std::array<double, 3> limited_translational_vel = {limited_vel[0], limited_vel[1], limited_vel[2]};
      std::array<double, 3> last_translational_vel = {robot_state.O_dP_EE_c[0], robot_state.O_dP_EE_c[1], robot_state.O_dP_EE_c[2]};

      //double tmp = l2_norm(&last_translational_vel[0], last_translational_vel.size());
      //cout << "tmp " << tmp << endl;




      //cout << "gr " << goal_reached << "," << l2_norm(last_translational_vel) << endl;

      //cout << l2_norm(last_translational_vel) << endl;
      // TODO: fix goal_reached condition.
      std::array<double, 6> zero_output = {0, 0, 0, 0, 0, 0};
      if (l2_norm(last_translational_vel) <= 0.1e-4 && reset_initiated) {
        // i.e. is close to stopped
        cout << "Sending reset signal" << endl;
        reset_initiated = false;
        rst = 1;
        //finished_rst_buffer = rst;
        send_finished = true;
      } else if (goal_reached && l2_norm(last_translational_vel) <= 0.0007) {
        std::array<double, 6> zero_output = {0, 0, 0, 0, 0, 0};
        if (z_state == 2) {
          cout << "Motion finished. Initiating grasp. " << gripper_state << endl;
          cout << z_state << endl;
          if (gripper_state == -1) {
            gripper_state = 2;
            gripper_state_buffer = 2;
          } else if (gripper_state == 1) {
            gripper_state = 3;
            gripper_state_buffer = 3;
          }
          send_finished = true;
        } else if (z_state == 3) {
          z_state = -1;
        }
      }

      // IMPORTANT:
      // THIS MUST BE BEFORE ANY RETURN STATEMENT.
      if (got_new_control) {
        // Ensure that gripper state values of 2 and 3 are sent at least once.
        int gs_to_send = gripper_state;
        int rst_to_send = 0;
        int start_press_to_send = 0;
        int a_press_to_send = is_A_press;
        int b_press_to_send = is_B_press;
        if (finished_rst_buffer == 1) {
          //sendState(pose, axis_x, axis_y, gripper_state_buffer, sock);
          rst_to_send = finished_rst_buffer;
          //finished_rst_buffer = 0;
        }
        if (gripper_state_buffer == 2) {
          //sendState(pose, axis_x, axis_y, gripper_state_buffer, sock);
          gs_to_send = gripper_state_buffer;
          gripper_state_buffer = 1;
        } else if (gripper_state_buffer == 3) {
          //sendState(pose, axis_x, axis_y, gripper_state_buffer, sock);
          gs_to_send = gripper_state_buffer;
          gripper_state_buffer = -1;
        } else {}
        if (a_press_buffer == 1) {
          a_press_to_send = a_press_buffer;
          a_press_buffer = 0;
        }
        if (b_press_buffer == 1) {
          b_press_to_send = b_press_buffer;
          b_press_buffer = 0;
        }
        if (start_press_buffer == 1){
          start_press_to_send = start_press_buffer;
          start_press_buffer = 0;
        }
        //sendState(pose, axis_x, axis_y, gs_to_send, rst_to_send, sock);
        sendState(pose, axis_x, axis_y, gs_to_send, rst_to_send, start_press_to_send, a_press_to_send, b_press_to_send, 
                  gripper_retval, gripper_state_at_ret, z_state, z_force, sock);

        // refresh some buffers
        gripper_retval = -1;
      }

      //cout << "next_obj " << next_obj[0] << ", " << next_obj[1] << endl;

      if (send_finished) {
        return franka::MotionFinished(zero_output);
      }
      //return output;
      return limited_vel; 
    });
    cout << "Gripper state" << gripper_state << endl;
    if (rst) {
      if (gripper_state == 1) {
        // Release if holding an object.
        franka::GripperState gripper_read = robot.get_franka_gripper().readOnce();
        robot.get_franka_gripper().move(gripper_read.max_width, 0.05);
      }
      gripper_state = -1; // Set state to not holding.
    }

    // MISSING: feedback to python script if the grip is successful or not.
    if (gripper_state == 2)
    {
      cout << "!!attempting gripper at " << next_obj[0] << ", " << next_obj[1] << endl;
      // original duration 1.2
      ////robot.absolute_cart_motion(next_obj[0], next_obj[1], pose[14], 0.6);
      
      //if (pickplace_is_valid) {
      //  robot.absolute_cart_motion(next_obj[0], next_obj[1], -0.066, 1.8);
      //} else {
      //  robot.absolute_cart_motion(next_obj[0], next_obj[1], -0.066+0.0254, 1.8);
      //}
      cout << "a" << endl;
      double force = 18;
      gripper_retval = robot.get_franka_gripper().grasp(0.030, 0.03, force, 0.01, 0.015);
      gripper_state_at_ret = 2;
      if (gripper_retval) {
        gripper_state = 1;
        cout << "Object Grasped" << endl;
      } else {
        gripper_state = -1;
        franka::GripperState gripper_read = robot.get_franka_gripper().readOnce();
        cout << "MAX gripper width is: " << gripper_read.max_width << endl;
        cout << "Object unable to be Grasped" << endl;
        robot.get_franka_gripper().move(gripper_read.max_width, 0.05);
      }
      cout << "b" << endl;
      // original duration 2.5
      //robot.absolute_cart_motion(next_obj[0], next_obj[1], 0.08, 3.0);
      robot.absolute_cart_motion(next_obj[0], next_obj[1], 0.03, 1.5);
      set_last_grasp_time = true;
      cout << "c" << endl;
    }
    else if (gripper_state == 3)
    {
      // original z -0.065
      ////robot.absolute_cart_motion(next_obj[0], next_obj[1], pose[14], 0.6);
      //if (pickplace_is_valid) {
      //  robot.absolute_cart_motion(next_obj[0], next_obj[1], -0.065, 1.8);
      //} else {
      //  robot.absolute_cart_motion(next_obj[0], next_obj[1], -0.065+1.3*0.0254, 1.8);
      //}
      franka::GripperState gripper_read = robot.get_franka_gripper().readOnce();
      gripper_retval = robot.get_franka_gripper().move(gripper_read.max_width, 0.05);
      gripper_state_at_ret = 3;
      if (gripper_retval) {
        gripper_state = -1;
        cout << "Object Released" << endl;
      } else {
        gripper_state = 1;
        cout << "Object unable to be Released" << endl;
      }
      //robot.absolute_cart_motion(next_obj[0], next_obj[1], 0.08, 3.0);
      robot.absolute_cart_motion(next_obj[0], next_obj[1], 0.03, 1.5);
      set_last_grasp_time = true;
    }
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Resetting z_state" << endl;
    z_state = 0;
    cout << "gret" << gripper_retval << endl;
    grasp_at = {NAN, NAN};
  }
  tp->shutdown();

  return 0;
}