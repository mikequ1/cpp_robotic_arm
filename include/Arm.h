#pragma once

#include "GamePad.h"
#include "Comms.h"
#include <string>
#include <array>
#include <franka/robot.h>
#include <liborl/liborl.h>

class Arm
{
public:
    /**
     * @brief Construct a new Arm object
     *
     * @param addr Local IP address of the robotic arm
     * @param gp GamePad pointer used to interface with the arm
     */
    Arm(const std::string &addr, GamePad *gp, Comms *c);

    /**
     * @brief Allows a user to assign 4 deterministic movement functions to the robot
     * and play those movements with the gamepad.
     */
    void controlfuncs();
    
    /**
     * @brief Allows a user to use the X Y A B keys control the arm in the X-Z 2D space.
     *
     */
    void controlvel();


    /**
     * @brief Get the pose of the arm via socket, when control loop ISN'T running
     * 
     */
    void get_pose();

    /**
     * @brief Get the pose of the arm via socket, when control loop IS running
     * 
     */
    void get_pose(std::array<double, 16> pose);

    /**
     * @brief Get the gripper width of the arm via socket, when control loop ISN'T running
     * 
     */
    void get_gripper_width();

    /**
     * @brief Go to gripper position
     * 
     * @param dest represents the destination for the gripper to move to.
     */
    void goto_gripper(double dest);

    /**
     * @brief Go to destination pose over the specified duration
     * 
     * @param x absolute cartesian x position of destination
     * @param y absolute cartesian y position of destination
     * @param z absolute cartesian z position of destination
     * @param duration time elapsed over movement
     */
    void goto_pose(double x, double y, double z, double duration);

    /**
     * @brief Go to destination pose over the specified duration
     * 
     * @param pose 16-element array specifying desired destination pose
     * @param duration time elapsed over movement
     */
    void goto_pose(std::array<double, 16> pose, double duration);

    void goto_pose_delta(double dx, double dy, double dz, double duration);
    
    bool isFinished();

private:
    /// @brief franka orl bot
    orl::Robot *m_robot;

    /// @brief gamepad pointer
    GamePad *m_gp;
    Comms *m_c;
    bool m_finished;
};