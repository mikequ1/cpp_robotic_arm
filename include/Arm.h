#pragma once

#include "GamePad.h"
#include <string>
#include <franka/robot.h>

class Arm
{
public:
    /**
     * @brief Construct a new Arm object
     *
     * @param addr Local IP address of the robotic arm
     * @param gp GamePad pointer used to interface with the arm
     */
    Arm(const std::string &addr, GamePad *gp);

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

    bool isFinished();

private:
    /// @brief franka bot
    franka::Robot *m_robot;
    /// @brief gamepad pointer
    GamePad *m_gp;
    bool m_finished;
};