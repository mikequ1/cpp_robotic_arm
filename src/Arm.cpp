#include "Arm.h"

#include <cmath>
#include <iostream>
#include <vector>

#include <franka/exception.h>
#include <liborl/SpeedProfile.h>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "examples_common.h"

using namespace std;

Arm::Arm(const std::string &addr, GamePad *gp, Comms *c)
{
    m_robot = new orl::Robot(addr);
    m_gp = gp;
    m_c = c;
    m_finished = 0;

    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    std::cout << "WARNING: This program will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    m_robot->joint_motion(q_goal, 0.2);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
}

void Arm::controlfuncs()
{
    std::array<double, 16> initial_pose;
    double time = 0.0;      // total time elapsed
    double thistrack = 0.0; // time elapsed on current trajectory
    double rest = 9999.0;   //
    int ready = 1;
    GamePad *gp = m_gp;
    m_robot->get_franka_robot().control([&time, &initial_pose, &gp, &thistrack, &ready, &rest](const franka::RobotState &robot_state,
                                                                                               franka::Duration period) -> franka::CartesianPose
                                        {
        time += period.toSec();
        thistrack += period.toSec();

        if (time == 0.0) {
            initial_pose = robot_state.O_T_EE_c;
        }

        if (time >= rest) {
            ready = 1;
            rest = 9999.0;
            thistrack = 0;
            cout << "Ready for next motion" << endl;
        }

        std::array<double, 16> new_pose = initial_pose;
        int bs = gp->getButtonState();
        // A button
        if (bs == 2) {
            if (thistrack >= 5.0 && ready != 2) {
                ready = 2;
                rest = time + 2;
                cout << "Motion finished, release button" << endl;
            }
            if (ready == 1) {
                thistrack = 0;
                ready = 0;
                cout << "Starting motion for button A" << endl; 
            }
            if (ready == 0) {
                constexpr double kRadius = 0.3;
                double angle = M_PI / 4 * (1 - std::cos(M_PI / 2.5 * thistrack));
                double delta_z = kRadius * (std::cos(angle) - 1);

                new_pose[14] += delta_z;//down
            }
        }

      // X button
        if (bs == 1) {
            if (thistrack >= 5.0 && ready != 2) {
                ready = 2;
                rest = time + 2;
                cout << "Motion finished, release button" << endl;
            }
            if (ready == 1) {
                thistrack = 0;
                ready = 0;
                cout << "Starting motion for button X" << endl; 
            }
            if (ready == 0) {
                constexpr double kRadius = 0.3;
                double angle = M_PI / 4 * (1 - std::cos(M_PI / 2.5 * thistrack));
                double delta_x = kRadius * std::sin(angle);
                double delta_z = kRadius * (std::cos(angle) - 1);

                new_pose[12] += delta_x; //forward
                new_pose[14] += delta_z; //down
            }
        }

      // B button
        if (bs == 4) {
            if (thistrack >= 5.0 && ready != 2) {
                ready = 2;
                rest = time + 2;
                cout << "Motion finished, release button" << endl;
            }
            if (ready == 1) {
                thistrack = 0;
                ready = 0;
                cout << "Starting motion for button B" << endl; 
            }
            if (ready == 0) {
                constexpr double kRadius = 0.3;
                double angle = M_PI / 4 * (1 - std::cos(M_PI / 2.5 * thistrack));
                double delta_z = kRadius * (std::cos(angle) - 1);

                new_pose[14] -= delta_z; //up
            }
        }

      // Y button
        if (bs == 8) {
            if (thistrack >= 5.0 && ready != 2) {
                ready = 2;
                rest = time + 2;
                cout << "Motion finished, release button" << endl;
            }
            if (ready == 1) {
                thistrack = 0;
                ready = 0;
                cout << "Starting motion for button Y" << endl; 
            }
            if (ready == 0) {
                constexpr double kRadius = 0.3;
                double angle = M_PI / 4 * (1 - std::cos(M_PI / 2.5 * thistrack));
                double delta_x = kRadius * std::sin(angle);

                new_pose[12] += delta_x; // forward
            }
        }

        if (time >= 40.0) {
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(new_pose);
        }
        return new_pose; });
}

void Arm::controlvel()
{
    double time_max = 0.2;
    double v_max = 0.1;
    double angle = M_PI / 4.0;
    double time = 0.0;
    double thistrack = 0.0;
    int status = 1;

    GamePad *gp = m_gp;
    this->get_gripper_width();
    this->goto_gripper(0.08);
    this->get_gripper_width();
    this->goto_gripper(0.01);
    this->goto_pose(0.65, 0.0, 0.2, 3);
    this->goto_pose_delta(-0.2, 0, 0.1, 2);
    m_robot->get_franka_robot().control([=, &status, &time, &thistrack](const franka::RobotState &robot_state,
                                                                        franka::Duration period) -> franka::CartesianVelocities
                                        {
        
        int bs = gp->getButtonState();

        time += period.toSec();
        thistrack += period.toSec();

        this->get_pose(robot_state.O_T_EE_c);

        cout << status << endl;
        franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        if (bs == 8) {
            if (status == 1){
                thistrack = 0;
                status = 88;
            }
            if (status == 88){
                if (thistrack >= time_max / 2)
                    status = 80;
                double v = v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * thistrack));
                double v_z = std::sin(angle) * v;
                franka::CartesianVelocities output = {{0.0, 0.0, v_z, 0.0, 0.0, 0.0}};
                return output;
            }
            if (status == 80) {
                thistrack = 0;
                double v = v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * (time_max / 2)));
                double v_z = std::sin(angle) * v;
                franka::CartesianVelocities output = {{0.0, 0.0, v_z, 0.0, 0.0, 0.0}};
                return output;
            }
        } 

        if (bs == 2) {
            if (status == 1){
                thistrack = 0;
                status = 28;
            }
            if (status == 28){
                if (thistrack >= time_max / 2)
                    status = 20;
                double v = v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * thistrack));
                double v_z = std::sin(angle) * v;
                franka::CartesianVelocities output = {{0.0, 0.0, -v_z, 0.0, 0.0, 0.0}};
                return output;
            }
            if (status == 20) {
                thistrack = 0;
                double v = v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * (time_max / 2)));
                double v_z = std::sin(angle) * v;
                franka::CartesianVelocities output = {{0.0, 0.0, -v_z, 0.0, 0.0, 0.0}};
                return output;
            }
        } 

        if (bs == 1) {
            if (status == 1){
                thistrack = 0;
                status = 18;
            }
            if (status == 18){
                if (thistrack >= time_max / 2)
                    status = 10;
                double v = v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * thistrack));
                double v_x = std::cos(angle) * v;
                franka::CartesianVelocities output = {{v_x, 0.0, 0.0, 0.0, 0.0, 0.0}};
                return output;
            }
            if (status == 10) {
                thistrack = 0;
                double v = v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * (time_max / 2)));
                double v_x = std::cos(angle) * v;
                franka::CartesianVelocities output = {{v_x, 0.0, 0.0, 0.0, 0.0, 0.0}};
                return output;
            }
        } 

        if (bs == 4) {
            if (status == 1){
                thistrack = 0;
                status = 48;
            }
            if (status == 48){
                if (thistrack >= time_max / 2)
                    status = 40;
                double v = v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * thistrack));
                double v_x = std::cos(angle) * v;
                franka::CartesianVelocities output = {{-v_x, 0.0, 0.0, 0.0, 0.0, 0.0}};
                return output;
            }
            if (status == 40) {
                thistrack = 0;
                double v = v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * (time_max / 2)));
                double v_x = std::cos(angle) * v;
                franka::CartesianVelocities output = {{-v_x, 0.0, 0.0, 0.0, 0.0, 0.0}};
                return output;
            }
        } 

        if (bs == 0) {
            if (status == 80) {
                thistrack = 0;
                status = 89;
            }
            if (status == 89) {
                if (thistrack >= time_max / 2)
                    status = 1;
                double v = v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * (thistrack + time_max/2)));
                double v_z = std::sin(angle) * v;
                franka::CartesianVelocities output = {{0.0, 0.0, v_z, 0.0, 0.0, 0.0}};
                return output;
            }
            if (status == 20) {
                thistrack = 0;
                status = 29;
            }
            if (status == 29) {
                if (thistrack >= time_max / 2)
                    status = 1;
                double v = v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * (thistrack + time_max/2)));
                double v_z = std::sin(angle) * v;
                franka::CartesianVelocities output = {{0.0, 0.0, -v_z, 0.0, 0.0, 0.0}};
                return output;
            }
            if (status == 10) {
                thistrack = 0;
                status = 19;
            }
            if (status == 19) {
                if (thistrack >= time_max / 2)
                    status = 1;
                double v = v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * (thistrack + time_max/2)));
                double v_x = std::cos(angle) * v;
                franka::CartesianVelocities output = {{v_x, 0.0, 0.0, 0.0, 0.0, 0.0}};
                return output;
            }
            if (status == 40) {
                thistrack = 0;
                status = 49;
            }
            if (status == 49) {
                if (thistrack >= time_max / 2)
                    status = 1;
                double v = v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * (thistrack + time_max/2)));
                double v_x = std::cos(angle) * v;
                franka::CartesianVelocities output = {{-v_x, 0.0, 0.0, 0.0, 0.0, 0.0}};
                return output;
            }
        }

        if (time >= 60) {
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(output);
        }
        return output; });
}

void Arm::get_pose()
{
    franka::RobotState state = m_robot->get_franka_robot().readOnce();
    std::array<double, 16> pose = state.O_T_EE;
    this->get_pose(pose);
}

void Arm::get_pose(std::array<double, 16> pose)
{
    rapidjson::Document document;
    document.SetObject();
    rapidjson::Value poseArray(rapidjson::kArrayType);
    rapidjson::Document::AllocatorType &allocator = document.GetAllocator();
    for (int i = 0; i < 16; i++)
    {
        poseArray.PushBack(std::round(pose[i] * 1000.0) / 1000.0, allocator);
    }
    document.AddMember("pose", poseArray, allocator);
    rapidjson::StringBuffer strbuf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
    document.Accept(writer);

    m_c->send_data(strbuf.GetString());
}

void Arm::get_gripper_width()
{
    double width = m_robot->get_franka_gripper().readOnce().width;
    rapidjson::Document document;
    document.SetObject();
    rapidjson::Document::AllocatorType &allocator = document.GetAllocator();
    document.AddMember("gripper_width", width, allocator);
    rapidjson::StringBuffer strbuf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
    document.Accept(writer);

    m_c->send_data(strbuf.GetString());
}

void Arm::goto_gripper(double dest)
{
    m_robot->get_franka_gripper().move(dest, 0.1);
}

void Arm::goto_pose(double x, double y, double z, double duration)
{
    m_robot->absolute_cart_motion(x, y, z, duration);
}

void Arm::goto_pose(std::array<double, 16> pose, double duration)
{
    m_robot->absolute_cart_motion(pose[12], pose[13], pose[14], duration);
}

void Arm::goto_pose_delta(double dx, double dy, double dz, double duration)
{
    auto tcp_motion = orl::PoseGenerators::RelativeMotion(orl::Position(dx, dy, dz), Frame::RobotTCP);
    orl::apply_speed_profile(tcp_motion);
    m_robot->move_cartesian(tcp_motion, duration);
}

bool Arm::isFinished()
{
    return m_finished;
}