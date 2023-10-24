#include "GamePad.h"
#include "Arm.h"
#include "Comms.h"
#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"

#include <string>
#include <thread>
#include <queue>

static const char *DEVICE_PATH = "/dev/input/js0";
static const std::string ARM_ADDR = "172.16.0.2";

int main()
{

    // Comms *server = new Comms(5555);
    Comms *server = nullptr;
    // server->startThread();

    // initialize GamePad object
    GamePad *gp = new GamePad(DEVICE_PATH, server);
    gp->startThread();

    // initialize franka robot
    Arm *arm = new Arm(ARM_ADDR, gp, server);
    arm->controlvel();


    //TODO: instead of reading incoming TCP comms in the main thread, read in new thread and push to queue
    // while (true) {
    //     std::string buffer;
    //     if (server->get_command(buffer) == false)
    //         continue;
    //     rapidjson::Document document;
    //     document.Parse(buffer.c_str());
    //     if (document["func"] == "get_pose") {
    //         arm->get_pose();
    //     }
    //     if (document["func"] == "goto_pose") {
    //         double x = document["x"].GetDouble();
    //         double y = document["y"].GetDouble();
    //         double z = document["z"].GetDouble();
    //         double duration = document["duration"].GetDouble();
    //         arm->goto_pose(x,y,z,duration);
    //     }
    //     if (document["func"] == "goto_pose_delta") {
    //         double x = document["x"].GetDouble();
    //         double y = document["y"].GetDouble();
    //         double z = document["z"].GetDouble();
    //         double duration = document["duration"].GetDouble();
    //         arm->goto_pose_delta(x,y,z,duration);
    //     }
    //     if (document["func"] == "goto_gripper") {
    //         double dest = document["dest"].GetDouble();
    //         arm->goto_gripper(dest);
    //     }
    //     if (document["func"] == "get_action") {
    //         gp->get_action();
    //     }
    //     if (document["func"] == "get_gripper_width") {
    //         arm->get_gripper_width();
    //     }
    // }

    delete arm;
    delete server;

    return 0;
}