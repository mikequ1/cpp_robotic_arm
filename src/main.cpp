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

    Comms *server = new Comms(5555);
    server->startThread();

    // initialize GamePad object
    GamePad *gp = new GamePad(DEVICE_PATH, server);
    gp->startThread();

    // initialize franka robot
    Arm *arm = new Arm(ARM_ADDR, gp, server);
    // arm->controlvel();


    //TODO: instead of reading incoming TCP comms in the main thread, read in new thread and push to queue
    while (server->get_q().size() != 0) {
        std::string buffer = server->get_q().pop();
        printf(buffer);
        rapidjson::Document document;
        document.Parse(buffer.c_str());
        if (document["func"] == "pose") {
            arm->get_pose();
        }
    }

    delete arm;
    delete server;

    return 0;
}