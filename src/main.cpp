#include "GamePad.h"
#include "Arm.h"
#include "Comms.h"
#include "rapidjson/rapidjson.h"

#include <string>

static const char *DEVICE_PATH = "/dev/input/js0";
static const std::string ARM_ADDR = "172.16.0.2";

int main()
{
    // initialize GamePad object
    GamePad *gp = new GamePad(DEVICE_PATH);
    gp->startThread();

    // initialize franka robot
    // Arm *arm = new Arm(ARM_ADDR, gp);
    // arm->controlfuncs();
    // arm->controldirect();
    // arm->controlvel();

    Comms *server = new Comms(5555);
    server->receive_data();
    std::string str("hi hi hi");
    server->send_data(str.c_str());


    return 0;
}