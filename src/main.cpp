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

    Comms *server = new Comms(5555);
    std::string str("connection successful");
    server->send_data(str.c_str());

    // initialize franka robot
    Arm *arm = new Arm(ARM_ADDR, gp, server);
    arm->controlvel();

    delete arm;
    delete server;

    return 0;
}