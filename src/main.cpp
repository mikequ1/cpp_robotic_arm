#include "GamePad.h"
#include "Arm.h"

#include <string>

static const char *DEVICE_PATH = "/dev/input/js0";
static const std::string ARM_ADDR = "172.16.0.2";

int main()
{
    // initialize GamePad object
    GamePad *gp = new GamePad(DEVICE_PATH);
    gp->startThread();

    // initialize franka robot
    Arm *arm = new Arm(ARM_ADDR, gp);
    // arm->controlfuncs();
    // arm->controldirect();
    arm->controlvel();

    return 0;
}