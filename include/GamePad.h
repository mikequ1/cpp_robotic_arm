#pragma once

#include <thread>
#include <mutex>

#include "Comms.h"

struct axis_state
{
    short x, y;
};

class GamePad
{
public:
    /**
     * @brief Construct a new Game Pad object
     *
     * @param device Device path e.g. /dev/input/js0
     */
    GamePad(const char *device, Comms* c);

    /// @brief Destructor
    virtual ~GamePad();

    /**
     * @brief Start a new thread responsible for reading Gamepad inputs
     *
     */
    void startThread();

    /**
     * @brief Routine run by the thread
     *
     */
    void run();

    int getButtonState();

    void get_action();

private:
    /**
     * @brief read a current button event
     *
     * @param fd File descriptor of joystick
     * @param event event status obtained from joystick
     * @return 0 on success, -1 on failure
     */
    int readEvent(int fd, struct js_event *event);

    /**
     * @brief Get the Axis State object
     *
     * @param event event status obtained from the joystick
     * @param axes struct representing joystick position on each axis
     * @return size_t returns the axis that the event indicated.
     */
    size_t getAxisState(struct js_event *event, struct axis_state axes[3]);

    /// @brief socket comms object
    Comms* m_c;
    /// @brief Abstraction of a GamePad device
    int m_gp;
    /// @brief Button State represented by binary (XABY = [0000]) 1=pressed, 0=released
    int m_bs;
    /// @brief GamePad status reading thread
    std::thread mThread;
    /// @brief Mutex protecting the button state (XYAB)
    std::mutex mMutex;
};