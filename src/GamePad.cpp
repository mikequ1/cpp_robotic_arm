#include "GamePad.h"

#include <iostream>
#include <cmath>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

using namespace std;

GamePad::GamePad(const char *device, Comms* c)
{
    m_c = c;
    cout << "GamePad initialized" << endl;
    m_gp = open(device, O_RDONLY);

    if (m_gp == -1)
        perror("Could not open joystick");
};

GamePad::~GamePad()
{
    cout << "Calling GamePad destructor" << endl;
    mThread.join();
    close(m_gp);
}

void GamePad::startThread()
{
    cout << "running thread" << endl;
    mThread = std::thread(&GamePad::run, this);
}

void GamePad::run()
{
    struct js_event event;
    struct axis_state axes[3] = {0};
    size_t axis;
    m_bs = 0;

    while (readEvent(m_gp, &event) == 0)
    {
        if (event.type == JS_EVENT_BUTTON)
        {
            lock_guard<mutex> lock(mMutex);
            // printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
            m_bs += 2 * (event.value - 0.5) * pow(2, event.number);
            cout << m_bs << endl;
        }
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

int GamePad::readEvent(int fd, struct js_event *event)
{
    ssize_t bytes;
    bytes = read(fd, event, sizeof(*event));
    if (bytes == sizeof(*event))
        return 0;
    return -1;
}

size_t GamePad::getAxisState(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;
    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }
    return axis;
}

int GamePad::getButtonState()
{
    return m_bs;
}

void GamePad::get_action()
{
    rapidjson::Document document;
    document.SetObject();
    rapidjson::Document::AllocatorType &allocator = document.GetAllocator();
    document.AddMember("action", m_bs, allocator);
    rapidjson::StringBuffer strbuf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
    document.Accept(writer);

    m_c->send_data(strbuf.GetString());
}