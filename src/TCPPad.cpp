#include "TCPPad.h"

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

TCPPad::TCPPad(const char *device)
{
    cout << "TCPPad initialized" << endl;
    m_gp = open(device, O_RDONLY);

    if (m_gp == -1)
        perror("Could not open joystick");
};

TCPPad::~TCPPad()
{
    cout << "Calling TCPPad destructor" << endl;
    mThread.join();
    close(m_gp);
}

void TCPPad::startThread()
{
    cout << "running thread" << endl;
    mThread = std::thread(&TCPPad::run_atomic, this);
}

void TCPPad::run()
{
    struct js_event event;
    m_bs = 0;

    while (readEvent(m_gp, &event) == 0)
    {
        if (event.type == JS_EVENT_BUTTON)
        {
            lock_guard<mutex> lock(mMutex);
            // printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
            m_bs += 2 * (event.value - 0.5) * pow(2, event.number);
        }
        if (event.type == JS_EVENT_AXIS)
        {
            lock_guard<mutex> lock(mMutex);
            size_t axis = event.number / 2;

            if (axis < 3)
            {
                if (event.number % 2 == 0)
                    m_jsx = event.value;
                else
                    m_jsy = event.value;
            }
        }
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void TCPPad::run_atomic()
{
    struct js_event event;
    m_bs = atomic_int(0);
    m_jsx = 0;
    m_jsy = 0;

    while (readEvent(m_gp, &event) == 0)
    {
        if (event.type == JS_EVENT_BUTTON)
        {
            if (event.value == 1){
                atomic_fetch_add(&m_bsa, pow(2, event.number));
            } else {
                atomic_fetch_sub(&m_bsa, pow(2, event.number));
            }
            fflush(stdout);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        if (event.type == JS_EVENT_AXIS)
        {
            lock_guard<mutex> lock(mMutex);
            size_t axis = event.number / 2;
            size_t xy = event.number % 2;

            if (axis < 3)
            {
                if (xy % 2 == 0)
                    m_jsx = event.value;
                else
                    m_jsy = event.value;
            }
            // cout << axis << " ==> " << xy << " | " << m_jsx << ", " << m_jsy << endl;
            fflush(stdout);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
    }
}

int TCPPad::readEvent(int fd, struct js_event *event)
{
    ssize_t bytes;
    bytes = read(fd, event, sizeof(*event));
    if (bytes == sizeof(*event))
        return 0;
    return -1;
}

size_t TCPPad::getAxisState(struct js_event *event, struct axis_state axes[3])
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

int TCPPad::getAxisX()
{
    return m_jsx;
}

int TCPPad::getAxisY()
{
    return m_jsy;
}

int TCPPad::getButtonState()
{
    return m_bs;
}

int TCPPad::getButtonStateAtomic()
{
    int bs = m_bsa.load(std::memory_order_relaxed);
    return bs;
}

void TCPPad::get_action()
{
    rapidjson::Value action(m_bs);
    rapidjson::Document document;
    document.SetObject();
    rapidjson::Document::AllocatorType &allocator = document.GetAllocator();
    document.AddMember("action", action, allocator);
    rapidjson::StringBuffer strbuf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
    document.Accept(writer);
}