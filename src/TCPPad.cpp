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

// Networking libraries
#include <unistd.h>
#include <netinet/in.h>
#include <netinet/tcp.h> // for disabling nagle buffering
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>

using namespace std;

TCPPad::TCPPad(const char *ip)
{
    //int sockfd;
    if ((tcp_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation failed.");
    }

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));

    int port = 7879;
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    servaddr.sin_addr.s_addr = inet_addr(ip);
    // Connect to the server
    if (connect(tcp_fd, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0) {
        close(tcp_fd);
        perror("Failed to connect to socket.");
    }
    cout << "TCPPad initialized" << " at " << tcp_fd << endl;
};

TCPPad::~TCPPad()
{
    cout << "Calling TCPPad destructor" << endl;
    mThread.join();
    close(tcp_fd);
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

    while (!is_shutdown)
    {
        //cout << "run_atomic" << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        int bytes = recv(tcp_fd, &buf[buf_pos], sizeof(buf) - buf_pos, 0);
        if (bytes < 0)
        {
            // if no data is available to be read.
            cout << "Error: " << strerror(errno) << endl;
            continue;
        }
        size_t total_bytes = (bytes + buf_pos);
        size_t nDouble = total_bytes / __SIZEOF_DOUBLE__;
        if (nDouble < 2)
        {
            continue;
        }
        // Grab most recent pair of doubles. Ignore previous values.
        size_t start = 2 * (nDouble / 2) - 2;
        double * double_ptr = (double *) &buf[0];
        double jsx_d = double_ptr[start];
        double jsy_d = double_ptr[start + 1];
        double SCALE = 30000; //32768;
        //double SCALE = 16384;
        m_jsx = (int)(SCALE*jsx_d);
        m_jsy = (int)(SCALE*jsy_d);
        //cout << m_jsx << "," << m_jsy << endl;

        size_t bytes_used = (start + 2) * __SIZEOF_DOUBLE__;
        // Move the unused bytes to the beginning of the buffer.
        buf_pos = (int)total_bytes - (int)bytes_used;
        for(int i=0; i < buf_pos; ++i) 
        {
            buf[i] = buf[i+bytes_used];
        }
    }
    
    //int m_jsx;
    //int m_jsy;

    // while (readEvent(m_gp, &event) == 0)
    // {
    //     if (event.type == JS_EVENT_BUTTON)
    //     {
    //         if (event.value == 1){
    //             atomic_fetch_add(&m_bsa, pow(2, event.number));
    //         } else {
    //             atomic_fetch_sub(&m_bsa, pow(2, event.number));
    //         }
    //         fflush(stdout);
    //         std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //         continue;
    //     }
    //     if (event.type == JS_EVENT_AXIS)
    //     {
    //         lock_guard<mutex> lock(mMutex);
    //         size_t axis = event.number / 2;
    //         size_t xy = event.number % 2;

    //         if (axis < 3)
    //         {
    //             if (xy % 2 == 0)
    //                 m_jsx = event.value;
    //             else
    //                 m_jsy = event.value;
    //         }
    //         // cout << axis << " ==> " << xy << " | " << m_jsx << ", " << m_jsy << endl;
    //         fflush(stdout);
    //         std::this_thread::sleep_for(std::chrono::milliseconds(2));
    //         continue;
    //     }
    // }
    return;
}

int TCPPad::readEvent(int fd, struct js_event *event)
{
    ssize_t bytes;
    bytes = read(fd, event, sizeof(*event));
    if (bytes == sizeof(*event))
        return 0;
    return -1;
}

// size_t TCPPad::getAxisState(struct js_event *event, struct axis_state axes[3])
// {
//     size_t axis = event->number / 2;
//     if (axis < 3)
//     {
//         if (event->number % 2 == 0)
//             axes[axis].x = event->value;
//         else
//             axes[axis].y = event->value;
//     }
//     return axis;
// }

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

void TCPPad::shutdown()
{
    is_shutdown = 1;
}