#pragma once

#include <queue>
#include <thread>

class Comms {
public:
    /**
     * @brief Construct a new Comms object
     * 
     * @param port socket port number
     */
    Comms(int port);

    /// @brief Destructor
    virtual ~Comms();

    void send_data(const char* payload);

    void startThread();
    void receiveLoop();

    int receive_data(char* buffer);

    std::queue<char*>& get_q();

private:
    /// @brief File descriptor of the socket server
    int m_server_fd;
    /// @brief socket
    int m_socket;
    /// @brief queue of incoming communications
    std::queue<char*> m_q;

    std::thread mThread;
    

};