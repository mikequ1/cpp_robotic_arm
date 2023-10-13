#pragma once

#include <queue>
#include <thread>
#include <mutex>

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

    /**
     * @brief send a message via socket
     * 
     * @param payload 
     */
    void send_data(const char* payload);

    /**
     * @brief starting a new thread to accept incoming socket communications
     * 
     */
    void startThread();
    
    /**
     * @brief routine run inside thread
     * 
     */
    void receiveLoop();

    bool get_command(std::string& buffer);

    /**
     * @brief receive data from the other end of the TCP socket
     * 
     * @param buffer buffer containing the read communication
     * @return int status of the read data
     */
    int receive_data(char* buffer);

    std::queue<std::string>& get_q();

private:
    /// @brief File descriptor of the socket server
    int m_server_fd;
    /// @brief socket
    int m_socket;
    /// @brief queue of incoming communications
    std::queue<std::string> m_q;

    std::thread mThread;

    std::mutex mMutex;

    std::mutex qMutex;
    

};