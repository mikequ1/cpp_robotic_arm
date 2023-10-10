#pragma once

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

    void receive_data();

private:
    /// @brief File descriptor of the socket server
    int m_server_fd;
    /// @brief socket
    int m_socket;

};