#include "Comms.h"

#include <netinet/in.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 
#include <unistd.h> 

using namespace std;

Comms::Comms(int port) {
    struct sockaddr_in address; 
    int addrlen = sizeof(address); 
    int opt = 1; 
  
    // Creating socket file descriptor 
    if ((m_server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    } 
  
    // Forcefully attaching socket to the port 8080 
    if (setsockopt(m_server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons(port); 
  
    // Forcefully attaching socket to the port 8080 
    if (bind(m_server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
    if (listen(m_server_fd, 3) < 0) { 
        perror("listen"); 
        exit(EXIT_FAILURE); 
    } 
    if ((m_socket = accept(m_server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) { 
        perror("accept"); 
        exit(EXIT_FAILURE); 
    } 
}

Comms::~Comms() {
    mThread.join();
    close(m_socket); 
    shutdown(m_server_fd, SHUT_RDWR); 
}

void Comms::send_data(const char* payload) {
    lock_guard<mutex> lock(mMutex);
    send(m_socket, payload, strlen(payload), 0); 
}

void Comms::startThread() {
    mThread = thread(&Comms::receiveLoop, this);
}

void Comms::receiveLoop() {
    while (true) {
        char buffer[1024] = { 0 }; 
        int val = read(m_socket, buffer, 1024);
        if (val > 0) {
            lock_guard<mutex> lock(qMutex);
            m_q.push(string(buffer));
        }
    }
}

bool Comms::get_command(string& buffer) {
    lock_guard<mutex> lock(qMutex);
    if (m_q.size() == 0) {
        return -1;
    }
    buffer = m_q.front();
    m_q.pop();
    return 0;
}

int Comms::receive_data(char* buffer) {
    int val = read(m_socket, buffer, 1024);
    return val;
}

queue<string>& Comms::get_q() {
    return m_q;
}