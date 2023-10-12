#include "Comms.h"

#include <netinet/in.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 
#include <unistd.h> 

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
    printf("Destructing Comms socket");
    close(m_socket); 
    shutdown(m_server_fd, SHUT_RDWR); 
}

void Comms::send_data(const char* payload) {
    send(m_socket, payload, strlen(payload), 0); 
}

void Comms::receive_data() {
    char buffer[1024] = { 0 }; 
    read(m_socket, buffer, 1024);
    printf(buffer);
}