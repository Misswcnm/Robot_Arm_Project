#ifndef MACHINE_CONTROLLER_H
#define MACHINE_CONTROLLER_H

#include <string>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>

class MachineController {
public:
    MachineController(unsigned int port);
    ~MachineController();
    
    bool startServer();
    void stopServer();
    bool acceptConnection();
    std::string receivePosition();
    void sendStatus(const std::string& status);

private:
    unsigned int m_port;
    int m_listenSocket;
    int m_clientSocket;
    std::atomic<bool> m_running;
};

#endif // MACHINE_CONTROLLER_H