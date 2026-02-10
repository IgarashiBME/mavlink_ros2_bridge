#pragma once

#include <cstdint>
#include <string>
#include <netinet/in.h>

#include <common/mavlink.h>

class UDPSocket {
public:
    UDPSocket();
    ~UDPSocket();

    UDPSocket(const UDPSocket&) = delete;
    UDPSocket& operator=(const UDPSocket&) = delete;

    bool open(uint16_t local_port, const std::string& remote_ip, uint16_t remote_port);
    void close();
    bool is_open() const;

    ssize_t send(const uint8_t* data, size_t length);
    ssize_t receive(uint8_t* buffer, size_t max_length);

private:
    int fd_ = -1;
    sockaddr_in remote_addr_{};
};
