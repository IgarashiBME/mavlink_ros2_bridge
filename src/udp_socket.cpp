#include "mavlink_ros2_bridge/udp_socket.hpp"

#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>

UDPSocket::UDPSocket() = default;

UDPSocket::~UDPSocket()
{
    close();
}

bool UDPSocket::open(uint16_t local_port, const std::string& remote_ip, uint16_t remote_port)
{
    fd_ = ::socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd_ < 0) {
        return false;
    }

    // Bind to local port for receiving
    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(local_port);

    if (::bind(fd_, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Set non-blocking mode
    if (::fcntl(fd_, F_SETFL, O_NONBLOCK) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Configure remote (GCS) address for sending
    std::memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_addr.s_addr = ::inet_addr(remote_ip.c_str());
    remote_addr_.sin_port = htons(remote_port);

    return true;
}

void UDPSocket::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool UDPSocket::is_open() const
{
    return fd_ >= 0;
}

ssize_t UDPSocket::send(const uint8_t* data, size_t length)
{
    if (fd_ < 0) {
        return -1;
    }
    return ::sendto(fd_, data, length, 0,
                    reinterpret_cast<const sockaddr*>(&remote_addr_),
                    sizeof(remote_addr_));
}

ssize_t UDPSocket::receive(uint8_t* buffer, size_t max_length)
{
    if (fd_ < 0) {
        return -1;
    }
    sockaddr_in sender_addr{};
    socklen_t from_len = sizeof(sender_addr);
    ssize_t n = ::recvfrom(fd_, buffer, max_length, 0,
                           reinterpret_cast<sockaddr*>(&sender_addr),
                           &from_len);
    if (n > 0) {
        remote_addr_ = sender_addr;
    }
    return n;
}

std::string UDPSocket::remote_endpoint() const
{
    char ip_str[INET_ADDRSTRLEN];
    ::inet_ntop(AF_INET, &remote_addr_.sin_addr, ip_str, sizeof(ip_str));
    return std::string(ip_str) + ":" + std::to_string(ntohs(remote_addr_.sin_port));
}
