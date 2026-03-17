#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

class UdpSocket {
public:
    explicit UdpSocket(uint16_t bind_port)
    {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            std::perror("socket");
            std::exit(1);
        }

        sockaddr_in local{};
        local.sin_family      = AF_INET;
        local.sin_addr.s_addr = INADDR_ANY;
        local.sin_port        = htons(bind_port);

        if (::bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
            std::perror("bind");
            std::exit(1);
        }
    }

    ~UdpSocket() { if (fd_ >= 0) ::close(fd_); }

    UdpSocket(const UdpSocket&)            = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    ssize_t recv_nonblocking(uint8_t* buf, size_t len, sockaddr_in& from, socklen_t& from_len)
    {
        return ::recvfrom(fd_, buf, len, MSG_DONTWAIT,
                          reinterpret_cast<sockaddr*>(&from), &from_len);
    }

    ssize_t send_to(const uint8_t* buf, size_t len, const sockaddr_in& to, socklen_t to_len)
    {
        return ::sendto(fd_, buf, len, 0,
                        reinterpret_cast<const sockaddr*>(&to), to_len);
    }

private:
    int fd_{-1};
};
