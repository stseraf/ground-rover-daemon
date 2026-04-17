#include "link_switcher.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "config.hpp"
#include "logger.hpp"

bool LinkSwitcher::send(const char* cmd)
{
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        logger::line("[link_switch] socket(): %s", ::strerror(errno));
        return false;
    }

    ::fcntl(sock, F_SETFL, ::fcntl(sock, F_GETFL) | O_NONBLOCK);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(Config::Lte::SWITCH_PORT);
    ::inet_pton(AF_INET, Config::Lte::STATUS_HOST, &addr.sin_addr);

    int rc = ::connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (rc < 0 && errno != EINPROGRESS) {
        logger::line("[link_switch] connect(%s:%u): %s",
                     Config::Lte::STATUS_HOST, Config::Lte::SWITCH_PORT, ::strerror(errno));
        ::close(sock);
        return false;
    }

    if (rc != 0) {
        struct pollfd pfd{sock, POLLOUT, 0};
        int prc = ::poll(&pfd, 1, 3000);
        if (prc <= 0) {
            logger::line("[link_switch] connect timeout to %s:%u",
                         Config::Lte::STATUS_HOST, Config::Lte::SWITCH_PORT);
            ::close(sock);
            return false;
        }
        int err = 0; socklen_t errlen = sizeof(err);
        ::getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &errlen);
        if (err != 0) {
            logger::line("[link_switch] connect error: %s", ::strerror(err));
            ::close(sock);
            return false;
        }
    }

    // Send "wifi\n" or "lte\n" and close — no response expected
    char buf[16]{};
    int len = ::snprintf(buf, sizeof(buf), "%s\n", cmd);
    ::fcntl(sock, F_SETFL, ::fcntl(sock, F_GETFL) & ~O_NONBLOCK);
    ::send(sock, buf, static_cast<size_t>(len), 0);
    ::close(sock);

    logger::line("[link_switch] sent '%s' to modem", cmd);
    return true;
}
