#include "usb_lte_monitor.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cerrno>

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "logger.hpp"
#include "config.hpp"

// ---------------------------------------------------------------------------
// Interface presence check
// ---------------------------------------------------------------------------

bool UsbLteMonitor::check_interface()
{
    // USB network adapters report operstate="unknown" even when working.
    // Read flags instead: IFF_UP (0x1) means the interface is administratively up.
    int fd = ::open(Config::Lte::IFACE_FLAGS_PATH, O_RDONLY);
    if (fd < 0)
        return false;

    char buf[16]{};
    ssize_t n = ::read(fd, buf, sizeof(buf) - 1);
    ::close(fd);

    if (n <= 0)
        return false;

    unsigned long flags = ::strtoul(buf, nullptr, 16);
    return (flags & 0x1UL) != 0;  // IFF_UP
}

// ---------------------------------------------------------------------------
// Signal quality — connect to lte_status_srv.sh TCP daemon on modem
// ---------------------------------------------------------------------------

bool UsbLteMonitor::fetch_signal()
{
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
        return false;

    // Non-blocking connect with 3 s timeout via poll().
    ::fcntl(sock, F_SETFL, ::fcntl(sock, F_GETFL) | O_NONBLOCK);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(Config::Lte::STATUS_PORT);
    ::inet_pton(AF_INET, Config::Lte::STATUS_HOST, &addr.sin_addr);

    int rc = ::connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (rc < 0 && errno != EINPROGRESS) {
        ::close(sock);
        return false;
    }

    if (rc != 0) {
        struct pollfd pfd{sock, POLLOUT, 0};
        int prc = ::poll(&pfd, 1, 3000);
        if (prc <= 0) {
            ::close(sock);
            return false;
        }
        int err = 0;
        socklen_t errlen = sizeof(err);
        ::getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &errlen);
        if (err != 0) {
            ::close(sock);
            return false;
        }
    }

    // Blocking read with 2 s timeout.
    ::fcntl(sock, F_SETFL, ::fcntl(sock, F_GETFL) & ~O_NONBLOCK);
    struct timeval tv{2, 0};
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    char line[256]{};
    int total = 0;
    while (total < static_cast<int>(sizeof(line)) - 1) {
        ssize_t n = ::recv(sock, line + total, sizeof(line) - 1 - total, 0);
        if (n <= 0) break;
        total += static_cast<int>(n);
        if (line[total - 1] == '\n') break;
    }
    ::close(sock);

    if (total == 0)
        return false;

    // Parse "key=value" pairs separated by spaces.
    // Format: connected=1 rssi=15 netmode=LTE oper=KYIVSTAR
    char* p = line;
    while (*p && *p != '\n') {
        // Skip spaces
        while (*p == ' ') ++p;
        if (!*p || *p == '\n') break;

        // Find key end
        char* key = p;
        while (*p && *p != '=' && *p != ' ' && *p != '\n') ++p;
        if (*p != '=') break;
        *p++ = '\0';

        // Find value end
        char* val = p;
        while (*p && *p != ' ' && *p != '\n') ++p;
        char saved = *p;
        *p = '\0';

        if (::strcmp(key, "connected") == 0) {
            status_.connected = (val[0] == '1');
        } else if (::strcmp(key, "rssi") == 0) {
            status_.rssi = static_cast<uint8_t>(::atoi(val));
        } else if (::strcmp(key, "netmode") == 0) {
            ::strncpy(status_.netmode, val, sizeof(status_.netmode) - 1);
            status_.netmode[sizeof(status_.netmode) - 1] = '\0';
        } else if (::strcmp(key, "oper") == 0) {
            ::strncpy(status_.oper, val, sizeof(status_.oper) - 1);
            status_.oper[sizeof(status_.oper) - 1] = '\0';
        }

        *p = saved;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Traffic stats — read /proc/net/dev for usb0 RX/TX byte counters
// ---------------------------------------------------------------------------

bool UsbLteMonitor::fetch_traffic()
{
    FILE* f = ::fopen(Config::Lte::PROC_NET_DEV, "r");
    if (!f)
        return false;

    char line[256];
    while (::fgets(line, sizeof(line), f)) {
        const char* p = line;
        while (*p == ' ') ++p;
        if (::strncmp(p, "usb0:", 5) != 0) continue;
        p += 5;

        char* end;
        status_.rx_bytes = ::strtoull(p, &end, 10);
        // Skip 7 fields: rx_packets, errs, drop, fifo, frame, compressed, multicast
        for (int i = 0; i < 7; i++) ::strtoull(end, &end, 10);
        status_.tx_bytes = ::strtoull(end, nullptr, 10);

        ::fclose(f);
        return true;
    }
    ::fclose(f);
    return false;
}

// ---------------------------------------------------------------------------
// Public update() — called from main loop every LTE_POLL_INTERVAL_US
// ---------------------------------------------------------------------------

void UsbLteMonitor::update()
{
    bool was_present   = status_.present;
    bool was_connected = status_.connected;
    uint8_t prev_rssi  = status_.rssi;

    status_.present = check_interface();

    if (!status_.present) {
        if (was_present)
            logger::line("[lte] modem disconnected (usb0 down)");
        status_.connected  = false;
        status_.rssi       = 0;
        status_.netmode[0] = '\0';
        status_.oper[0]    = '\0';
        status_.rx_bytes   = 0;
        status_.tx_bytes   = 0;
        return;
    }

    if (!was_present)
        logger::line("[lte] modem detected (usb0 up)");

    fetch_traffic();

    if (!fetch_signal()) {
        if (was_connected)
            logger::line("[lte] status daemon unreachable — treating as disconnected");
        else
            logger::line("[lte] status daemon not ready yet — retrying next poll");
        status_.connected = false;
        status_.rssi      = 0;
        return;
    }

    if (status_.connected != was_connected) {
        if (status_.connected)
            logger::line("[lte] connected — oper=%s netmode=%s rssi=%u",
                         status_.oper, status_.netmode, status_.rssi);
        else
            logger::line("[lte] disconnected");
    } else if (status_.connected) {
        if (status_.rssi != prev_rssi)
            logger::line("[lte] oper=%s netmode=%s rssi=%u→%u",
                         status_.oper, status_.netmode, prev_rssi, status_.rssi);
    }
}
