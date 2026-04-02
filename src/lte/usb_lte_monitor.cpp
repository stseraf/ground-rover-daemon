#include "usb_lte_monitor.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include <fcntl.h>
#include <unistd.h>
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
// Raw HTTP/1.0 POST over a plain TCP socket
// ---------------------------------------------------------------------------

bool UsbLteMonitor::http_post(const char* body, const char* extra_headers,
                               char* resp_buf, int resp_buf_size)
{
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
        return false;

    // 2 s connect + I/O timeout — long enough for modem, short enough to not
    // stall the main loop noticeably.
    struct timeval tv{2, 0};
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(Config::Lte::API_PORT);
    ::inet_pton(AF_INET, Config::Lte::API_HOST, &addr.sin_addr);

    if (::connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(sock);
        return false;
    }

    int body_len = static_cast<int>(::strlen(body));

    char req[512];
    int req_len = ::snprintf(req, sizeof(req),
        "POST /ajax HTTP/1.0\r\n"
        "Host: %s\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "%s"
        "\r\n"
        "%s",
        Config::Lte::API_HOST, body_len,
        extra_headers ? extra_headers : "",
        body);

    if (req_len <= 0 || req_len >= static_cast<int>(sizeof(req))) {
        ::close(sock);
        return false;
    }

    ssize_t sent = ::send(sock, req, static_cast<size_t>(req_len), 0);
    if (sent != req_len) {
        ::close(sock);
        return false;
    }

    int total = 0;
    while (total < resp_buf_size - 1) {
        ssize_t n = ::recv(sock, resp_buf + total, static_cast<size_t>(resp_buf_size - 1 - total), 0);
        if (n <= 0)
            break;
        total += static_cast<int>(n);
    }
    resp_buf[total] = '\0';
    ::close(sock);
    return total > 0;
}

// ---------------------------------------------------------------------------
// Signal quality fetch (login + signal query)
// ---------------------------------------------------------------------------

bool UsbLteMonitor::fetch_signal()
{
    char resp[1024];

    // The modem tracks session by IP server-side (no cookie returned).
    // funcNo=1000 must be called first each time to initialise the session,
    // then funcNo=1001 returns signal data.
    static const char login_body[]  = "{\"funcNo\":1000,\"username\":\"admin\",\"password\":\"admin\"}";
    static const char signal_body[] = "{\"funcNo\":1001}";

    if (!http_post(login_body, nullptr, resp, sizeof(resp))) {
        logger::line("[lte] login HTTP failed");
        return false;
    }

    if (!http_post(signal_body, nullptr, resp, sizeof(resp))) {
        logger::line("[lte] signal HTTP failed");
        return false;
    }

    // Parse "rssi":<value> from the fixed-schema JSON response.
    // Example: {"results":[{"oper":"KYIVSTAR","netstatus":"Connected","netmode":"LTE","rssi":4}],...}
    const char* rssi_key = ::strstr(resp, "\"rssi\":");
    if (!rssi_key) {
        logger::line("[lte] signal response missing rssi — resp: %.120s", resp);
        return false;
    }
    rssi_key += ::strlen("\"rssi\":");
    status_.rssi = static_cast<uint8_t>(::atoi(rssi_key));

    // Parse "netmode":"<value>"
    const char* nm = ::strstr(resp, "\"netmode\":\"");
    if (nm) {
        nm += ::strlen("\"netmode\":\"");
        int i = 0;
        while (*nm && *nm != '"' && i < static_cast<int>(sizeof(status_.netmode)) - 1)
            status_.netmode[i++] = *nm++;
        status_.netmode[i] = '\0';
    }

    // Parse "oper":"<value>"
    const char* op = ::strstr(resp, "\"oper\":\"");
    if (op) {
        op += ::strlen("\"oper\":\"");
        int i = 0;
        while (*op && *op != '"' && i < static_cast<int>(sizeof(status_.oper)) - 1)
            status_.oper[i++] = *op++;
        status_.oper[i] = '\0';
    }

    // Parse "netstatus":"Connected"
    const char* ns_key = ::strstr(resp, "\"netstatus\":\"");
    char netstatus[16]{};
    if (ns_key) {
        ns_key += ::strlen("\"netstatus\":\"");
        int i = 0;
        while (*ns_key && *ns_key != '"' && i < static_cast<int>(sizeof(netstatus)) - 1)
            netstatus[i++] = *ns_key++;
        netstatus[i] = '\0';
    }
    status_.connected = ::strcmp(netstatus, "Connected") == 0;

    return true;
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
        return;
    }

    if (!was_present)
        logger::line("[lte] modem detected (usb0 up)");

    if (!fetch_signal()) {
        // API unreachable — treat as disconnected but keep present=true.
        logger::line("[lte] signal query failed — treating as disconnected");
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
