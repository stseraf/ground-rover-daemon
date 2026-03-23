#pragma once

#include <cstdio>
#include <cstdarg>
#include <ctime>

namespace logger {
namespace detail {

inline bool& last_was_inline()
{
    static bool v = false;
    return v;
}

inline char* inline_buf()
{
    static char buf[320] = {};
    return buf;
}

inline bool& has_inline()
{
    static bool v = false;
    return v;
}

inline void fill_prefix(char* out, size_t cap)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    static long sec_diff = 0;
    if (!sec_diff) sec_diff = ts.tv_sec;
    long seconds = ts.tv_sec - sec_diff;
    int  millis  = static_cast<int>(ts.tv_nsec / 1000000);
    std::snprintf(out, cap, "[%05ld.%03d] ", seconds, millis);
}

inline void print_prefix()
{
    char tmp[32];
    fill_prefix(tmp, sizeof(tmp));
    std::fputs(tmp, stdout);
}

} // namespace detail

inline void line(const char *format, ...)
{
    if (detail::last_was_inline()) {
        // Erase the status line so the new message appears in its place
        std::printf("\r\033[K");
    }
    detail::last_was_inline() = false;

    detail::print_prefix();

    va_list args;
    va_start(args, format);
    std::vprintf(format, args);
    va_end(args);

    std::printf("\n");

    // Re-draw the status line below the new message
    if (detail::has_inline()) {
        std::printf("%s", detail::inline_buf());
        detail::last_was_inline() = true;
    }

    std::fflush(stdout);
}

inline void same_line(const char *format, ...)
{
    char* buf = detail::inline_buf();
    constexpr size_t cap = 320;

    char prefix[32];
    detail::fill_prefix(prefix, sizeof(prefix));
    size_t off = static_cast<size_t>(std::snprintf(buf, cap, "%s*", prefix));

    va_list args;
    va_start(args, format);
    std::vsnprintf(buf + off, cap - off, format, args);
    va_end(args);

    detail::has_inline() = true;
    std::printf("\r%s", buf);
    std::fflush(stdout);
    detail::last_was_inline() = true;
}

} // namespace logger
