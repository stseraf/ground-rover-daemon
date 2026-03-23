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

inline void print_prefix()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    static long sec_diff = 0;
    if (!sec_diff) sec_diff = ts.tv_sec;
    long seconds = ts.tv_sec - sec_diff;
    int  millis  = static_cast<int>(ts.tv_nsec / 1000000);
    std::printf("[%05ld.%03d] ", seconds, millis);
}

} // namespace detail

inline void line(const char *format, ...)
{
    if (detail::last_was_inline()) std::printf("\n");
    detail::last_was_inline() = false;

    detail::print_prefix();

    va_list args;
    va_start(args, format);
    std::vprintf(format, args);
    va_end(args);

    std::printf("\n");
    std::fflush(stdout);
}

inline void same_line(const char *format, ...)
{
    std::printf("\r");
    detail::print_prefix();
    std::printf("*");

    va_list args;
    va_start(args, format);
    std::vprintf(format, args);
    va_end(args);

    std::fflush(stdout);
    detail::last_was_inline() = true;
}

} // namespace logger
