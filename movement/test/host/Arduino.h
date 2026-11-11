#pragma once
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>

inline uint32_t hostTime = 0;
inline uint32_t millis() { return hostTime; }
inline void delay(unsigned ms) { hostTime += ms; }
struct HostSerial {
    std::deque<char> input;
    std::string output;
    void begin(int) {}
    int available() { return int(input.size()); }
    int read() { char c = input.front(); input.pop_front(); return c; }
    void println(const char* s) { output += s; output += '\n'; }
    template<class... Args> void printf(const char* fmt, Args... args) {
        char b[1024]; std::snprintf(b, sizeof(b), fmt, args...); output += b;
    }
};
inline HostSerial Serial;

