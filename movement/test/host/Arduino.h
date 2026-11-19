#pragma once
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>

inline uint32_t hostTime = 0;
inline uint32_t millis() { return hostTime; }
inline void delay(unsigned ms) { hostTime += ms; }
struct HostSerial {
    unsigned printDelayMs = 0; // Simulate blocking diagnostic output when testing deadlines.
    std::deque<char> input;
    std::string output;
    void begin(int) {}
    int available() { return int(input.size()); }
    int read() { char c = input.front(); input.pop_front(); return c; }
    void println(const char* s) { output += s; output += '\n'; hostTime += printDelayMs; }
    template<class... Args> void printf(const char* fmt, Args... args) {
        char b[1024]; std::snprintf(b, sizeof(b), fmt, args...); output += b;
        hostTime += printDelayMs;
    }
};
inline HostSerial Serial;

