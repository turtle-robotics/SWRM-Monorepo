#pragma once
#include "../host/Arduino.h"
#include <cstdlib>
#include <cstring>
#include <vector>
using byte=uint8_t;
using boolean=bool;
constexpr unsigned OUTPUT=1, HIGH=1, LOW=0;
inline int hostLed=-1;
inline unsigned hostLedPin=0;
inline void pinMode(unsigned, unsigned) {}
inline void digitalWrite(unsigned pin,unsigned state) { hostLedPin=pin; hostLed=int(state); }
inline void (*hostNetworkWaitHook)()=nullptr;
inline void yield() { ++hostTime; if (hostNetworkWaitHook) hostNetworkWaitHook(); }
#define pgm_read_byte_near(address) (*reinterpret_cast<const uint8_t*>(address))
class Print {
public:
    virtual ~Print()=default;
    virtual std::size_t write(uint8_t)=0;
    virtual std::size_t write(const uint8_t* data,std::size_t len) {
        for (std::size_t i=0;i<len;++i) write(data[i]);
        return len;
    }
};
