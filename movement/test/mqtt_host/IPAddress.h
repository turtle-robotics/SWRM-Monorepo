#pragma once
#include <cstdint>
class IPAddress {
public:
    uint8_t bytes[4]={};
    IPAddress()=default;
    IPAddress(uint8_t a,uint8_t b,uint8_t c,uint8_t d):bytes{a,b,c,d} {}
};
