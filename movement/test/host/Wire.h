#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include <deque>
#include <array>
struct HostWire {
    bool ack = true;
    unsigned readLength = 12;
    unsigned readRequests = 0; // Proves the first-day PWM mode does not need encoder reads.
    int32_t encoder[4] = {};
    unsigned voltageMv = 5000, voltageReadLength = 2;
    int8_t motor[4] = {};
    uint8_t activeReg = 51;
    std::vector<uint8_t> tx;
    std::deque<uint8_t> rx;
    std::vector<std::vector<uint8_t>> writes;
    void begin(int, int) {}
    void setClock(int) {}
    void setTimeOut(int) {}
    void beginTransmission(uint8_t) { tx.clear(); }
    size_t write(uint8_t b) { tx.push_back(b); return 1; }
    size_t write(const uint8_t* b, size_t n) { tx.insert(tx.end(), b, b+n); return n; }
    int endTransmission(bool = true) {
        if (!ack) return 4;
        if (tx.size() > 1) writes.push_back(tx);
        if (tx.size() == 5 && (tx[0] == 31 || tx[0] == 51)) {
            activeReg = tx[0];
            for (int i = 0; i < 4; ++i) motor[i] = int8_t(tx[i+1]);
        }
        if (tx.size() == 2 && tx[0] >= 51 && tx[0] <= 54) {
            activeReg = 51;
            motor[tx[0]-51] = int8_t(tx[1]);
        }
        return 0;
    }
    int requestFrom(uint8_t, uint8_t requested) {
        ++readRequests;
        rx.clear();
        if (!ack) return 0;
        if (tx.size() == 1 && tx[0] == 0) {
            for (unsigned i = 0; i < voltageReadLength && i < requested && i < 2; ++i)
                rx.push_back(uint8_t(voltageMv >> (8*i)));
            return int(rx.size());
        }
        for (unsigned i = 0; i < readLength && i < requested && i < 16; ++i)
            rx.push_back(uint8_t(uint32_t(encoder[i/4]) >> (8*(i%4))));
        return int(rx.size());
    }
    int available() { return int(rx.size()); }
    int read() { int b = rx.front(); rx.pop_front(); return b; }
};
inline HostWire Wire;

