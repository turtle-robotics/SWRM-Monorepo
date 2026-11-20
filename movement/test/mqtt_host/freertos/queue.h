#pragma once
#include "FreeRTOS.h"
#include <cstring>
#include <deque>
#include <vector>
struct HostQueue { unsigned capacity, itemSize; std::deque<std::vector<uint8_t>> values; };
using QueueHandle_t=HostQueue*;
inline QueueHandle_t xQueueCreate(unsigned capacity,unsigned itemSize) { return new HostQueue{capacity,itemSize,{}}; }
inline int xQueueOverwrite(QueueHandle_t queue,const void* value) {
    if (!queue) return 0;
    queue->values.clear();
    const auto* bytes=static_cast<const uint8_t*>(value);
    queue->values.emplace_back(bytes,bytes+queue->itemSize); return pdPASS;
}
inline int xQueueReceive(QueueHandle_t queue,void* value,unsigned) {
    if (!queue || queue->values.empty()) return 0;
    std::memcpy(value,queue->values.front().data(),queue->itemSize); queue->values.pop_front(); return pdPASS;
}
inline void vQueueDelete(QueueHandle_t queue) { delete queue; }
