#pragma once
#include "FreeRTOS.h"
inline unsigned hostTaskCreates=0;
inline void (*hostCreatedTask)(void*)=nullptr;
inline int xTaskCreatePinnedToCore(void (*task)(void*),const char*,unsigned,void*,unsigned,void*,unsigned) {
    ++hostTaskCreates; hostCreatedTask=task; return pdPASS;
}
inline void vTaskDelay(uint32_t) {}
