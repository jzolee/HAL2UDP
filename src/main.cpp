#include "FastDIO.h"

#include <Arduino.h>

#include "comm.h"

#include "stepgen.h"

void setup()
{
    startMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(comm_task, "comm_task", 2048, NULL, 2, &comm_task_handle, 0);
    xTaskCreatePinnedToCore(watchdog_task, "watchdog_task", 2048, NULL, 1, NULL, 0);
    xSemaphoreTake (startMutex, portMAX_DELAY);
    xTaskCreatePinnedToCore(stepgen_task, "stepgen_task", 2048, NULL, 1, NULL, 1);
    vTaskDelete(NULL);
}

void loop() { }
