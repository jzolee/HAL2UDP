#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "globals.h"
#include "io.h"
#include "comm.h"
#include "stepgen.h"

void app_main(void)
{
    startMutex = xSemaphoreCreateMutex();
    io_init();
    xTaskCreatePinnedToCore(comm_task, "comm_task", 8192, NULL, 2, &comm_task_handle, 0);
    xTaskCreatePinnedToCore(watchdog_task, "watchdog_task", 2048, NULL, 1, NULL, 0);
    xSemaphoreTake(startMutex, portMAX_DELAY);
    xTaskCreatePinnedToCore(stepgen_task, "stepgen_task", 4096, NULL, 1, NULL, 1);
}
