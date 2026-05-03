// file: main.c

#include "esp_attr.h"

#include "comm.h"
#include "stepgen.h"

void IRAM_ATTR app_main(void) {
    start_stepgen_task();
    comm_loop();
}
