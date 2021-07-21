#include "grbl.h"

status_code_t simple_execute_block (char *block, char *message) {
    // TODO store one block of raw steps in buffer
}

ISR_CODE void simple_driver_interrupt_handler (void)
{
    // TODO execute one block of raw steps
}
