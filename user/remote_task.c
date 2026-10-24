#include "cmsis_os2.h"

#include "remote.h"
#include "user_uart.h"

#define REMOTE_SEMAPHORE_WAIT_MS 10U
#define REMOTE_LINK_TIMEOUT_MS   100U

extern osSemaphoreId_t RemoteSemHandle;

void Remote_Task(void *argument)
{
    (void)argument;

    remote_clear();
    remote_uart_init();

    for (;;)
    {
        if (osSemaphoreAcquire(RemoteSemHandle, REMOTE_SEMAPHORE_WAIT_MS) == osOK)
        {
            remote_uart_process();
        }

        if (remote_uart_is_timeout(REMOTE_LINK_TIMEOUT_MS))
        {
            remote_clear();
            remote_uart_clear_state();
        }
    }
}
