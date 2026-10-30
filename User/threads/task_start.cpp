#include "app_main.h"

#include "cmsis_os2.h"

extern "C" void TaskStart(void* argument)
{
    (void)argument;
    app_main();
    osThreadExit();
}
