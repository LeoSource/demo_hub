#include <iostream>
#include <thread>
// #include <unistd.h>
#include "spdlog_comm.h"
#include "class_test.h"

void thread1()
{
    SPD_WARN("warn1");
    SPD_WARN("warn2");
    SPD_WARN("warn3");
}

int main(int argc, char **argv) 
{
    RTLog rt_log;

    rt_log.warn(__FILENAME__,__FUNCTION__,__LINE__,"warn0");
    rt_log.warn(__FILENAME__,__FUNCTION__,__LINE__,"warn1");
    rt_log.warn(__FILENAME__,__FUNCTION__,__LINE__,"warn2");

    while (true)
    {
        rt_log.log();
        // sleep(1);
    }
    
    return 0;
}
