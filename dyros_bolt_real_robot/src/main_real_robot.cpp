
#include "dyros_bolt_real_robot/odrive_socketcan_master.h"

#include "sys/mman.h"
#include <signal.h>


void SIGINT_handler(int sig)
{
    shutdownSystem();
}

int main(int argc, char **argv)
{
    mlockall(MCL_CURRENT | MCL_FUTURE);
    signal(SIGINT, SIGINT_handler);
    std::cout << "test1" << std::endl;
    
    
    DyrosBoltInitArgs init_args;
    
    int max_jnum = 6;
    init_args.q_start_ = max_jnum;
    init_args.can_slave_num = 6;
    init_args.is_main = true;

    initDyrosBoltArgs(init_args);
    
    bool init_result = initDyrosBoltSystem(init_args);
    if (!init_result)
    {
        printf("[CAN - ERRO] init failed\n");
        return -1;
    }

    ethercatThread1(&init_args);
    std::cout << "test5" << std::endl;

    printf("[CAN - INFO] cleaning up\n");
    cleanupDyrosBoltSystem();
    return 0;
}
