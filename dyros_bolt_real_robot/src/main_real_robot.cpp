
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

    struct sched_param param;
    pthread_attr_t attr;
    pthread_t thread1;
    int ret;

    initDyrosBoltArgs(init_args);
    
    bool init_result = initDyrosBoltSystem(init_args);
    if (!init_result)
    {
        printf("[CAN - ERRO] init failed\n");
        return -1;
    }
    /* Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr);
    if (ret)
    {
        printf("init pthread attributes failed\n");
        return ret;
    }

    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret)
    {
        printf("pthread setschedpolicy failed\n");
        return ret;
    }
    param.sched_priority = 46+50;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret)
    {
        printf("pthread setschedparam failed\n");
        return ret;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(6, &cpuset);

    ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
    if (ret)
    {
        printf("pthread setaffinity failed\n");
        return ret;
    }

    /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret)
    {
        printf("pthread setinheritsched failed\n");
        return ret;
    }    

    ret = pthread_create(&thread1, &attr, ethercatThread1, &init_args);
    if (ret)
    {
        printf("create pthread 1 failed\n");
        return ret;
    }
    
    
    pthread_attr_destroy(&attr);


    ret = pthread_join(thread1, NULL);
    if (ret)
        printf("join pthread failed: %m\n");

        

    // ethercatThread1(&init_args);
    std::cout << "test5" << std::endl;

    printf("[CAN - INFO] cleaning up\n");
    cleanupDyrosBoltSystem();
    return 0;
}
