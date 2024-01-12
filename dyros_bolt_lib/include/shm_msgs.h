#ifndef SHM_MSGS_H
#define SHM_MSGS_H

#include <pthread.h>
#include <atomic>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <time.h>
#include <iostream>

#if defined(__x86_64) || defined(__i386)
#define cpu_relax() __asm__("pause" :: \
                                : "memory")
#else
#define cpu_relax() __asm__("" :: \
                                : "memory")
#endif

//per link
//Jac * 4
//33 * 6 * 39 * 4

// #define MODEL_DOF 6

typedef struct SHMmsgs
{

    float pos_virtual[4]; //virtual pos(3) + virtual quat(4)
    float vel_virtual[3]; //virtual vel(3) + virtual twist(3)
    float imu_acc[3];

    std::atomic<bool> imuWriting;


    int imu_state;

    //command val

    std::atomic<int> process_num;
    volatile bool shutdown; //true for exit

} SHMmsgs;

//static SHMmsgs *shm_msgs_;

static const key_t shm_msg_key = 10562;
static const key_t shm_rd_key = 10335;

// const std::string cred("\033[0;31m");
// const std::string creset("\033[0m");
// const std::string cblue("\033[0;34m");
// const std::string cgreen("\033[0;32m");
// const std::string cyellow("\033[0;33m");

static void init_shm(int shm_key, int &shm_id_, SHMmsgs **shm_ref)
{
    if ((shm_id_ = shmget(shm_key, sizeof(SHMmsgs), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm mtx failed\n";
        exit(0);
    }
    if ((*shm_ref = (SHMmsgs *)shmat(shm_id_, NULL, 0)) == (SHMmsgs *)-1)
    {
        std::cout << "shmat failed\n";
        exit(0);
    }

    if ((*shm_ref)->process_num == 0)
    {
        // printf("Process num 0 ! Clean Start!\n");
    }

    (*shm_ref)->process_num++;
}

static void init_shm2(int shm_key, int &shm_id_, SHMmsgs *shm_ref)
{
    if ((shm_id_ = shmget(shm_key, sizeof(SHMmsgs), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm mtx failed\n";
        exit(0);
    }
    if ((shm_ref = (SHMmsgs *)shmat(shm_id_, NULL, 0)) == (SHMmsgs *)-1)
    {
        std::cout << "shmat failed\n";
        exit(0);
    }

    if ((shm_ref)->process_num == 0)
    {
        // printf("Process num 0 ! Clean Start!\n");
    }

    (shm_ref)->process_num++;
}

static void deleteSharedMemory(int shm_id__, SHMmsgs *shm_ref)
{
    shm_ref->process_num--;
    if (shm_ref->process_num == 0)
    {
        printf("process num 0. removing shared memory\n");

        if (shmctl(shm_id__, IPC_RMID, NULL) == -1)
        {
            printf("shared memoty failed to remove. \n");
        }
        else
        {
            printf("Shared memory succesfully removed\n");
        }
    }
}

// static void init_shm_master()
// {

//     if ((shm_msg_id = shmget(shm_msg_key, sizeof(SHMmsgs), IPC_CREAT | 0666)) == -1)
//     {
//         std::cout << "shm mtx failed " << std::endl;
//         exit(0);
//     }

//     if ((shm_msgs_ = (SHMmsgs *)shmat(shm_msg_id, NULL, 0)) == (SHMmsgs *)-1)
//     {
//         std::cout << "shmat failed " << std::endl;
//         exit(0);
//     }

//     if (shmctl(shm_msg_id, SHM_LOCK, NULL) == 0)
//     {
//         //std::cout << "SHM_LOCK enabled" << std::endl;
//     }
//     else
//     {
//         std::cout << "SHM lock failed" << std::endl;
//     }

//     shm_msgs_->t_cnt = 0;
//     shm_msgs_->t_cnt2 = 0;
//     shm_msgs_->controllerReady = false;
//     shm_msgs_->statusWriting = 0;
//     shm_msgs_->commanding = false;
//     shm_msgs_->reading = false;
//     shm_msgs_->shutdown = false;

//     //
//     //float lat_avg, lat_min, lat_max, lat_dev;
//     //float send_avg, send_min, send_max, send_dev;

//     shm_msgs_->lat_avg2 = 0;
//     shm_msgs_->lat_min2 = 0;
//     shm_msgs_->lat_max2 = 100000;
//     shm_msgs_->lat_dev2 = 0;

//     shm_msgs_->send_avg2 = 0;
//     shm_msgs_->send_min2 = 0;
//     shm_msgs_->send_max2 = 100000;
//     shm_msgs_->send_dev2 = 0;

//     //std::cout << "shm master initialized" << std::endl;
// }

// void SendCommand(float *torque_command, float *position_command, int *mode)
// {
//     shm_msgs_->commanding = true;

//     shm_msgs_->commanding = false;
// }

#endif
