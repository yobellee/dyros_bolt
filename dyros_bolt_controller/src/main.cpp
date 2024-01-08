#include <ros/ros.h>
#include "dyros_bolt_controller/mujoco_interface.h"
#include "dyros_bolt_controller/real_robot_interface.h"


#include "dyros_bolt_controller/dyros_bolt_controller.h"


using namespace dyros_bolt_controller;

#include <math.h>

#include <signal.h>

#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <ctime>

volatile bool *prog_shutdown;

void SIGINT_handler(int sig)
{
    std::cout << " CNTRL : shutdown Signal" << std::endl;
    *prog_shutdown = true;
}

int main(int argc, char **argv)
{
    std::cout << std::endl
         << "=====================================" << std::endl;
    std::cout << " CNTRL : Starting DYROS BOLT CONTROLLER! " << std::endl;

#ifdef COMPILE_REALROBOT
    mlockall(MCL_CURRENT | MCL_FUTURE);
#endif

    ros::init(argc, argv, "dyros_bolt_controller", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");

    std::string mode;
    nh.param<std::string>("run_mode", mode, "simulation");
    ControlBase *ctr_obj;
    ROS_INFO("!!!!!!!");
    

    double Hz;
    nh.param<double>("control_frequency", Hz, 200.0);

    if(mode == "simulation")
    {
        ROS_INFO("DYROS BOLT MAIN CONTROLLER - !!! MUJOCO SIMULATION MODE !!!");
        ctr_obj = new mujoco_interface(nh, Hz);
    }
    else if(mode == "real_robot")
    {
        ROS_INFO("DYROS BOLT MAIN CONTROLLER - !!! REAL ROBOT MODE !!!");
        ctr_obj = new RealRobotInterface(nh, Hz);
    }
    else
    {
        ROS_FATAL("Please choose simulation or real_robot");
    }

    while(ros::ok())
    {
        ctr_obj->readDevice();
        ctr_obj->update();
        ctr_obj->compute();
        ctr_obj->reflect();
        ctr_obj->writeDevice();
        ctr_obj->wait();

        if(ctr_obj->isShuttingDown())
        {
          break;
        }
    }

    delete ctr_obj;

    return 0;
}


