#include <ros/ros.h>
#include "dyros_bolt_controller/mujoco_interface.h"
#include "dyros_bolt_controller/real_robot_interface.h"
using namespace dyros_bolt_controller;

#include <math.h>
// volatile bool *prog_shutdown;
// void SIGINT_handler(int sig)
// {//     cout << " CNTRL : shutdown Signal" << endl;
//     *prog_shutdown = true;// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyros_bolt_controller");//prepares the ROS environment for communication
    //ROS node name: dyros_bolt_controller

    ros::NodeHandle nh("~");
// ros::NodeHandle is used to communicate with thr ROS master, manage topics, services, parameters, etc.
//nh("~"): This creates a private node handle for the node. The tilde ~ signifies that the node handle will access the node's private namespace. This allows for namespacing of parameters and topics to avoid conflicts with other nodes or parts of the system.

    std::string mode; // To store the current run mode of the controller
    nh.param<std::string>("run_mode", mode, "simulation"); // if can't find run_mode --> the default is "simulation"
    ControlBase *ctr_obj; //ctr_obj라는 객체 생성- Type: ControlBase* 
    
    // int shm_id_;
    // init_shm(shm_msg_key, shm_id_, &ctr_obj->tc_shm_);
    // prog_shutdown = &ctr_obj->tc_shm_->shutdown;


    double Hz;
    nh.param<double>("control_frequency", Hz, 150.0); // if can't find control_frequency, Hz is set by the default value=150

    if(mode == "simulation")
    {
        //ROS_INFO allows to print a message to the console
        ROS_INFO("DYROS BOLT MAIN CONTROLLER - !!! MUJOCO SIMULATION MODE !!!");// It's running in simulation mode

        ctr_obj = new mujoco_interface(nh, Hz); 
        //mujoco_interface inherits ControlBase
        //Base Class: ControlBase, Derived Class: mujoco_interface
    }
    else if(mode == "real_robot")
    {
        ROS_INFO("DYROS BOLT MAIN CONTROLLER - !!! REAL ROBOT MODE !!!");
        ctr_obj = new RealRobotInterface(nh, Hz); 
        // RealRobotInterface inherits ControlBase 
        // Base Class: ControlBase, Derived Class: RealRobotInterface
    }
    else
    {
        ROS_FATAL("Please choose simulation or real_robot");
    }

    while(ros::ok())// checks whether the node has been shut down via an external command, such as a shutdown signal (Ctrl+C) or a failure in node initialization.
    {
        //Yobel_custome to find control_time_ value.
        //std::cout<<"Yobel custom: what's inside control_time -->"<<ctr_obj->control_time_<<std::endl;

        ctr_obj->readDevice(); 
        //ControlBase의 readDevice함수 호출// action server와 관련 // 새로운 goal 받고 갱신하고 등등// 

        ctr_obj->update();//mujoco_interface의 update함수 호출 // Updating kinematic and dynamic models of the robot. // Applying any necessary state transformations. // Filtering sensor data.
                
        ctr_obj->compute();//mujoco_interface의 compute함수 호출 // Calculating desired joint positions, velocities, and torques. // Executing control algorithms (e.g., PID control, RL control, trajectory generation).

        ctr_obj->reflect();//updating internal state representation, and publishing the current state to ROS topics.     
        
        ctr_obj->writeDevice();// Sends the computed control commands to the robot's actuators // Sending desired joint positions, velocities, and torques to the motors. // Ensuring the commands are executed on the robot.

        ctr_obj->wait();// to ensure the control loop runs at a consistent frequency // synchronize the "control loop" with the "simulation time in MUJOCO simulator"

        if(ctr_obj->isShuttingDown())//Check if the 'ctr_obj' has received a shutdown signal
        {
          break;
        }
    }
    delete ctr_obj;

    return 0;
}