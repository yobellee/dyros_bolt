// // #include <ros/ros.h>
// // #include "dyros_bolt_controller/mujoco_interface.h"
// // #include "dyros_bolt_controller/real_robot_interface.h"
// // using namespace dyros_bolt_controller;

// // #include <math.h>



// // int main(int argc, char **argv)
// // {
// //     ros::init(argc, argv, "dyros_bolt_controller");
// //     ros::NodeHandle nh("~");

// //     std::string mode;
// //     nh.param<std::string>("run_mode", mode, "simulation");
// //     ControlBase *ctr_obj;
// //     ROS_INFO("!!!!!!!");
    

// //     double Hz;
// //     nh.param<double>("control_frequency", Hz, 200.0);

// //     if(mode == "simulation")
// //     {
// //         ROS_INFO("DYROS BOLT MAIN CONTROLLER - !!! MUJOCO SIMULATION MODE !!!");
// //         ctr_obj = new mujoco_interface(nh, Hz);
// //     }
// //     else if(mode == "real_robot")
// //     {
// //         ROS_INFO("DYROS BOLT MAIN CONTROLLER - !!! REAL ROBOT MODE !!!");
// //         ctr_obj = new RealRobotInterface(nh, Hz);
// //     }
// //     else
// //     {
// //         ROS_FATAL("Please choose simulation or real_robot");
// //     }

// //     while(ros::ok())
// //     {
// //         ctr_obj->readDevice();
// //         ctr_obj->update();
// //         ctr_obj->compute();
// //         ctr_obj->reflect();
// //         ctr_obj->writeDevice();
// //         ctr_obj->wait();

// //         if(ctr_obj->isShuttingDown())
// //         {
// //           break;
// //         }
// //     }

// //     delete ctr_obj;

// //     return 0;
// // }


// #include "dyros_bolt_controller/dyros_bolt_controller.h"

// using namespace std;

// DyrosBoltController::DyrosBoltController(StateManager %std_global) :  dc_(stm_global.dc_), stm_(stm_global), rd_(stm_global.dc_.rd_)
// #ifdef COMPILE_DYROS_BOLT_CC
//                                                                ,
//                                                                my_cc(*(new CustomController(rd_)))
// #endif
// {
//     // nh_controller_.setCallbackQueue(&queue_controller_);
//     // sub_1 = nh_controller_.subscribe("/tocabi/avatar_test", 1, &AvatarController::avatar_callback, this);

//     // task_command_sub_ = nh_controller_.subscribe("/dyros_bolt/taskcommand", 100, &TocabiController::TaskCommandCallback, this);
//     // task_command_que_sub_ = nh_controller_.subscribe("/dyros_bolt/taskquecommand", 100, &TocabiController::TaskQueCommandCallback, this);
//     // position_command_sub_ = nh_controller_.subscribe("/dyros_bolt/positioncommand", 100, &TocabiController::PositionCommandCallback, this);
//     // task_gain_sub_ = nh_controller_.subscribe("/dyros_bolt/taskgaincommand", 100, &TocabiController::TaskGainCommandCallback, this);

//     // ros::param::get("/dyros_bolt_controller/Kp", rd_.pos_kp_v);
//     // ros::param::get("/dyros_bolt_controller/Kv", rd_.pos_kv_v);
// }

// DyrosBoltController::~DyrosBoltController()
// {
//     cout << "TocabiController Terminated" << endl;
// }

