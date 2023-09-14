#include "dyros_bolt_controller/real_robot_interface.h"

namespace dyros_bolt_controller
{

RealRobotInterface::RealRobotInterface(ros::NodeHandle &nh, double Hz):
  ControlBase(nh, Hz), rate_(Hz), odrv(nh)
{
    ROS_INFO("ODrive starting up");
    cl_ctrl_sub = nh.subscribe<std_msgs::Bool>("/cl_start", 1, &RealRobotInterface::motorEngageCallback, this);
    calibration_sub = nh.subscribe<std_msgs::Bool>("/calibration_start", 1, &RealRobotInterface::calibrationCallback, this);
}

void RealRobotInterface::motorEngageCallback(const std_msgs::Bool::ConstPtr& msg) {
    if ( msg->data)
    {
        odrv.engage();
    }
}

void RealRobotInterface::calibrationCallback(const std_msgs::Bool::ConstPtr& msg) {
    if ( msg->data )
    {
        for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
            odrv.setAxisRequestedState(odrv.axis_can_ids_list[i], odrive::ODriveAxisState::ENCODER_OFFSET_CALIBRATION);
        }
    }
}

void RealRobotInterface::readDevice()
{
    ControlBase::readDevice();
    
    for (int i = 0; i < 3; i++)
    {
        q_(i) = odrv.axis_angle[i];
        q_(i) = odrv.axis_velocity[i];

        q_(i+4) = odrv.axis_angle[i+3];
        q_(i+4) = odrv.axis_velocity[i+3];
    }
    q_[3] = M_PI/2;
    q_[7] = M_PI/2;
}

void RealRobotInterface::update()
{
    ControlBase::update();
}

void RealRobotInterface::writeDevice()
{
    if(areMotorsReady()){
        for(int i=0; i< DyrosBoltModel::HW_TOTAL_DOF; i++)
        {
            if(i == 3 || i == 7)
                odrv.setInputTorque(i, 0.0);

            std::cout << "ready?" << desired_torque_(0) << std::endl;
            odrv.setInputTorque(i, double(desired_torque_(i)));
        }
    }
}

void RealRobotInterface::wait()
{
    rate_.sleep();
}

bool RealRobotInterface::areMotorsReady()
{
    for(int i = 0; i < odrv.axis_can_ids_list.size()-4; i++) {
        if(odrv.axis_current_state[i] != 8) {
            return false;
        }
    }
    return true;
}

}