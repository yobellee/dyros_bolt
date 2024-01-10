#include "dyros_bolt_controller/real_robot_interface.h"

namespace dyros_bolt_controller
{
std::ofstream outFile("/home/yong/data.txt");

RealRobotInterface::RealRobotInterface(ros::NodeHandle &nh, double Hz):
  ControlBase(nh, Hz), rate_(Hz), odrv(nh)
{
    ROS_INFO("ODrive starting up");
    axis_request_state_sub = nh.subscribe<std_msgs::Int16>("/odrv_axis_request_states", 1, &RealRobotInterface::axisRequestStateCallback, this);
    axis_current_state_pub = nh.advertise<std_msgs::Int16MultiArray>("/odrv_axis_current_states", 1);

}

void RealRobotInterface::axisRequestStateCallback(const std_msgs::Int16::ConstPtr& msg) {
    int16_t requestState = msg->data;
    
    switch (requestState) {
        case 1:
            odrv.disengage();
            break;
        case 2:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.requestODriveCmd(i, odrive::ODriveCommandId::ESTOP_MESSAGE);
            }
            break;    
        case 4:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.setAxisRequestedState(odrv.axis_can_ids_list[i], odrive::ODriveAxisState::MOTOR_CALIBRATION);
            }
            break;
        case 7:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.setAxisRequestedState(odrv.axis_can_ids_list[i], odrive::ODriveAxisState::ENCODER_OFFSET_CALIBRATION);
            }
            break;
        case 8:
            odrv.engage();
            for(int i=0; i< DyrosBoltModel::HW_TOTAL_DOF / 2 - 1; i++)
            {
                odrv.setInputTorque(i, 0);
                odrv.setInputTorque(i+3,0);
            }
            break;
        case 16:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.requestODriveCmd(i, odrive::ODriveCommandId::REBOOT_ODRIVE);
            }
            break;
        case 19:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.resetEncoder(i, odrive::ODriveCommandId::SET_ABSOLUTE_POSITION);
            }
            break;    
    }
}

void RealRobotInterface::readDevice()
{
    ControlBase::readDevice();
    axisCurrentPublish();
    
    for (int i = 0; i < 3; i++)
    {
        q_(i) = odrv.axis_angle[i];
        q_dot_(i) = odrv.axis_velocity[i];

        q_(i+4) = odrv.axis_angle[i+3];
        q_dot_(i+4) = odrv.axis_velocity[i+3];
    }
    q_[3] = 0;
    q_[7] = 0;
    q_dot_[3] =0;
    q_dot_[7] =0;
}

void RealRobotInterface::update()
{
    ControlBase::update();
}

void RealRobotInterface::writeDevice()
{
    if(areMotorsReady()){
        for(int i=0; i< DyrosBoltModel::HW_TOTAL_DOF / 2 - 1; i++)
        {
            // if(i = 3)
            // {
            //     // odrv.setInputTorque(i, 0.0);
            //     // odrv.setInputTorque(i+4, 0.0);
            // }
            odrv.setInputTorque(i, double(k_tau[i] * desired_torque_(i)));
            odrv.setInputTorque(i+3, double(k_tau[i+3] * desired_torque_(i+4)));
        }
    }
}

void RealRobotInterface::wait()
{
    rate_.sleep();
}

bool RealRobotInterface::areMotorsReady()
{
    for(int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
    // for(int i = 0; i < 1; i++) {
        if(odrv.axis_current_state[i] != 8) {
            return false;
        }
    }
    return true;
}

void RealRobotInterface::axisCurrentPublish()
{
    std_msgs::Int16MultiArray state_msgs;
    for (int i = 0; i < 6; i++)
    {
        state_msgs.data.push_back(odrv.axis_current_state[i]);
    }
    axis_current_state_pub.publish(state_msgs);
}

}