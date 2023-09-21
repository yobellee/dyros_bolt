#include "dyros_bolt_controller/real_robot_interface.h"

namespace dyros_bolt_controller
{

RealRobotInterface::RealRobotInterface(ros::NodeHandle &nh, double Hz):
  ControlBase(nh, Hz), rate_(Hz), odrv(nh)
{
    ROS_INFO("ODrive starting up");
    axis_request_state_sub = nh.subscribe<std_msgs::Int16>("/odrv_axis_request_states", 1, &RealRobotInterface::axisRequestStateCallback, this);
}

void RealRobotInterface::axisRequestStateCallback(const std_msgs::Int16::ConstPtr& msg) {
    int16_t requestState = msg->data;
    
    swtich (requestState) {
        case 1:
            odrv.disengage();
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
            break;    
    }
}

void RealRobotInterface::readDevice()
{
    ControlBase::readDevice();
    
    for (int i = 0; i < 3; i++)
    {
        q_(i) = odrv.axis_angle[i];
        q_dot_(i) = odrv.axis_velocity[i];

        q_(i+4) = odrv.axis_angle[i+3];
        q_dot_(i+4) = odrv.axis_velocity[i+3];
    }
    q_[3] = 0;
    q_[3] = 0;
    // q_[3] = M_PI/2;
    // q_[7] = M_PI/2;
}

void RealRobotInterface::update()
{
    ControlBase::update();
}

void RealRobotInterface::writeDevice()
{
    if(areMotorsReady()){
        std::cout << "motor" << std::endl;
        for(int i=0; i< DyrosBoltModel::HW_TOTAL_DOF / 2; i++)
        {
            if(i == 3)
            {
                odrv.setInputTorque(i, 0.0);
                odrv.setInputTorque(i+4, 0.0);
            }
            odrv.setInputTorque(i, double(desired_torque_(i)));
            odrv.setInputTorque(i+4, double(desired_torque_(i+4)));
        }
    }
}

void RealRobotInterface::wait()
{
    rate_.sleep();
}

bool RealRobotInterface::areMotorsReady()
{
    // for(int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
    for(int i = 3; i < 4; i++) {
        if(odrv.axis_current_state[i] != 8) {
            return false;
        }
    }
    return true;
}

}