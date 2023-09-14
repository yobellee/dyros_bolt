#include "dyros_bolt_controller/real_robot_interface.h"

namespace dyros_bolt_controller
{

RealRobotInterface::RealRobotInterface(ros::NodeHandle &nh, double Hz):
  ControlBase(nh, Hz), rate_(Hz)
{

    ROS_INFO("ODrive starting up");
    nh.param<bool>("engage_on_startup", engage_on_startup, DEFAULT_ENGAGE_ON_STARTUP);
    nh.param<bool>("disengage_on_shutdown", disengage_on_shutdown, DEFAULT_DISENGAGE_ON_SHUTDOWN);
    if (engage_on_startup) {
        ROS_INFO("Will engage axises on startup");
    } else {
        ROS_INFO("Will not engage axises on startup");
    }
    if (!nh.hasParam("axis_names")) {
        ROS_ERROR("Can't run without axis_names parameter");
    }
    if (!nh.hasParam("axis_can_ids")) {
        ROS_ERROR("Can't run without axis_can_ids parameter");
    }
    if (!nh.hasParam("axis_directions")) {
        ROS_ERROR("Can't run without axis_directions parameter");
    }
    nh.getParam("axis_names", axis_names_list);
    nh.getParam("axis_can_ids", axis_can_ids_list);
    nh.getParam("axis_directions", axis_directions_list);

    cl_ctrl_sub = nh.subscribe<std_msgs::Bool>("/cl_start", 1, &RealRobotInterface::motorEngageCallback, this);
    calibration_sub = nh.subscribe<std_msgs::Bool>("/calibration_start", 1, &RealRobotInterface::calibrationCallback, this);

    if (!(axis_names_list.size() == axis_can_ids_list.size() && axis_can_ids_list.size() ==
        axis_directions_list.size())) {
        ROS_ERROR("axis_names, axis_can_ids and axis_can_directions must be of an equal size");
    }
    if (!RealRobotInterface::stringsAreDistinct(axis_names_list)) {
        ROS_ERROR("axis names must be distinct");
    }
    if (!RealRobotInterface::intsAreDistinct(axis_can_ids_list)) {
        ROS_ERROR("axis CAN ids must be distinct");
    }
    for (int i = 0; i < (int)axis_names_list.size(); i++) {
        ROS_INFO("Adding axis %s with CAN id %d and direction %s", axis_names_list[i].c_str(), 
            axis_can_ids_list[i], axis_directions_list[i].c_str());
        if (axis_names_list[i].length() == 0) {
            ROS_ERROR("axis name can't be empty");
        }
        if (axis_can_ids_list[i] < 0) {
            ROS_ERROR("axis CAN id must be >0");
        }
        odrive_axises.push_back(new odrive::ODriveAxis(&nh, axis_names_list[i], axis_can_ids_list[i], 
            axis_directions_list[i]));
    }

}

bool RealRobotInterface::intsAreDistinct(std::vector<int> arr) {
    std::unordered_set<int> s;
    for (int i = 0; i < (int)arr.size(); i++) {
        s.insert(arr[i]);
    }
    return (s.size() == arr.size());
}

bool RealRobotInterface::stringsAreDistinct(std::vector<std::string> arr) {
    int n = arr.size();
    std::unordered_set<std::string> s;
    for (int i = 0; i < n; i++) {
        s.insert(arr[i]);
    }
    return (s.size() == arr.size());
}

void RealRobotInterface::motorEngageCallback(const std_msgs::Bool::ConstPtr& msg) {
    if ( msg->data)
    {
        for (int i = 0; i < (int)odrive_axises.size(); i++) {
            odrive_axises[i]->engage();
        }
    }
}

void RealRobotInterface::calibrationCallback(const std_msgs::Bool::ConstPtr& msg) {
    if ( msg->data)
    {
        for (int i = 0; i < (int)odrive_axises.size(); i++) {
            odrive_axises[i]->setAxisRequestedState(odrive::ODriveAxisState::ENCODER_OFFSET_CALIBRATION);
            // std::cout <
        }
    }
}

void RealRobotInterface::readDevice()
{
    ControlBase::readDevice();
    for (int i = 0; i < 3; i++)
    {
        q_(i) = odrive_axises[i]->axis_angle_;
        // std::cout << "encoder?" << q_(i) << std::endl;

        q_(i) = odrive_axises[i]->axis_velocity_;
        q_(i+4) = odrive_axises[i+3]->axis_angle_;
        q_(i+4) = odrive_axises[i+3]->axis_velocity_;
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
            std::cout << "ready?" << desired_torque_(0) << std::endl;
            odrive_axises[i]->setInputTorque(double(desired_torque_(i)));
        }
    }
}

void RealRobotInterface::wait()
{
    rate_.sleep();
}

bool RealRobotInterface::areMotorsReady()
{
    for(int i = 0; i < odrive_axises.size()-3; i++) {
        std::cout << "axis" << i<< ":" <<odrive_axises[i]->axis_current_state_ << std::endl;

        if(odrive_axises[i]->axis_current_state_ != 8) {
            return false;
        }
    }
    return true;
}

}