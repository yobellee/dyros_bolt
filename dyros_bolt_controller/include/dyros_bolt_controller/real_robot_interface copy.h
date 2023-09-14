#ifndef REAL_ROBOT_INTERFACE_H
#define REAL_ROBOT_INTERFACE_H

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <unordered_set>
#include <can_msgs/Frame.h>
#include "dyros_bolt_controller/odrive_axis.hpp"

#define DEFAULT_CAN_INTERFACE       "can0"
#define DEFAULT_CAN_BITRATE         500000
#define DEFAULT_ENGAGE_ON_STARTUP   false
#define DEFAULT_DISENGAGE_ON_SHUTDOWN   true

#include "control_base.h"

namespace dyros_bolt_controller
{
class RealRobotInterface : public ControlBase
{
public:
    RealRobotInterface(ros::NodeHandle &nh, double Hz);
    virtual ~RealRobotInterface() { odrive_axises.clear();}

    virtual void readDevice() override;
    virtual void update() override; // update controller based on readdevice
    virtual void writeDevice() override; // publish to actuate devices
    virtual void wait() override;

    void motorEngageCallback(const std_msgs::Bool::ConstPtr& msg);
    void calibrationCallback(const std_msgs::Bool::ConstPtr& msg);
    bool intsAreDistinct(std::vector<int> arr);
    bool stringsAreDistinct(std::vector<std::string> arr);
    
    std::vector<odrive::ODriveAxis *> odrive_axises;
    
    bool engage_on_startup;
    bool disengage_on_shutdown;
    
    std::vector<std::string> axis_names_list;
    std::vector<int> axis_can_ids_list;
    std::vector<std::string> axis_directions_list;
private:
    ros::Rate rate_;

    ros::Subscriber cl_ctrl_sub;
    ros::Subscriber calibration_sub;

    bool areMotorsReady();
    

};

}
#endif // REAL_ROBOT_INTERFACE_H










