#include "dyros_bolt_controller/joint_controller.h"

namespace dyros_bolt_controller
{

JointController::JointController(const VectorQd& current_q, const VectorQd &current_q_dot, const double& control_time) :
  Controller(PRIORITY),
  current_q_(current_q), current_q_dot_(current_q_dot),
  current_time_(control_time), total_dof_(DyrosBoltModel::HW_TOTAL_DOF),
  start_time_{}, end_time_{}
{

}

void JointController::compute()
{//total_dof=8
  for(unsigned int i=0; i<total_dof_; i++)
  {
    if(joint_enable_[i])
    {
      if (current_time_ >= end_time_[i])
      {
        desired_q_(i) = target_q_(i);
        joint_enable_[i] = false;
      }
      else
      {
        desired_q_(i) = DyrosMath::cubic(current_time_, start_time_[i], end_time_[i], start_q_(i), target_q_(i), start_q_dot_(i), 0);
      }
    }
  }
}

void JointController::setTarget(unsigned int joint_number, double target, double start_time, double end_time)
{
  if(joint_number >= total_dof_)
  {
    ROS_ERROR("JointController::setTarget - Out of range. Input = %u", joint_number);
    return ;
  }
  start_time_[joint_number] = start_time; // in outr bolt, joint variable은 8개-> 6자유도 + pinjoint2개; start_time_[8]
  end_time_[joint_number] = end_time;

  //start_q_(join_number): start_q_ 벡터의 joint_number번째 요소를 가리킴.*note: start_q_ is a vector which has 8 elements
  start_q_(joint_number) = current_q_(joint_number);//motion planning starts from the joint's current position
  start_q_dot_(joint_number) = current_q_dot_(joint_number);
  target_q_(joint_number) = target;
}

void JointController::setTarget(unsigned int joint_number, double target, double duration)
{
  setTarget(joint_number, target, current_time_, current_time_ + duration);
}

void JointController::setEnable(unsigned int joint_number, bool enable)
{
  if (joint_number < total_dof_)
  {
    joint_enable_[joint_number] = enable;
  }
  else
  {
    ROS_ERROR("JointController::setEnable - Out of range. Input = %u", joint_number);
  }
}

void JointController::updateControlMask(unsigned int *mask)
{
  for(unsigned int i=0; i<total_dof_; i++)
  {
    if(joint_enable_[i])
    {
      setMask(&mask[i]);// Sets the mask for the i-th joint, indicating that this joint is controlled by the JointController.
    }
    else
    {
      resetMask(&mask[i]);//Resets the mask for the i-th joint, indicating that this joint is not controlled by the JointController.
    }
  }
}

void JointController::writeDesired(const unsigned int *mask, VectorQd& desired_q)
{
  for(unsigned int i=0; i<total_dof_; i++)
  {
    if(isAvailable(&mask[i]))// Check whether 'priority_' bit is set in the 'mask[i]'
    {
      desired_q(i) = desired_q_(i);
      std::cout<<"joint("<<i<<"): "<<desired_q(i)<<std::endl;
    }
  }
}

}
