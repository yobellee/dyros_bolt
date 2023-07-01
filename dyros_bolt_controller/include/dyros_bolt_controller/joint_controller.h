#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include "dyros_bolt_controller/dyros_bolt_model.h"
#include "dyros_bolt_controller/controller.h"
#include "math_type_define.h"

namespace dyros_bolt_controller
{


class JointController : public Controller
{
public:
  static constexpr unsigned int PRIORITY = 64;  ///< Joint priority

  JointController(const VectorQd& current_q, const VectorQd& current_q_dot, const double& control_time);
  virtual void compute();

  void setTarget(unsigned int joint_number, double target, double start_time, double end_time);
  void setTarget(unsigned int joint_number, double target, double duration);
  void setEnable(unsigned int joint_number, bool enable);
  virtual void updateControlMask(unsigned int *mask);
  virtual void writeDesired(const unsigned int *mask, VectorQd& desired_q);
  bool isEnabled(int index) { return joint_enable_[index]; }
private:
  bool joint_enable_[DyrosBoltModel::HW_TOTAL_DOF];

  const unsigned int total_dof_;
  VectorQd start_q_;
  VectorQd start_q_dot_;
  VectorQd desired_q_;
  VectorQd target_q_;
  const VectorQd& current_q_;
  const VectorQd& current_q_dot_;
  const double &current_time_;
  double start_time_[DyrosBoltModel::HW_TOTAL_DOF];
  double end_time_[DyrosBoltModel::HW_TOTAL_DOF];
};

} // namespace dyros_bolt_controller

#endif // JOINT_CONTROLLER_H
