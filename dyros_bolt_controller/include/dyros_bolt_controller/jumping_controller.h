#ifndef JUMPING_CONTROLLER_H
#define JUMPING_CONTROLLER_H
#include "dyros_bolt_controller/dyros_bolt_model.h"


namespace dyros_bolt_controller
{
class JumpingController
{
    public:
        JumpingController(DyrosBoltModel& model, const VectorQd& current_q, const VectorQd& current_qdot, const double hz, const double& control_time) :
            total_dof_(DyrosBoltModel::HW_TOTAL_DOF), model_(model), current_q_(current_q), current_qdot_(current_qdot), hz_(hz), current_time_(control_time)
        {

        }

        void setEnable(bool enable);
        void setTarget();
        void compute();
        void updateControlMask(unsigned int *mask);
        void writeDesired(const unsigned int *mask, VectorQd& desired_q);
        void computeIK(Eigen::Isometry3d float_pelv_transform,
                       Eigen::Isometry3d float_lleg_transform,
                       Eigen::Isometry3d float_rleg_transform,
                       Eigen::VectorXd& q_desired);

    private:

        const double hz_;
        const double &current_time_; // updated by control_base

        const VectorQd& current_q_;
        const VectorQd& current_qdot_;
        
        DyrosBoltModel &model_;
        const unsigned int total_dof_;


};
}

#endif // JUMPING_CONTROLLER_H
