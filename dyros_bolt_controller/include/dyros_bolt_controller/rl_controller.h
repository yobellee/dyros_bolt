#ifndef RL_CONTROLLER_H
#define RL_CONTROLLER_H

#include "dyros_bolt_controller/dyros_bolt_model.h"
#include <torch/script.h> // Include the PyTorch C++ API


namespace dyros_bolt_controller
{
class RLController
{
public:
    static constexpr unsigned int PRIORITY = 8;

    RLController(const VectorQd& current_q, const VectorQd &current_q_dot, const double hz, const double& control_time) :
        current_q_(current_q), current_q_dot_(current_q_dot), hz_(hz), current_time_(control_time)
    {
        // std::string desc_package_path = ros::package::getPath("dyros_bolt_controller");
        std::string jitPtFilePath = "/policy/policy_1.pt";
        try {
            module = torch::jit::load(jitPtFilePath);
        }
        catch (const c10::Error& e) {
            std::cerr << "error loading the model\n";
        }
    }

    void setEnable(bool enable);

    void compute();
    void observationAllocation(VectorQd current_q, VectorQd current_q_dot, torch::Tensor& observation);
    void updateControlMask(unsigned int *mask);
    void writeDesired(const unsigned int *mask, VectorQd& desired_torque);


private: 
    const double hz_;
    const double &current_time_; // updated by control_base
    const unsigned int total_dof_ = DyrosBoltModel::HW_TOTAL_DOF;

    const VectorQd& current_q_;
    const VectorQd& current_q_dot_;

    torch::jit::script::Module module;

    VectorQd desired_torque_;

    bool rl_enable_ = false;

    torch::Tensor observation;
    torch::Tensor action;

    int observation_size;
    int action_size;
};

}
#endif // RL_CONTROLLER_H
