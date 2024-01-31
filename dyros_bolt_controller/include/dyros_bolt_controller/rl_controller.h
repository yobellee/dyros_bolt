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

    RLController(const VectorQd& current_q, const VectorQd& current_q_dot, const Vector6d& virtual_q_dot, const Eigen::Quaterniond& base_quat, const double hz, const double& control_time) :
        current_q_(current_q), current_q_dot_(current_q_dot), virtual_q_dot_(virtual_q_dot),base_quat_(base_quat), hz_(hz), current_time_(control_time)
    {
        init_q_ << 0, 0.1, -0.25, 0, 0.1, -0.25;

        std::string desc_package_path = ros::package::getPath("dyros_bolt_controller");
        std::string jitPtFilePath = desc_package_path + "/policy/policy_1.pt";
        try {
            module = torch::jit::load(jitPtFilePath);
        }
        catch (const c10::Error& e) {
            std::cerr << "error loading the model\n";
        }
        observation = torch::zeros({1, observation_size});
        action = module.forward({observation}).toTensor();
        action = action.to(torch::kDouble);
        Eigen::Map<Eigen::VectorXd> desired_torque__(action.data<double>(), action.numel());
        for(int i=0; i<3; i++)
        {
            this->desired_torque_[i] = desired_torque__[i];
            this->desired_torque_[i+4] = desired_torque__[i+3];
        }
    }

    void setEnable(bool enable);

    void compute();
    void observationAllocation(VectorQd current_q, VectorQd current_q_dot, Vector6d virtual_q_dot, Eigen::Quaterniond base_quat);
    void updateControlMask(unsigned int *mask);
    void writeDesired(const unsigned int *mask, VectorQd& desired_torque);

    Eigen::Vector3d quat_rotate_inverse(const Eigen::Quaterniond& q, const Eigen::Vector3d& v);
private: 
    const double hz_;
    const double &current_time_; // updated by control_base
    const unsigned int total_dof_ = DyrosBoltModel::HW_TOTAL_DOF;

    const VectorQd& current_q_;
    Vector6d init_q_;
    const VectorQd& current_q_dot_;
    const Vector6d& virtual_q_dot_;
    const Eigen::Quaterniond& base_quat_;
    Vector3d gravity = Vector3d(0, 0, -9.81);

    torch::jit::script::Module module;

    VectorQd desired_torque_;

    bool rl_enable_ = false;

    torch::Tensor observation;
    torch::Tensor action;

    int observation_size = 29;
    int action_size;
};

}
#endif // RL_CONTROLLER_H
