#include "dyros_bolt_controller/rl_controller.h"

namespace dyros_bolt_controller
{

void RLController::setEnable(bool enable)
{
    this->rl_enable_ = enable;
}

void RLController::compute()
{
    // std::cout << "mujoco_virtual_[]: " << virtual_q_dot_.transpose() << std::endl;
    // std::cout << "q_[]: " << current_q_.transpose() << std::endl;
    if(this->rl_enable_)
    {
        observationAllocation(current_q_, current_q_dot_, virtual_q_dot_, base_quat_);
        this->action = module.forward({this->observation}).toTensor();
        this->action = this->action.to(torch::kDouble);
        Eigen::Map<Eigen::VectorXd> desired_torque_(this->action.data<double>(), this->action.numel());
        for(int i=0; i<3; i++)
        {
            this->desired_torque_[i] = desired_torque_[i];
            this->desired_torque_[i+4] = desired_torque_[i+3];
        }
        // this->desired_torque_ = desired_torque_;
    }
}

void RLController::observationAllocation(VectorQd current_q, VectorQd current_q_dot, Vector6d virtual_q_dot, Eigen::Quaterniond base_quat)
{
    /*
        self.base_lin_vel:  torch.Size([4096, 3])
        self.base_ang_vel:  torch.Size([4096, 3])
        self.projected_gravity:  torch.Size([4096, 3])
        self.commands[:, :2]:  torch.Size([4096, 2])
        (self.dof_pos - self.default_dof_pos):  torch.Size([4096, 6])
        self.dof_vel:  torch.Size([4096, 6])
        self.actions:  torch.Size([4096, 6])

        3 + 3 + 3 + 2 + 6 + 6 + 6 = 29(num_observation)
    */

    Vector3d base_lin_vel_ = virtual_q_dot.head(3)*2.0;
    // std::cout << "base_lin_vel_: " << virtual_q_dot_.transpose() << std::endl;
    Vector3d base_ang_vel_ = virtual_q_dot.tail(3)*0.25;
    Vector3d projected_gravity_ = quat_rotate_inverse(base_quat, this->gravity);
    Vector2d commands_ = Vector2d(4, 0)*2.0;

    // Vector6d dof_pos_ = current_q;
    // Vector6d dof_vel_ = current_q_dot*0.05;
    Vector6d dof_pos_;
    Vector6d dof_vel_;
    Vector6d action_;

    for(int i=0; i<3; i++)
    {
        dof_pos_[i] = current_q[i]-init_q_[i];
        dof_pos_[i+3] = current_q[i+4]-init_q_[i+3];
        dof_vel_[i] = current_q_dot[i]*0.05;
        dof_vel_[i+3] = current_q_dot[i+4]*0.05;
        action_[i] = this->desired_torque_[i];
        action_[i+3] = this->desired_torque_[i+4];
    }

    torch::Tensor base_lin_vel = torch::from_blob(base_lin_vel_.data(), {1, 3});
    // std::cout << "base_lin_vel: " << base_lin_vel << std::endl;
    torch::Tensor base_ang_vel = torch::from_blob(base_ang_vel_.data(), {1, 3});
    torch::Tensor projected_gravity = torch::from_blob(projected_gravity_.data(), {1, 3});
    torch::Tensor commands = torch::from_blob(commands_.data(), {1, 2});
    std::cout << "command: " << commands_.data() << std::endl;
    std::cout << "command0: " << commands << std::endl;
    torch::Tensor dof_pos = torch::from_blob(dof_pos_.data(), {1, 6});
    torch::Tensor dof_vel = torch::from_blob(dof_vel_.data(), {1, 6});
    torch::Tensor action = torch::from_blob(action_.data(), {1, 6});

    std::vector<torch::Tensor> tensor_list = {base_lin_vel, base_ang_vel, projected_gravity, commands, dof_pos, dof_vel, action};
    
    // this->observation = torch::zeros({1, this->observation_size});
    this->observation = torch::cat(tensor_list, 1);
    // std::cout << "observation: " << observation << std::endl;
    // std::cout << "observation: " << observation.sizes() << std::endl;
}

void RLController::updateControlMask(unsigned int *mask)
{
    if(this->rl_enable_)
    {
        for (int i=0; i<total_dof_; i++)
        {
            mask[i] = (mask[i] | PRIORITY);
        }
    }
    else
    {
        for (int i=0; i<total_dof_; i++)
        {
            mask[i] = (mask[i] & ~PRIORITY);
        }
    }
}

void RLController::writeDesired(const unsigned int *mask, VectorQd& desired_torque)
{
    if(this->rl_enable_)
    {
        for (int i=0; i<8; i++)
        {
            if(mask[i] & PRIORITY)
            {
                desired_torque[i] = desired_torque_[i];
            }
        }
    }

}

Eigen::Vector3d RLController::quat_rotate_inverse(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) 
{
    Eigen::Vector3d q_vec = q.vec();
    double q_w = q.w();

    Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);
    Eigen::Vector3d b = q_vec.cross(v) * q_w * 2.0;
    Eigen::Vector3d c = q_vec * (q_vec.dot(v) * 2.0);

    return a - b + c;
}
}