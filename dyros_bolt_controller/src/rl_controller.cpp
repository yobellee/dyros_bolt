#include "dyros_bolt_controller/rl_controller.h"

namespace dyros_bolt_controller
{

void RLController::setEnable(bool enable)
{
    rl_enable_ = enable;
}

void RLController::compute()
{
    if(rl_enable_)
    {
        observationAllocation(current_q_, current_q_dot_, observation);
        action = module.forward({observation}).toTensor();
        action = action.to(torch::kDouble);
        Eigen::Map<Eigen::VectorXd> desired_torque_(action.data<double>(), action.numel());
    }
}

void RLController::observationAllocation(VectorQd current_q, VectorQd current_q_dot, torch::Tensor& observation)
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
    observation = torch::zeros({1, observation_size});
}

void RLController::updateControlMask(unsigned int *mask)
{
    if(rl_enable_)
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


}