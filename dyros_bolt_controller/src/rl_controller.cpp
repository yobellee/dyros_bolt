#include "dyros_bolt_controller/rl_controller.h"

namespace dyros_bolt_controller
{

RLController::RLController(const VectorQd& current_q, const VectorQd& current_q_dot, const Vector6d& virtual_q_dot, const Eigen::Quaterniond& base_quat, const double hz, const double& control_time) :
    current_q_(current_q), current_q_dot_(current_q_dot), virtual_q_dot_(virtual_q_dot), base_quat_(base_quat), hz_(hz), current_time_(control_time), total_dof_(DyrosBoltModel::HW_TOTAL_DOF)
{
    desired_torque_.setZero();

     /* if (is_write_file_)//이 if 문 어차피 안 써
    {
        if (is_on_robot_)
        {
            writeFile.open("/home/dyros/catkin_ws/src/tocabi_cc/result/data.csv", std::ofstream::out | std::ofstream::app);
        }
        else
        {
            writeFile.open("/home/dyros2/tocabi_ws/src/tocabi_cc/result/data.csv", std::ofstream::out | std::ofstream::app);
        }
        writeFile << std::fixed << std::setprecision(8);
    }*/

    initVariable();
    loadNetwork();

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RLController::joyCallback, this);
}


void RLController::setEnable(bool enable)
{
    this->rl_enable_ = enable;
}

void RLController::initVariable()
{
    // Define the size of the layers based on your network architecture --> for more information watch the book "UnderstandingDeepLearning", which Ph.D. gave.
    policy_net_w0_.resize(num_hidden1, num_cur_state * num_state_hist); // 512 by 120 matrix
    policy_net_b0_.resize(num_hidden1, 1);
    policy_net_w1_.resize(num_hidden2, num_hidden1);
    policy_net_b1_.resize(num_hidden2, 1);
    policy_net_w2_.resize(num_hidden3, num_hidden2);
    policy_net_b2_.resize(num_hidden3, 1);
    action_net_w_.resize(num_action, num_hidden3);
    action_net_b_.resize(num_action, 1);

    hidden_layer1_.resize(num_hidden1, 1);
    hidden_layer2_.resize(num_hidden2, 1);
    hidden_layer3_.resize(num_hidden3, 1);
    rl_action_.resize(num_action, 1);

    state_.resize(num_cur_state, 1);//actually we don't use this
    state_cur_.resize(num_cur_state, 1);
    //state_mean_.resize(num_cur_state, 1);
    state_var_.resize(num_cur_state, 1);
    state_buffer_.resize(num_cur_state * num_state_hist, 1);//num_state_skip 안 두었음.--> 얼만큼의 state을 skip할지 몰라서 --> 박사님께 말하기

    //q_dot_lpf_.setZero();//actually we don't use this

    //Initializing q without consiering ankle!
    

    
    torque_bound_ << 333.0, 232.0, 263.0, 289.0, 222.0, 166.0;//bound 이렇게 줬는데 맞는지 모르겠음 ask 박사님!    
    //q_init_ << 0.0, 0.0, -0.24, 0.6, -0.36, 0.0;
}

void RLController::loadNetwork()
{
    // Load network weights from files
    std::string path = "/home/dyros2/bolt_ws/src/dyros_bolt/dyros_bolt_controller/";//weight folder is in this directory

    std::ifstream file[8];
    file[0].open(path + "weight/0_weight.txt", std::ios::in);
    file[1].open(path + "weight/0_bias.txt", std::ios::in);
    file[2].open(path + "weight/2_weight.txt", std::ios::in);
    file[3].open(path + "weight/2_bias.txt", std::ios::in);
    file[4].open(path + "weight/4_weight.txt", std::ios::in);
    file[5].open(path + "weight/4_bias.txt", std::ios::in);
    file[6].open(path + "weight/6_weight.txt", std::ios::in);
    file[7].open(path + "weight/6_bias.txt", std::ios::in);

    if (!file[0].is_open())
    {
        std::cerr << "Can not find the weight file" << std::endl;
        return;
    }

    double temp;
    int row = 0;
    int col = 0;

    while (!file[0].eof() && row != policy_net_w0_.rows())
    {
        file[0] >> temp;
        policy_net_w0_(row, col) = temp;
        col++;
        if (col == policy_net_w0_.cols())
        {
            col = 0;
            row++;
        }
    }
    row = 0;
    col = 0;
    while (!file[1].eof() && row != policy_net_b0_.rows())
    {
        file[1] >> temp;
        policy_net_b0_(row, col) = temp;
        col++;
        if (col == policy_net_b0_.cols())
        {
            col = 0;
            row++;
        }
    }
    row = 0;
    col = 0;
    while (!file[2].eof() && row != policy_net_w1_.rows())
    {
        file[2] >> temp;
        policy_net_w1_(row, col) = temp;
        col++;
        if (col == policy_net_w1_.cols())
        {
            col = 0;
            row++;
        }
    }
    row = 0;
    col = 0;
    while (!file[3].eof() && row != policy_net_b1_.rows())
    {
        file[3] >> temp;
        policy_net_b1_(row, col) = temp;
        col++;
        if (col == policy_net_b1_.cols())
        {
            col = 0;
            row++;
        }
    }
    row = 0;
    col = 0;
    while (!file[4].eof() && row != policy_net_w2_.rows())
    {
        file[4] >> temp;
        policy_net_w2_(row, col) = temp;
        col++;
        if (col == policy_net_w2_.cols())
        {
            col = 0;
            row++;
        }
    }
    row = 0;
    col = 0;
    while (!file[5].eof() && row != policy_net_b2_.rows())
    {
        file[5] >> temp;
        policy_net_b2_(row, col) = temp;
        col++;
        if (col == policy_net_b2_.cols())
        {
            col = 0;
            row++;
        }
    }
    row = 0;
    col = 0;
    while (!file[6].eof() && row != action_net_w_.rows())
    {
        file[6] >> temp;
        action_net_w_(row, col) = temp;
        col++;
        if (col == action_net_w_.cols())
        {
            col = 0;
            row++;
        }
    }
    row = 0;
    col = 0;
    while (!file[7].eof() && row != action_net_b_.rows())
    {
        file[7] >> temp;
        action_net_b_(row, col) = temp;
        col++;
        if (col == action_net_b_.cols())
        {
            col = 0;
            row++;
        }
    }
}

void RLController::processObservation()
{
    int data_idx = 0;

    // Projected gravity --> 박사님께 여쭤보기 챗 지피티에서 알려준 계산법이 두 개인데, 정반대의 결과임
    Eigen::Vector3d projected_gravity = quat_rotate_inverse(base_quat_, Eigen::Vector3d(0, 0, -9.81));
    state_cur_(data_idx++) = projected_gravity(0);//state_curr_(0)=projected_gravity(0);
    state_cur_(data_idx++) = projected_gravity(1);
    state_cur_(data_idx++) = projected_gravity(2);

///////여기서부터 봐!!!!!!![2024.08.02]///////////
    // Command (x, y, yaw)
    state_cur_(data_idx++) = target_vel_x_;
    state_cur_(data_idx++) = target_vel_y_;
    state_cur_(data_idx++) = target_yaw_; 

    // Joint positions
    for (int i = 0; i < 3; i++)
    {
        state_cur_(data_idx++) = current_q_(i);
    }
    for (int i = 4; i < num_action; i++)
    {
        state_cur_(data_idx++) = current_q_(i);
    }


    // Joint velocities
    for (int i = 0; i < 3; i++)
    {
        state_cur_(data_idx++) = current_q_dot_(i);
    }
    for (int i = 4; i < num_action; i++)
    {
        state_cur_(data_idx++) = current_q_dot_(i);
    }
    

    // Previous actions
    for (int i = 0; i < num_action; i++)
    {
        state_cur_(data_idx++) = rl_action_(i);
    }


    state_buffer_.block(0, 0, num_cur_state * (num_state_hist - 1), 1) = state_buffer_.block(num_cur_state, 0, num_cur_state * (num_state_hist - 1), 1);

    //맨 뒤에는 최신 state update
    //mean값이 들어있는 파일이 없어서 normalize하진 않음--> 박사님께 말하기
    state_buffer_.block(num_cur_state * (num_state_hist - 1), 0, num_cur_state, 1) = state_cur_;

    //state_buffer_ = (state_cur_ - state_mean_).array() / state_var_.cwiseSqrt().array();
}

void RLController::feedforwardPolicy()
{
    hidden_layer1_ = policy_net_w0_ * state_buffer_ + policy_net_b0_;
    hidden_layer1_ = hidden_layer1_.cwiseMax(0.0); // ReLU activation

    hidden_layer2_ = policy_net_w1_ * hidden_layer1_ + policy_net_b1_;
    hidden_layer2_ = hidden_layer2_.cwiseMax(0.0); // ReLU activation

    hidden_layer3_ = policy_net_w2_ * hidden_layer2_ + policy_net_b2_;
    hidden_layer3_ = hidden_layer3_.cwiseMax(0.0); // ReLU activation

    rl_action_ = action_net_w_ * hidden_layer3_ + action_net_b_;
}

void RLController::compute()
{
    if (rl_enable_)
    {
        double current_time = current_time_; // Assuming current_time_ is in seconds
        double elapsed_time = current_time - last_policy_update_time_;
        
        // hz_: Control Base의 constructor를 통해서, Bolt의 제어주기로 초기화되는 애임
        if (elapsed_time >= 1.0 / hz_)
        {
            processObservation();
            feedforwardPolicy();



            // Update the last policy update time
            last_policy_update_time_ = current_time;
        }
        for (int i = 0; i < num_action; i++)
        {
            desired_torque_(i) = DyrosMath::minmax_cut(rl_action_(i)*torque_bound_(i), -torque_bound_(i), torque_bound_(i));//torque 위 아래로 짜르는거 bound 줘서            
        }
    }
}

void RLController::updateControlMask(unsigned int *mask)
{
    if (rl_enable_)
    {
        for (int i = 0; i < total_dof_; i++)
        {
            mask[i] |= PRIORITY;
        }
    }
    else
    {
        for (int i = 0; i < total_dof_; i++)
        {
            mask[i] &= ~PRIORITY;
        }
    }
}

void RLController::writeDesired(const unsigned int *mask, VectorQd& desired_torque)
{
    if (rl_enable_)
    {
        for (int i = 0; i < total_dof_; i++)
        {
            if (mask[i] >= PRIORITY && mask[i] < PRIORITY * 2)
            {
                desired_torque[i] = desired_torque_(i);
            }
        }
    }
}

Eigen::VectorXd RLController::quat_rotate_inverse(const Eigen::Quaterniond& q, const Eigen::Vector3d& v)
{
    Eigen::Quaterniond q_inv = q.conjugate();
    Eigen::Vector3d rotated_v = q_inv * v;
    return rotated_v.cast<double>();
}

//박사님 저희는 joystick으로 명령 안 주지만 target_을 정해주고자 이렇게 했어요.
void RLController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    target_vel_x_ = DyrosMath::minmax_cut(0.5 * joy->axes[1], -0.2, 0.5);
    target_vel_y_ = DyrosMath::minmax_cut(0.5 * joy->axes[0], -0.2, 0.2);
    target_yaw_ = DyrosMath::minmax_cut(0.5 * joy->axes[3], -M_PI, M_PI); // Example axis for yaw control
}









///////////Someone who wrote before me(YOBEL)


/*
void RLController::compute()
{    // std::cout << "mujoco_virtual_[]: " << virtual_q_dot_.transpose() << std::endl;
    // std::cout << "q_[]: " << current_q_.transpose() << std::endl;
    if(this->rl_enable_)
    {
        observationAllocation(current_q_, current_q_dot_, virtual_q_dot_, base_quat_);
        // this->observation = this->observation.to(torch::kFloat);
        this->action = module.forward({this->observation}).toTensor();
        auto data_ptr = this->action.data_ptr<double>();
        // this->action = this->action.to(torch::kDouble);
        Eigen::Map<Eigen::VectorXd> desired_torque_(data_ptr, this->action.size(0), this->action.size(1));
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
    
        // self.base_lin_vel:  torch.Size([4096, 3])
        // self.base_ang_vel:  torch.Size([4096, 3])
        // self.projected_gravity:  torch.Size([4096, 3])
        // self.commands[:, :2]:  torch.Size([4096, 2])
        // (self.dof_pos - self.default_dof_pos):  torch.Size([4096, 6])
        // self.dof_vel:  torch.Size([4096, 6])
        // self.actions:  torch.Size([4096, 6])

        // 3 + 3 + 3 + 2 + 6 + 6 + 6 = 29(num_observation)
    

    Eigen::VectorXf base_lin_vel_ = virtual_q_dot.head(3).cast<float>()*2.0;
    // std::cout << "base_lin_vel_: " << virtual_q_dot_.transpose() << std::endl;
    VectorXf base_ang_vel_ = virtual_q_dot.tail(3).cast<float>()*0.25;
    VectorXf projected_gravity_ = quat_rotate_inverse(base_quat, this->gravity);
    VectorXf commands_ = Vector2d(4, 0).cast<float>()*2.0;

    // Vector6d dof_pos_ = current_q;
    // Vector6d dof_vel_ = current_q_dot*0.05;
    VectorXf dof_pos_;
    VectorXf dof_vel_;
    VectorXf action_;

    for(int i=0; i<3; i++)
    {
        dof_pos_[i] = (float)(current_q[i]-init_q_[i]);
        dof_pos_[i+3] = (float)(current_q[i+4]-init_q_[i+3]);
        dof_vel_[i] = (float)current_q_dot[i]*0.05;
        dof_vel_[i+3] = (float)current_q_dot[i+4]*0.05;
        action_[i] = (float)this->desired_torque_[i];
        action_[i+3] = (float)this->desired_torque_[i+4];
    }

    torch::Tensor base_lin_vel = torch::from_blob(base_lin_vel_.data(), {1, 3}, torch::kFloat);
    torch::Tensor base_ang_vel = torch::from_blob(base_ang_vel_.data(), {1, 3}, torch::kFloat);
    torch::Tensor projected_gravity = torch::from_blob(projected_gravity_.data(), {1, 3}, torch::kFloat);
    torch::Tensor commands = torch::from_blob(commands_.data(), {1, 2}, torch::kFloat);
    torch::Tensor dof_pos = torch::from_blob(dof_pos_.data(), {1, 6}, torch::kFloat);
    torch::Tensor dof_vel = torch::from_blob(dof_vel_.data(), {1, 6}, torch::kFloat);
    torch::Tensor action = torch::from_blob(action_.data(), {1, 6}, torch::kFloat);

    std::vector<torch::Tensor> tensor_list = {base_lin_vel, base_ang_vel, projected_gravity, commands, dof_pos, dof_vel, action};
    
    // this->observation = torch::zeros({1, this->observation_size});
    this->observation = torch::cat(tensor_list, 1);
    std::cout << "observation: " << observation << std::endl;
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
            if( mask[i] >= PRIORITY && mask[i] < PRIORITY * 2 )
            {
                desired_torque[i] = desired_torque_[i];
            }
        }
    }

}

Eigen::VectorXf RLController::quat_rotate_inverse(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) 
{//'v' is the global gravity vector
    Eigen::Vector3d q_vec = q.vec();
    double q_w = q.w();

    Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);
    Eigen::Vector3d b = q_vec.cross(v) * q_w * 2.0;
    Eigen::Vector3d c = q_vec * (q_vec.dot(v) * 2.0);

    // Eigen::VectorXf result = a.cast<float>() + b.cast<float>() + c.cast<float>();
    return (a - b + c).cast<float>();
}

*/


}