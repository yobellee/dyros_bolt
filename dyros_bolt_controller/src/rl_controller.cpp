#include "dyros_bolt_controller/rl_controller.h"
#include <cmath> // For the ELU function

namespace dyros_bolt_controller
{

RLController::RLController(const VectorQd& current_q, const VectorQd& current_q_dot, const Vector6d& virtual_q_dot, const Eigen::Quaterniond& base_quat, const double hz, const double& control_time) :
    current_q_(current_q), current_q_dot_(current_q_dot), virtual_q_dot_(virtual_q_dot), base_quat_(base_quat), hz_(hz), current_time_(control_time), total_dof_(DyrosBoltModel::HW_TOTAL_DOF)
{
    desired_torque_rl.setZero();
    //desired_torque_rl<<100, 100, 100, 100, 100, 100;//changed here!!

    initVariable();
    loadNetwork();

    rl_cmd_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>("/dyros_bolt/rl_vel_command", 10, &RLController::rlCmdCallback, this); // Add this line->to not conflict, you have to change the topic name differently
    std::cout<<"checking initial targets:"<<target_vel_x_<<"  "<<target_vel_y_<<"  "<<target_yaw_<<std::endl<<std::endl;
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RLController::joyCallback, this);
}


void RLController::setEnable(bool enable)
{
    this->rl_enable_ = enable;
    std::cout << "is rl_controller_ enabled?: " << enable << std::endl;
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
    state_var_.resize(num_cur_state, 1);//actually we don't use this
    state_buffer_.resize(num_cur_state * num_state_hist, 1);//120개 input //num_state_skip 안 두었음.--> 얼만큼의 state을 skip할지 몰라서 -->이게 맞대 굿
    
    //q_dot_lpf_.setZero();//actually we don't use this

    //Initializing q without consiering ankle!
    //q_init_ << 0.0, 0.0, -0.24, 0.6, -0.36, 0.0;

    torque_bound_ << 2.5, 2.5, 2.5, 2.5, 2.5, 2.5;//Torque bound는 bolt urdf 보고 함.
    //torque_bound_ << 1000, 1000,1000,1000,1000,1000;//Torque bound는 bolt urdf 보고 함.
}

void RLController::loadNetwork()
{
    state_buffer_.setZero();
    rl_action_.setZero();
    // Load network weights from files
    std::string path = "/home/dyros2/bolt_ws/src/dyros_bolt/dyros_bolt_controller/";//weight folder is in this directory

    std::ifstream file[8];
    file[0].open(path + "weight/0_weight.txt", std::ios::in); //weight랑 bias 이름 앞에 숫자 규칙이 있는건지 그냥 궁금 : ask 박사님
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

    // Projected gravity 
    Eigen::Vector3d projected_gravity = quat_rotate(base_quat_, Eigen::Vector3d(0, 0, -1));//0,0,-1? Not 0,0,-9.81 -->방향만 중요

    state_cur_(data_idx++) = projected_gravity(0);//state_curr_(0)=projected_gravity(0);
    state_cur_(data_idx++) = projected_gravity(1);
    state_cur_(data_idx++) = projected_gravity(2);
  
    // Command (x, y, yaw)
    std::cout<<"checking targets:"<<target_vel_x_<<"  "<<target_vel_y_<<"  "<<target_yaw_<<std::endl<<std::endl;
    state_cur_(data_idx++) = target_vel_x_;
    state_cur_(data_idx++) = target_vel_y_;
    state_cur_(data_idx++) = target_yaw_;

    // Joint positions --> extracting without ankle pinjoint
    for (int i = 0; i < 3; i++)
    {   
        std::cout<<"state_cur_"<<data_idx<<": "<< i << " " <<current_q_(i)<<std::endl;
        state_cur_(data_idx++) = current_q_(i);
    }
    for (int i = 4; i <= num_action; i++)
    {
        std::cout<<"state_cur_"<<data_idx<<": "<< i << " " << current_q_(i)<<std::endl;
        state_cur_(data_idx++) = current_q_(i);
    }


    // Joint velocities --> extracting without ankle pinjoint
    for (int i = 0; i < 3; i++)
    {
        std::cout<<"state_cur_"<<data_idx<<": "<< i << " "<<current_q_dot_(i)<<std::endl;        
        state_cur_(data_idx++) = current_q_dot_(i);
    }
    for (int i = 4; i <= num_action; i++)
    {
        std::cout<<"state_cur_"<<data_idx<<": "<< i << " "<<current_q_dot_(i)<<std::endl;        
        state_cur_(data_idx++) = current_q_dot_(i);
    }


    // Previous actions
    for (int i = 0; i < num_action; i++)
    {
        std::cout<<"state_cur_"<<data_idx<<": "<< rl_action_(i)<<std::endl;        
        state_cur_(data_idx++) = rl_action_(i);
    }


    state_buffer_.block(0, 0, num_cur_state * (num_state_hist - 1), 1) = state_buffer_.block(num_cur_state, 0, num_cur_state * (num_state_hist - 1), 1);

    //맨 뒤에는 최신 state update
    //mean값이 들어있는 파일이 없어서 normalize하진 않음--> ㅇㅇ 이게 맞음
    state_buffer_.block(num_cur_state * (num_state_hist - 1), 0, num_cur_state, 1) = state_cur_;
    //state_buffer_ = (state_cur_ - state_mean_).array() / state_var_.cwiseSqrt().array();
    std::cout<<"state_buffer_: "<<state_buffer_<<std::endl;
}

void RLController::feedforwardPolicy()
{   //INPUT TEST
    // state_buffer_ << 0.006745, -0.012829, -0.999895, 0.800000, 0.281065, -0.056200, -0.020981, 0.126828, -0.385926, 0.017118, 0.033514, 0.055365, -0.045377, 0.318585, -0.974033, -0.007720, -0.161393, 0.944074, 0.243126, 0.112755, -0.190724, -0.730870, -0.256435, 0.513908, 0.013436, -0.020553, -0.999698, 0.800000, 0.281065, -0.056200, -0.028936, 0.132538, -0.330088, 0.015319, -0.016344, 0.256393, -0.038167, -0.064588, 0.687877, -0.010185, -0.272112, 1.000615, -0.003724, 0.047158, 1.036185, 0.181626, -0.545985, -0.077336, 0.022620, -0.031963, -0.999233, 0.800000, 0.281065, -0.056200, -0.043542, 0.100002, -0.123719, 0.016536, -0.050968, 0.343557, -0.083490, -0.164117, 1.003824, 0.014247, -0.137155, 0.243072, -0.559171, -0.135530, 0.505279, 0.536301, -0.268832, -0.456461, 0.033760, -0.044874, -0.998422, 0.800000, 0.281065, -0.056200, -0.070361, 0.095900, -0.107367, 0.017819, -0.079059, 0.400001, -0.151484, 0.035752, -0.237333, 0.003118, -0.070578, -0.000054, -0.582305, -0.651182, -0.850146, 0.132070, 0.146664, 0.157073, 0.046654, -0.059598, -0.997131, 0.800000, 0.281065, -0.056200, -0.104433, 0.116280, -0.231406, 0.015357, -0.098567, 0.400000, -0.176760, 0.122868, -0.744951, -0.017243, -0.106578, -0.000000, -0.116090, -0.119972, -0.323813, -0.136617, -0.268221, -0.027150;
    //state_buffer_ << 0.053732,0.130249,-0.990024,0.800000,0.281065,-0.056200,-0.048375,0.352480,-1.138700,0.149122,0.198651,-0.130067,0.009541,-0.029013,1.003201,0.078145,0.051959,0.028770,-0.194382,0.481634,0.743227,-0.328708,-0.140526,0.058640,0.044367,0.145425,-0.988374,0.800000,0.281065,-0.056200,-0.053046,0.341183,-0.925798,0.152605,0.253182,-0.342310,-0.029117,-0.058150,1.005271,0.000073,0.249913,-1.009158,0.102668,0.009547,0.982810,-0.587500,-0.250455,-1.294461,0.035344,0.153590,-0.987502,0.800000,0.281065,-0.056200,-0.060924,0.321725,-0.708902,0.138465,0.309398,-0.576540,-0.039017,-0.093397,1.006934,-0.091678,0.245860,-1.013439,0.317442,-0.047693,1.180526,-0.788486,0.093153,-2.004452,0.027487,0.155690,-0.987423,0.800000,0.281065,-0.056200,-0.068241,0.318352,-0.626777,0.111786,0.360072,-0.811685,-0.067346,0.265844,-0.350450,-0.161003,0.221166,-1.013525,0.225367,-0.221760,1.625515,-0.320610,-0.047156,-2.326538,0.021882,0.146301,-0.988998,0.800000,0.281065,-0.056200,-0.089341,0.348886,-0.662925,0.073542,0.392798,-1.041802,-0.110187,0.113922,-0.125517,-0.200098,0.109590,-1.011332,0.340910,-0.033539,2.890638,0.100027,-0.404556,-2.336008;

    hidden_layer1_ = policy_net_w0_ * state_buffer_ + policy_net_b0_;
    hidden_layer1_ = hidden_layer1_.unaryExpr([](double x) { return (x >= 0) ? x : (std::exp(x) - 1); }); // ELU activation
    //hidden_layer1_ = hidden_layer1_.cwiseMax(0.0); // ReLU activation 

    hidden_layer2_ = policy_net_w1_ * hidden_layer1_ + policy_net_b1_;
    hidden_layer2_ = hidden_layer2_.unaryExpr([](double x) { return (x >= 0) ? x : (std::exp(x) - 1); }); // ELU activation
    //hidden_layer2_ = hidden_layer2_.cwiseMax(0.0); // ReLU activation

    hidden_layer3_ = policy_net_w2_ * hidden_layer2_ + policy_net_b2_;
    hidden_layer3_ = hidden_layer3_.unaryExpr([](double x) { return (x >= 0) ? x : (std::exp(x) - 1); }); // ELU activation
    //hidden_layer3_ = hidden_layer3_.cwiseMax(0.0); // ReLU activation 

    rl_action_ = action_net_w_ * hidden_layer3_ + action_net_b_;
}

void RLController::compute()
{
    if (rl_enable_)
    {
        double current_time = current_time_; // Assuming current_time_ is in seconds
        double elapsed_time = current_time - last_policy_update_time_;
        //std::cout<<"elapsed time: "<<elapsed_time<<std::endl;
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
            // desired_torque_rl(i) = DyrosMath::minmax_cut(rl_action_(i), -torque_bound_(i), torque_bound_(i));//torque 위 아래로 짜르는거 bound 줘서 // 민준이형꺼는 학습할 때 나오는 rl_action_이 바로 input torque이어서
            desired_torque_rl(i) = DyrosMath::minmax_cut(rl_action_(i)*torque_bound_(i), -torque_bound_(i), torque_bound_(i));//torque 위 아래로 짜르는거 bound 줘서 // 재용이형꺼는 학습할 때 나오는 rl_action_이 [-1,1]이어서
            //desired_torque_rl(i) = rl_action_(i)*torque_bound_(i);//torque 위 아래로 짜르는거 bound 줘서     
            //std::cout << "desired_torque_rl[" << i << "]: " << desired_torque_rl(i)<<std::endl;
            std::cout << "rl_action_[" << i << "]: " << rl_action_(i) << std::endl;            
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
        for (int i = 0; i < 3; i++)
        {
            if (mask[i] >= PRIORITY && mask[i] < PRIORITY * 2)
            {
                desired_torque[i] = desired_torque_rl(i);//TORQUE 1번joint~3번joint
            }
        }
        for (int i = 4; i < 7; i++)
        {
            if (mask[i] >= PRIORITY && mask[i] < PRIORITY * 2)
            {
                desired_torque[i] = desired_torque_rl(i-1);//Torque 4번joint~5번joint
                //std::cout<<"YOBEL updating rl torque"<<std::endl;
            }
        }

        // for(int i=0;i<total_dof_;i++)//Yobel Test the updated torque
        // {
        //     std::cout << "desired torque after rl: " <<desired_torque[i] << std::endl;
        // }
    }

    else//changed here//여기 바꿈//edit//edited
    {
        //desired_torque<<0,1,0,0,  0,1,0,0;
        // for (int i = 0; i < num_action; i++)
        // {
        //     std::cout << "desired_torque[" << i << "]: " << desired_torque(i) << std::endl;
        //     //std::cout << "rl_action_[" << i << "]: " << rl_action_(i) << std::endl;            
        // }
        //std::cout << "desired_torque : " << desired_torque.transpose() << std::endl;
    }
}



Eigen::VectorXd RLController::quat_rotate(const Eigen::Quaterniond& q, const Eigen::Vector3d& v)
{
    // q is the quaternion and v is the vector to be rotated
    Eigen::Vector3d q_vec = q.vec(); // Extract the vector part of the quaternion
    double q_w = q.w(); // Extract the scalar part of the quaternion

    // a = v * (2.0 * q_w * q_w - 1.0)
    Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);

    // b = 2.0 * q_w * q_vec.cross(v)
    Eigen::Vector3d b = 2.0 * q_w * q_vec.cross(v);

    // c = 2.0 * q_vec * q_vec.dot(v)
    Eigen::Vector3d c = 2.0 * q_vec * q_vec.dot(v);

    // Return the rotated vector
    return a + b + c;
}



//oystick으로 명령 안 주지만 target_을 정해주고자 이렇게 했어요. 
void RLController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    target_vel_x_ = DyrosMath::minmax_cut(0.5 * joy->axes[1], -0.2, 0.5);
    target_vel_y_ = DyrosMath::minmax_cut(0.5 * joy->axes[0], -0.2, 0.2);
    target_yaw_ = DyrosMath::minmax_cut(0.5 * joy->axes[3], -M_PI, M_PI); // Example axis for yaw control
}

void RLController::rlCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)//얘가 conflict가 있었던 거였어요 to 박사님.
{
    if (msg->data.size() == 3)
    {
        target_vel_x_ = msg->data[0];
        target_vel_y_ = msg->data[1];
        target_yaw_ = msg->data[2];
    }
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