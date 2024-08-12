#ifndef RL_CONTROLLER_H
#define RL_CONTROLLER_H

#include "dyros_bolt_controller/dyros_bolt_model.h"
#include <torch/script.h> // Include the PyTorch C++ API
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>//changed here
#include <sensor_msgs/Joy.h>
#include <fstream>
#include "math_type_define.h"
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

namespace dyros_bolt_controller
{
    //---RL controller designed by YOBEL Sungkook Lee [Data: 2024.08.02]---//
class RLController
{
    public:
    static constexpr unsigned int PRIORITY = 32;
    //joint controller PRIORITY = 64 // 1등 우선순위
    //custom controller PRIORITY = 8 // 3등우선순위
    //rl controller PRIORITY = 32 // 2등 우선순위
    //--> 앞의 두 controller와 PRIORITY를 달리 둠.

    RLController(const VectorQd& current_q, const VectorQd& current_q_dot, const Vector6d& virtual_q_dot, const Eigen::Quaterniond& base_quat, const double hz, const double& control_time);

    void setEnable(bool enable);
    void compute();
    //void observationAllocation(const VectorQd& current_q, const VectorQd& current_q_dot, const Vector6d& virtual_q_dot, const Eigen::Quaterniond& base_quat);
    void updateControlMask(unsigned int *mask);
    void writeDesired(const unsigned int *mask, VectorQd& desired_torque);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void rlCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);//박사님 얘랑 conflict 된거였어요
    void initVariable();
    void loadNetwork();
    void processObservation();
    void feedforwardPolicy();

    Eigen::VectorXd quat_rotate(const Eigen::Quaterniond& q, const Eigen::Vector3d& v);

private:
    const double hz_;
    const double &current_time_;
    const unsigned int total_dof_;
    
    const VectorQd& current_q_;
    const VectorQd& current_q_dot_;
    const Vector6d& virtual_q_dot_;
    const Eigen::Quaterniond& base_quat_;//IMU data with filter  

    Vector3d gravity = Vector3d(0, 0, -9.81);

    torch::jit::script::Module module;
    
    Eigen::Matrix<double, 6, 1> torque_bound_;
    Vector6d desired_torque_rl;


    bool rl_enable_ = false;//changed (false였는데 true로)

    torch::Tensor observation;
    torch::Tensor action;

    static const int num_action = 6; // action size
    static const int num_cur_state = 24; // observation size
    static const int num_state_hist = 5; // number of historical states to maintain in the buffer
    static const int num_hidden1 = 512; // first hidden layer size
    static const int num_hidden2 = 256; // second hidden layer size
    static const int num_hidden3 = 128; // third hidden layer size

    Eigen::MatrixXd policy_net_w0_;
    Eigen::MatrixXd policy_net_b0_;
    Eigen::MatrixXd policy_net_w1_;
    Eigen::MatrixXd policy_net_b1_;
    Eigen::MatrixXd policy_net_w2_;
    Eigen::MatrixXd policy_net_b2_;
    Eigen::MatrixXd action_net_w_;
    Eigen::MatrixXd action_net_b_;
    Eigen::MatrixXd hidden_layer1_;
    Eigen::MatrixXd hidden_layer2_;
    Eigen::MatrixXd hidden_layer3_;
    Eigen::MatrixXd rl_action_;

    Eigen::MatrixXd state_;//we actually don't use this
    Eigen::MatrixXd state_cur_;
    Eigen::MatrixXd state_buffer_;
    Eigen::MatrixXd state_mean_;
    Eigen::MatrixXd state_var_;

    std::ofstream writeFile;

    bool is_on_robot_ = false;
    bool is_write_file_ = true;

    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    ros::Subscriber rl_cmd_sub_;


    double target_vel_x_ = 0.03; //여기 젤 최근에 바꿈 2024.08.09
    double target_vel_y_ = 0.0;
    double target_yaw_ = 0.0;

    //to calculate the policy frequency
    double rl_policy_frequency_;
    double last_policy_update_time_;    
    




    
    /* 원래 주신 옛날 RL 코드 
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
        std::cout << "RL Controller is initialized" << std::endl;

        observation = torch::zeros({1, 24});//
        action = module.forward({observation}).toTensor();
        action = action.to(torch::kDouble);
        auto data_ptr = action.data_ptr<double>();
        Eigen::Map<Eigen::VectorXd> desired_torque__(data_ptr, action.size(0), action.size(1));
        for(int i=0; i<3; i++)
        {
            this->desired_torque_[i] = desired_torque__[i];
            this->desired_torque_[i+4] = desired_torque__[i+3];
        }
        std::cout << "RL Controller is initialized" << std::endl;
        //testtest
        
        std::cout << "action torq: " << desired_torque__.transpose()<< std::endl;
        std::cout << "action torq: " << this->desired_torque_.transpose()<< std::endl;
    }
    void setEnable(bool enable);

    void compute();
    void observationAllocation(VectorQd current_q, VectorQd current_q_dot, Vector6d virtual_q_dot, Eigen::Quaterniond base_quat);
    void updateControlMask(unsigned int *mask);
    void writeDesired(const unsigned int *mask, VectorQd& desired_torque);

    Eigen::VectorXf quat_rotate_inverse(const Eigen::Quaterniond& q, const Eigen::Vector3d& v);

private: 
    const double hz_;
    const double &current_time_; // updated by control_base
    const unsigned int total_dof_ = DyrosBoltModel::HW_TOTAL_DOF;

    const VectorQd& current_q_;
    Vector6d init_q_;
    const VectorQd& current_q_dot_;
    const Vector6d& virtual_q_dot_;
    const Eigen::Quaterniond& base_quat_;
    Vector3d gravity = Vector3d(0, 0, -1);

    torch::jit::script::Module module;

    VectorQd desired_torque_;

    bool rl_enable_ = false;

    torch::Tensor observation;
    torch::Tensor action;

    int observation_size = 24;//
    int action_size;*/
};



}
#endif // RL_CONTROLLER_H
