#ifndef CUSTOM_CONTROLLER_H
#define CUSTOM_CONTROLLER_H
#include "dyros_bolt_controller/dyros_bolt_model.h"
#include "fstream"


namespace dyros_bolt_controller
{
class CustomController
{
    public:
        
        static constexpr unsigned int PRIORITY = 8;

        CustomController(DyrosBoltModel& model, const VectorQd& current_q, const VectorQd& current_qdot, const double hz, const double& control_time) :
            total_dof_(DyrosBoltModel::HW_TOTAL_DOF), model_(model), current_q_(current_q), current_qdot_(current_qdot), hz_(hz), current_time_(control_time)
        {

        }

        

        void setEnable(bool enable);
        void setTarget();
        void setTarget(bool is_right_foot_swing, double x, double y, double z, double theta,
                                  double step_length_x, double step_length_y);

        void compute();
        
        void updateControlMask(unsigned int *mask);
        void writeDesired(const unsigned int *mask, VectorQd& desired_q);
        
        void parameterSetting();
        void updateInitialState();
        void getRobotState();

        void calculateFootStepTotal();
        void floatToSupportFootstep();

        void getComTrajectory();
        void getPelvTrajectory();
        void getFootTrajectory();
        void supportToFloat();
        void circling_motion();
        
        void comGenerator(const unsigned int norm_size, const unsigned planning_step_num);
        void onestepCom(unsigned int current_step_number, Eigen::VectorXd& temp_cx, Eigen::VectorXd& temp_cy);

        void computeIK(Eigen::Isometry3d float_pelv_transform,
                       Eigen::Isometry3d float_lleg_transform,
                       Eigen::Isometry3d float_rleg_transform,
                       Eigen::Vector8d& q_des);
        void updateNextStepTime();

    private:
        bool walking_enable_;
        bool motion_end_ = true;

        const double hz_;
        unsigned int walking_tick_ = 0;
        const double &current_time_; // updated by control_base

        double target_x_,target_y_,target_z_,target_theta_;
        double step_length_x_,step_length_y_;
        bool is_right_foot_swing_;

        unsigned int total_step_num_;
        unsigned int current_step_num_;
        double foot_height_;
        double t_double1_,t_double2_,t_rest_init_,t_rest_last_;
        double t_total_,t_temp_,t_last_,t_start_,t_start_real_;
        
        Vector8d q_des;
        VectorQd desired_q_;
        const VectorQd& current_q_;
        const VectorQd& current_qdot_;

        Eigen::Isometry3d pelv_traj_float_;
        Eigen::Isometry3d lfoot_traj_float_;
        Eigen::Isometry3d rfoot_traj_float_;

        Eigen::Isometry3d pelv_traj_support_;
        Eigen::Isometry3d lfoot_traj_support_;
        Eigen::Isometry3d rfoot_traj_support_;

        Eigen::Vector3d lfoot_traj_euler_support_;
        Eigen::Vector3d rfoot_traj_euler_support_;
        
        DyrosBoltModel &model_;
        const unsigned int total_dof_;

        Eigen::Vector3d com_desired_;
        Eigen::MatrixXd foot_step_;
        Eigen::MatrixXd foot_step_support_frame_;
        Eigen::MatrixXd ref_com_;

        Eigen::Vector3d com_float_init_;

        Eigen::Isometry3d pelv_float_init_;
        Eigen::Isometry3d lfoot_float_init_;
        Eigen::Isometry3d rfoot_float_init_;

        Eigen::Vector6d supportfoot_float_init_;
        Eigen::Vector6d swingfoot_float_init_;

        Eigen::Vector3d com_float_current_;
        Eigen::Vector3d com_float_current_dot_;

        Eigen::Isometry3d pelv_float_current_;
        Eigen::Isometry3d lfoot_float_current_;
        Eigen::Isometry3d rfoot_float_current_;

        Eigen::Isometry3d supportfoot_float_current_;
        
        Eigen::Vector3d com_support_init_;

        Eigen::Isometry3d pelv_support_init_;
        Eigen::Isometry3d lfoot_support_init_;
        Eigen::Isometry3d rfoot_support_init_;

        Eigen::Vector3d pelv_support_euler_init_;
        Eigen::Vector3d rfoot_support_euler_init_;
        Eigen::Vector3d lfoot_support_euler_init_;
        
        Eigen::Vector6d supportfoot_support_init_;

        Eigen::Isometry3d pelv_support_start_; 

        Eigen::Vector3d com_support_current_;

        Eigen::Isometry3d pelv_support_current_;
        Eigen::Isometry3d lfoot_support_current_;
        Eigen::Isometry3d rfoot_support_current_;
};
}

#endif // CUSTOM_CONTROLLER_H
