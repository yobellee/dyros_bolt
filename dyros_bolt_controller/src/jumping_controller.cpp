// #include "dyros_bolt_controller/jumping_controller.h"
#include "dyros_bolt_controller/dyros_bolt_model.h"

namespace dyros_bolt_controller
{

void JumpingController::setEnable(bool enable)
{
    jumping_enable_ = enable;
    desired_q_ = current_q_;
}

void JumpingController::setTarget()
{
    parameterSetting();
}

void JumpingController::parameterSetting()
{
    jumping_tick_ = 0;
}

void JumpingController::compute()
{
    if(jumping_enable_ == true)
    {
        updateInitialState();  
        getRobotState();  

        if(motion_end_)
        {
            getComTrajectory();
            getPelvTrajectory();
            getFootTrajectory();
            supportToFloat();
            computeIK(pelv_traj_float_, lfoot_traj_float_, rfoot_traj_float_, q_des);

            for(int i=0; i<8; i++)
            { desired_q_(i) = q_des(i); } 

            updateNextStepTime();
        }
        else
        {
            desired_q_ = current_q_;
        }
    }
}

void JumpingController::updateControlMask(unsigned int *mask)
{
    if(jumping_enable_)
    {
        for (int i=0; i<total_dof_; i++)
        {
            mask[i] = (mask[i] | PRIORITY);
        }
        // mask[3] = mask[3] & ~PRIORITY; 
        // mask[7] = mask[7] & ~PRIORITY; 
    }
    else
    {
        for (int i=0; i<total_dof_; i++)
        {
            mask[i] = (mask[i] & ~PRIORITY);
        }
    }
}

void JumpingController::writeDesired(const unsigned int *mask, VectorQd& desired_q)
{
    for(unsigned int i=0; i<total_dof_; i++)
    {     
        if( mask[i] >= PRIORITY && mask[i] < PRIORITY * 2 )
        {
            desired_q(i) = desired_q_(i);
        }         
    }
}

void JumpingController::updateInitialState()
{
    if(jumping_tick_ == 0)
    {
        com_float_init_ = model_.getCurrentCom();
        pelv_float_init_.setIdentity();
        lfoot_float_init_ = model_.getCurrentTransform((DyrosBoltModel::EndEffector)(0));
        rfoot_float_init_ = model_.getCurrentTransform((DyrosBoltModel::EndEffector)(1));  
    }

}

void JumpingController::getRobotState()
{
    Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF+1, 1> q_temp;
    Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF, 1>  qdot_temp, qddot_temp;
    q_temp.setZero();
    qdot_temp.setZero(); 
    qddot_temp.setZero(); 

    q_temp.segment<8>(6) = current_q_.segment<8>(0);   
    qdot_temp.segment<8>(6)= current_qdot_.segment<8>(0);

    model_.updateKinematics(q_temp, qdot_temp, qddot_temp);
    com_float_current_ = model_.getCurrentCom();
    com_float_current_dot_= model_.getCurrentComDot();
    lfoot_float_current_ = model_.getCurrentTransform((DyrosBoltModel::EndEffector)0); 
    rfoot_float_current_ = model_.getCurrentTransform((DyrosBoltModel::EndEffector)1);

    pelv_float_current_.setIdentity();

}

void JumpingController::getComTrajectory()
{

}

void JumpingController::getPelvTrajectory()
{

    pelv_traj_float_.translation()(0) = pelv_float_init_.translation()(0);
    pelv_traj_float_.translation()(1) = pelv_float_init_.translation()(1);
    pelv_traj_float_.translation()(2) = pelv_float_current_.translation()(2) 
        + 1.0*(DyrosMath::cubic(jumping_tick_, 0, 2*hz_, pelv_float_init_.translation()(2), pelv_float_init_.translation()(2)-0.05,0,0) - pelv_float_current_.translation()(2));
}   

void JumpingController::getFootTrajectory()
{
    lfoot_traj_float_ = lfoot_float_init_;
    rfoot_traj_float_ = rfoot_float_init_;
}

void JumpingController::supportToFloat()
{

}


void JumpingController::computeIK(Eigen::Isometry3d float_pelv_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector8d& q_des)
{
    Eigen::Vector3d L_r, R_r;
    Eigen::Matrix3d L_Hip_rot, R_Hip_rot;

    Eigen::Vector3d L_1, L_2, L_3, R_1, R_2, R_3;

    L_1 << 0, +0.0145, -0.0386;
    R_1 << 0, -0.0145, -0.0386;

    L_2 << 0, +0.0374, -0.2;
    R_2 << 0, -0.0374, -0.2;
    
    L_3 << 0, +0.008, -0.2;
    R_3 << 0, -0.008, -0.2;

    L_Hip_rot = float_pelv_transform.rotation().transpose() * float_lleg_transform.rotation();
    R_Hip_rot = float_pelv_transform.rotation().transpose() * float_rleg_transform.rotation();

    L_r = float_lleg_transform.translation() - float_pelv_transform.translation();
    R_r = float_rleg_transform.translation() - float_pelv_transform.translation();


    q_des(0) = atan2(L_Hip_rot(2,1), L_Hip_rot(1,1));
    q_des(1) = asin((L_r(0) + L_3(2)*L_Hip_rot(0,0)) / L_2(2));
    q_des(2) = atan2(L_Hip_rot(0,0), L_Hip_rot(0,2)) - q_des(1);

    q_des(4) = atan2(R_Hip_rot(2,1), R_Hip_rot(1,1));
    q_des(5) = asin((R_r(0) + R_3(2)*R_Hip_rot(0,0)) / R_2(2));
    q_des(6) = atan2(R_Hip_rot(0,0), R_Hip_rot(0,2)) - q_des(4);

    q_des(3) = 0;
    q_des(7) = 0;
}   

void JumpingController::updateNextStepTime()
{
    jumping_tick_ ++;
}





}
