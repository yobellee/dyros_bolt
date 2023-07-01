#include "dyros_bolt_controller/dyros_bolt_model.h"



namespace dyros_bolt_controller
{
constexpr const char* DyrosBoltModel::EE_NAME[2];
constexpr const size_t DyrosBoltModel::HW_TOTAL_DOF;
constexpr const size_t DyrosBoltModel::MODEL_DOF;

constexpr const size_t DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF;
  
// These should be replaced by YAML or URDF or something
const std::string DyrosBoltModel::JOINT_NAME[DyrosBoltModel::HW_TOTAL_DOF] = {
    "FL_SHOULDER","FL_UPPER_LEG","FL_LOWER_LEG","FL_ANKLE",
    "FR_SHOULDER","FR_UPPER_LEG","FR_LOWER_LEG","FR_ANKLE"};

// Dynamixel Hardware IDs
const int DyrosBoltModel::JOINT_ID[DyrosBoltModel::HW_TOTAL_DOF] = {
    1,2,3,4,
    5,6,7,8};       

DyrosBoltModel::DyrosBoltModel() :
    joint_start_index_{0, 4}
{
    base_position_.setZero();
    q_.setZero();
    q_ext_.setZero();
    extencoder_init_flag_ = false;

    std::string desc_package_path = ros::package::getPath("dyros_bolt_description");
    // std::string urdf_path = desc_package_path + "/robots/bolt.urdf";
    std::string urdf_path = desc_package_path + "/robots/bolt_with_passive.urdf";

    ROS_INFO("Loading DYROS BOLT description from = %s",urdf_path.c_str());
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_, true, false);
    ROS_INFO("Successfully loaded.");
    ROS_INFO("Total DoF = %d", model_.dof_count);
    ROS_INFO("Total DoF = %d", model_.q_size);
    if(model_.dof_count != MODEL_WITH_VIRTUAL_DOF)
    {
        ROS_WARN("The DoF in the model file and the code do not match.");
        ROS_WARN("Model file = %d, Code = %d", model_.dof_count, (int)MODEL_WITH_VIRTUAL_DOF);
    }

    for (size_t i=0; i<2; i++)
    {
        end_effector_id_[i] = model_.GetBodyId(EE_NAME[i]);
        ROS_INFO("%s: id - %d",EE_NAME[i], end_effector_id_[i]);
        std::cout << model_.mBodies[end_effector_id_[i]].mCenterOfMass << std::endl;
    }

    for (size_t i=0; i<HW_TOTAL_DOF; i++)
    {
        joint_name_map_[JOINT_NAME[i]] = i;
    }
}

void DyrosBoltModel::test()
{
    Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF, 1> q_vjoint;
    q_vjoint.setZero();

    updateKinematics(q_vjoint, q_vjoint);

    std::cout << "left_leg_jacobian_" << std::endl;
    std::cout << leg_jacobian_[0] << std::endl << std::endl;
    std::cout << "right_leg_jacobian_" << std::endl;
    std::cout << leg_jacobian_[1] << std::endl;
    std::cout << "currnet_transform_" << std::endl;
    std::cout << currnet_transform_[0].translation() << std::endl << std::endl;
    std::cout << currnet_transform_[1].translation() << std::endl << std::endl;
    std::cout << currnet_transform_[2].translation() << std::endl << std::endl;
    std::cout << currnet_transform_[3].translation() << std::endl << std::endl;
    std::cout << "com" << std::endl;
    std::cout << com_<< std::endl;
}

void DyrosBoltModel::updateKinematics(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot)
{
    q_virtual_ = q;
    q_virtual_dot_ = qdot;
    RigidBodyDynamics::UpdateKinematicsCustom(model_, &q, &qdot, NULL);

    getInertiaMatrix12DoF(&full_inertia_mat_);
    getInertiaMatrix12legDoF(&leg_inertia_mat_);
    getCenterOfMassPosition(&com_);
    getCenterOfMassPositionDot(&comDot_);
    for(unsigned int i=0; i<2; i++)
    {
        getTransformEndEffector((EndEffector)i, &currnet_transform_[i]);
        if (i < 2)
        {
            getJacobianMatrix6DoF((EndEffector)i, &leg_jacobian_[i]);
            getJacobianMatrix12DoF((EndEffector)i, &leg_with_vlink_jacobian_[i]);
        }
        else 
        {
        }
    }
}

void DyrosBoltModel::updateSensorData(const Eigen::Vector6d &r_ft, const Eigen::Vector6d &l_ft, const Eigen::Vector12d &q_ext, const Eigen::Vector3d &acc, const Eigen::Vector3d &angvel, const Eigen::Vector3d &grav_rpy)
{
    r_ft_wrench_ = r_ft;
    l_ft_wrench_ = l_ft;

    q_ext_ = q_ext;
    accel_ = acc;
    angvel_ = angvel;
    grav_rpy_ = grav_rpy;
}

void DyrosBoltModel::updateSimCom(const Eigen::Vector3d &sim_com)
{
    com_simulation_ = sim_com;
}

void DyrosBoltModel::updateSimGyro(const Eigen::Vector3d &sim_gyro)
{
    gyro_simulation_ = sim_gyro;
}

void DyrosBoltModel::updateSimAccel(const Eigen::Vector3d &sim_accel)
{
    accel_simulation_ = sim_accel;
}

void DyrosBoltModel::updateSimRfoot(const Eigen::Isometry3d &sim_rfoot)
{
    rfoot_simulation_ = sim_rfoot;
}

void DyrosBoltModel::updateSimLfoot(const Eigen::Isometry3d &sim_lfoot)
{
    lfoot_simulation_ = sim_lfoot;
}

void DyrosBoltModel::updateSimBase(const Eigen::Isometry3d &sim_base)
{
    base_simulation_ = sim_base;
}

void DyrosBoltModel::updateMujCom(const Eigen::Vector6d &sim_lfoot)
{
    q_virtual_1 = sim_lfoot;
}

void DyrosBoltModel::updateMujComDot(const Eigen::Vector6d &sim_base)
{
    q_dot_virtual_1 = sim_base;
}


void DyrosBoltModel::getTransformEndEffector // must call updateKinematics before calling this function
(EndEffector ee, Eigen::Isometry3d* transform_matrix)
{
    //Eigen::Vector3d gghg = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, end_effector_id_[ee], base_position_, false);
    transform_matrix->translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates
        (model_, q_virtual_, end_effector_id_[ee], base_position_, false);
    transform_matrix->linear() = RigidBodyDynamics::CalcBodyWorldOrientation
        (model_, q_virtual_, end_effector_id_[ee], false).transpose();
}

// void DyrosBoltModel::getTransformEndEffector // must call updateKinematics before calling this function
// (EndEffector ee, Eigen::Vector3d* position, Eigen::Matrix3d* rotation)
// {
//     *position = RigidBodyDynamics::CalcBodyToBaseCoordinates
//         (model_, q_virtual_, end_effector_id_[ee], base_position_, false);
//     *rotation = RigidBodyDynamics::CalcBodyWorldOrientation(
//         model_, q_virtual_ , end_effector_id_[ee], false).transpose();
// }

// void DyrosBoltModel::getTransformEndEffector
// (EndEffector ee, const Eigen::VectorXd& q, bool update_kinematics,
//  Eigen::Vector3d* position, Eigen::Matrix3d* rotation)
// {
//     Eigen::Matrix<double, 12, 1> q_new;
//     q_new = q_virtual_;
//     q_new.segment<6>(joint_start_index_[ee]) = q;
//     if (update_kinematics)
//     {
//     q_virtual_ = q_new;
//     }
//     *position = RigidBodyDynamics::CalcBodyToBaseCoordinates
//         (model_,q_new,end_effector_id_[ee], base_position_, update_kinematics);
//     *rotation = RigidBodyDynamics::CalcBodyWorldOrientation(
//         model_, q_new, end_effector_id_[ee], update_kinematics).transpose();
//     // RigidBodyDynamics::Calcpo
//     // model_.mBodies[0].mCenterOfMass
// }


void DyrosBoltModel::getJacobianMatrix6DoF(EndEffector ee, Eigen::Matrix<double, 6, 4> *jacobian)
{
    Eigen::MatrixXd full_jacobian(6,MODEL_WITH_VIRTUAL_DOF);
    full_jacobian.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, end_effector_id_[ee],
                                            Eigen::Vector3d::Zero(), full_jacobian, false);

    // swap
    jacobian->block<3, 4>(0, 0) = full_jacobian.block<3, 4>(3, joint_start_index_[ee]+6);
    jacobian->block<3, 4>(3, 0) = full_jacobian.block<3, 4>(0, joint_start_index_[ee]+6);

}

void DyrosBoltModel::getJacobianMatrix12DoF(EndEffector ee, Eigen::Matrix<double, 6, MODEL_WITH_VIRTUAL_DOF> *jacobian)
{
  // Non-realtime
  Eigen::MatrixXd full_jacobian(6,MODEL_WITH_VIRTUAL_DOF);
  full_jacobian.setZero();
  RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, end_effector_id_[ee],
                                         Eigen::Vector3d::Zero(), full_jacobian, false);

  switch (ee)
  {
  case EE_LEFT_FOOT:
      // swap
      // Virtual Link
      jacobian->block<3, 6>(0, 0) = full_jacobian.block<3, 6>(3, 0);
      jacobian->block<3, 6>(3, 0) = full_jacobian.block<3, 6>(0, 0);

      // left Leg Link
      jacobian->block<3, 4>(0, 6) = full_jacobian.block<3, 4>(3, joint_start_index_[ee]+6);
      jacobian->block<3, 4>(3, 6) = full_jacobian.block<3, 4>(0, joint_start_index_[ee]+6);
      break;
  case EE_RIGHT_FOOT:
    // swap
    // Virtual Link
    jacobian->block<3, 6>(0, 0) = full_jacobian.block<3, 6>(3, 0);
    jacobian->block<3, 6>(3, 0) = full_jacobian.block<3, 6>(0, 0);

    // right Leg Link
    jacobian->block<3, 4>(0, 8) = full_jacobian.block<3, 4>(3, joint_start_index_[ee]+6);
    jacobian->block<3, 4>(3, 8) = full_jacobian.block<3, 4>(0, joint_start_index_[ee]+6);
    break;
  }
}

void DyrosBoltModel::getCenterOfMassPosition(Eigen::Vector3d* position)
{
  RigidBodyDynamics::Math::Vector3d position_temp,position_dot;
  position_temp.setZero();
  Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, 1> qddot;
  qddot.setZero();
  //Eigen::Vector3d com_vel;
  //Eigen::Vector3d angular_momentum;
  double mass;

  RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_virtual_, q_virtual_dot_, mass, position_temp, &position_dot, NULL, false);
  //RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_, qdot, mass, position_temp, NULL, NULL, false);

  //RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_, qdot, mass, position_temp);

  *position = position_temp;
}

void DyrosBoltModel::getCenterOfMassPositionDot(Eigen::Vector3d* position)
{
  RigidBodyDynamics::Math::Vector3d position_temp, position_dot;
  position_temp.setZero();
  Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, 1> qddot;
  qddot.setZero();
  //Eigen::Vector3d com_vel;
  //Eigen::Vector3d angular_momentum;
  double mass;

  RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_virtual_, q_virtual_dot_, mass, position_temp, &position_dot, NULL,false);
  //RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_, qdot, mass, position_temp, NULL, NULL, false);

  //RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_, qdot, mass, position_temp);
  *position = position_dot;
}

void DyrosBoltModel::getInertiaMatrix12DoF(Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF> *inertia)
{
  // Non-realtime
  Eigen::MatrixXd full_inertia(MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF);
  full_inertia.setZero();
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, q_virtual_, full_inertia, false);

  inertia->block<MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF>(0, 0) = full_inertia.block<MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF>(0, 0);
}

void DyrosBoltModel::getInertiaMatrix12legDoF(Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF> *leg_inertia)
{
  // Non-realtime
  Eigen::MatrixXd full_inertia(MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF);
  full_inertia.setZero();
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, q_virtual_, full_inertia, false);

  leg_inertia->block<MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF>(0, 0) = full_inertia.block<MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF>(0, 0);
}

}