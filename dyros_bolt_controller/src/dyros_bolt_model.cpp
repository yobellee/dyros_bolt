#include "dyros_bolt_controller/dyros_bolt_model.h"
#include "fstream"



namespace dyros_bolt_controller
{
// std::ofstream torque_txt("/home/yong/Desktop/torque.txt");
constexpr const char* DyrosBoltModel::EE_NAME[2];
constexpr const size_t DyrosBoltModel::HW_TOTAL_DOF;
constexpr const size_t DyrosBoltModel::MODEL_DOF;

constexpr const size_t DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF;
  
// These should be replaced by YAML or URDF or something
const std::string DyrosBoltModel::JOINT_NAME[DyrosBoltModel::HW_TOTAL_DOF] = {
    "FL_HAA", "FL_HFE", "FL_KFE", "FL_ANKLE",
    "FR_HAA", "FR_HFE", "FR_KFE", "FR_ANKLE"};
/*
FL_HAA: Front Left_Hip Abduction/Adduction(다리 조이고 벌리는 거 생각)
FL_HFE: Front Left_Hip Flexion/Extension(앞차기 뒤차기 할때 움직이는 거 생각)
FL_KFE: Front Left_Knee Flexion/Extension
FL_ANKLE: 발목 움직이는 거
나머지는 오른쪽 발임.
*/


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
    ROS_INFO("Total DoF = %d", model_.qdot_size);

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
    // Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF+1, 1> q_vjoint;
    Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF, 1> q_vjoint;
    Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF, 1> qdot_vjoint;
    q_vjoint.setZero();
    q_vjoint(MODEL_WITH_VIRTUAL_DOF) = 1;//vector 첫번째 요소 index=0 -> MODEL_WITH_VIRTUAL_DOF=14이니까, 0~13 요소까지 접근 가능하게 하고, 이걸 넘어가면, outofrange로 처리하게 하려고
    // q_vjoint << 0, 0, 0.43, 0, 0, 0,
                // 0, 0.436332313, -0.872664626, 0.436332313,
                // 0, 0.436332313, -0.872664626, 0.436332313;

    qdot_vjoint.setZero();


    updateKinematics(q_vjoint, qdot_vjoint, qdot_vjoint);

    std::cout << "left_leg_jacobian_" << std::endl;
    std::cout << leg_jacobian_[0] << std::endl << std::endl;
    std::cout << "right_leg_jacobian_" << std::endl;
    std::cout << leg_jacobian_[1] << std::endl;
    std::cout << "current_transform_" << std::endl;
    std::cout << current_transform_[0].translation() << std::endl << std::endl;
    std::cout << current_transform_[1].translation() << std::endl << std::endl;
    std::cout << "com" << std::endl;
    std::cout << com_<< std::endl;
}

void DyrosBoltModel::updateKinematics(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot)
{
    // q_virtual_ = q;
    // q_virtual_dot_ = qdot;
    // q_virtual_ddot_ = qddot;

    RigidBodyDynamics::UpdateKinematicsCustom(model_, &q, &qdot, &qddot);
    //

    VectorXd G_, B_;//14개의 요소 가진 벡터들임. - G_와 B_에 nonlinear effect를 반영.(근데 안 씀 시뮬 상에서 하니까 그런듯)
    G_.setZero(MODEL_WITH_VIRTUAL_DOF);
    B_.setZero(MODEL_WITH_VIRTUAL_DOF);

    //there's no function for NonlinearEffects -->think the below two lines are trash
    RigidBodyDynamics::NonlinearEffects(model_, q, qddot, G_);//Purpose: to calculate the dynamic effects on the robot, when it experiences joint accelerations(qddot). --> It helps determine "how much Force or Torque is required" to counteract these effects to maintain a specific motion or state. 
    RigidBodyDynamics::NonlinearEffects(model_, q, qdot, B_);//Purpose: to compute the effects due to the robot's motion (velocities), capturing Coriolis and centrifugal forces specifically. Then by this information give the compensation

    // Gravity + Coriolis
    // std::cout << "GRAVITY"<<std::endl;
    // std::cout << G_.block<MODEL_DOF,1>(6,0).transpose() << std::endl;
    // std::cout << B_.block<MODEL_DOF,1>(6,0).transpose() << std::endl;



    //'block': Eigen library function that allows you to extract a submatrix or subvector from a larger matrix or vector. 여기선 'Model_DOF=8' By '1' vector 추출 --> 'G_'의 "7번째요소(index=6)~14번째요소(index=13)". 즉 8개 요소를 추출
    // 앞의 6 element = pelvis coord. info.여서  
    command_Torque = G_.block<MODEL_DOF,1>(6,0);// 'command_Torque' represents the forces that need to be countered or assisted by the actuators, because of the nonlinear effect. 


    // command_Torque.setZero(MODEL_DOF);

    // getInertiaMatrixDoF(&full_inertia_mat_);
    // getInertiaMatrixlegDoF(&leg_inertia_mat_);


    getCenterOfMassPosition(&com_);//calls the 'getCenterOfMassPosition' function with a pointer to the 'com_' variable. It calculates and sets the center of mass of the robot within the member variable 'com_'.
    getCenterOfMassPositionDot(&comDot_);// calculating and updating the velocity of the center of mass (COM) of the robot.
    for(unsigned int i=0; i<2; i++)//loop 2번 도는 이유: endeffector가 발 두개임.
    {
        //(EndEffector)0: left foot, (EndEffector)1: right foot
        getTransformEndEffector((EndEffector)i, &current_transform_[i]);
        getJacobianMatrix4DoF((EndEffector)i, &leg_jacobian_[i]);
        getJacobianMatrix14DoF((EndEffector)i, &leg_with_vlink_jacobian_[i]);
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
(EndEffector ee, Eigen::Isometry3d* transform_matrix)//transform_matrix 형태: 4X4 matrix --> 내가 아는 그 회전 3X3, 병진 3x1 같이 있는 거
{
    // *transform_matrix = rd_.link_[end_effector_id_[ee]].GetSpatialTranform();
    // transform_matrix->translation() = rd_.link_[end_effector_id_[ee]-2].xpos;
    // transform_matrix->linear() =rd_.link_[end_effector_id_[ee]-2].rotm;


    //'transform_matrix->translation()': accesses the translation component of the Eigen::Isometry3d matrix, which represents the position of the end effector in 3D space.
    //'transform_matrix->linear()': accesses the linear component of the Eigen::Isometry3d matrix, which represents the rotation or orientation of the end effector.
    transform_matrix->translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates
      (model_, q_virtual_, end_effector_id_[ee], base_position_, false);//CalcBodyToBaseCoordinates에서 'Body' refers to '(left or right)foot'
      //To get the end effector's position relative to the base
      //'transform_matrix->translation()' is a 3D vector 
    transform_matrix->linear() = RigidBodyDynamics::CalcBodyWorldOrientation
      (model_, q_virtual_, end_effector_id_[ee], false).transpose();// type: 3X3 rotation matrix// base frame에서 봤을 때, endeffector가 얼마나 orient했는지 알려고 transpose 시켜줘--> CalcBodyWorldOrientation의 결과는 body frame에서 본 world frame의 orient임.
}





//인자로 받는 자코비안 행렬: 6X4 matrix --> p벡터가 6개(병진,회전)요소로 이루어짐, 한쪽 발은 theta1, theta2, theta3, theta4(pinjoint)로 이루어짐. --> 이 함수는 계산된 자코비안을 이 두 번째 인자 'Eigen::Matrix<double, 6, 4> *jacobian'에 저장한다. 
// J = 'dp/dtheta1','dp/dtheta2','dp/dtheta3','dp/dtheta4'  --> 6행 4열 행렬
void DyrosBoltModel::getJacobianMatrix4DoF(EndEffector ee, Eigen::Matrix<double, 6, 4> *jacobian)
{
    // *jacobian = rd_.link_[end_effector_id_[ee]-2].jac_.middleCols<4>(joint_start_index_[ee]+6);
    Eigen::MatrixXd full_jacobian(6,MODEL_WITH_VIRTUAL_DOF);// 
    full_jacobian.setZero();

    // Calculates the Jacobian matrix of a point on the end effector in a 6-dimensional space (3 for position and 3 for orientation).
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, end_effector_id_[ee],
                                            Eigen::Vector3d::Zero(), full_jacobian, false);
    // swap



    //block은 submatrix추출하는것
    //joint_start_index[leftfoot]=0,    joint_start_index[rightfoot]=4
    //for more information fore below 2lines visit your IPAD Goodnotes(성균관대관련->학부연구생->DYROS->Code Analysis)
    jacobian->block<3, 4>(0, 0) = full_jacobian.block<3, 4>(3, joint_start_index_[ee]+6);
    jacobian->block<3, 4>(3, 0) = full_jacobian.block<3, 4>(0, joint_start_index_[ee]+6);
}

void DyrosBoltModel::getJacobianMatrix14DoF(EndEffector ee, Eigen::Matrix<double, 6, MODEL_WITH_VIRTUAL_DOF> *jacobian)
{
    Eigen::MatrixXd full_jacobian(6,MODEL_WITH_VIRTUAL_DOF);
    full_jacobian.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, end_effector_id_[ee],
                                            Eigen::Vector3d::Zero(), full_jacobian, false);
    switch (ee)
    {
    case EE_LEFT_FOOT:
        // Virtual Link
        // jacobian->block<6, 6>(0, 0) = rd_.link_[end_effector_id_[ee]-2].jac_.leftCols(6);
        // left Leg Link
        // jacobian->block<6, 4>(0, 6) = rd_.link_[end_effector_id_[ee]-2].jac_.leftCols<4>(joint_start_index_[ee]+6);
        // Virtual Link
        jacobian->block<3, 6>(0, 0) = full_jacobian.block<3, 6>(3, 0);
        jacobian->block<3, 6>(3, 0) = full_jacobian.block<3, 6>(0, 0);

        // left Leg Link
        jacobian->block<3, 4>(0, 6) = full_jacobian.block<3, 4>(3, joint_start_index_[ee]+6);
        jacobian->block<3, 4>(3, 6) = full_jacobian.block<3, 4>(0, joint_start_index_[ee]+6);
        break;
    case EE_RIGHT_FOOT:
        // Virtual Link
        // jacobian->block<6, 6>(0, 0) = rd_.link_[end_effector_id_[ee]-2].jac_.leftCols(6);
        // right Leg Link
        // jacobian->block<6, 4>(0, 10) = rd_.link_[end_effector_id_[ee]-2].jac_.leftCols<4>(joint_start_index_[ee]+6);
        // Virtual Link
        jacobian->block<3, 6>(0, 0) = full_jacobian.block<3, 6>(3, 0);
        jacobian->block<3, 6>(3, 0) = full_jacobian.block<3, 6>(0, 0);

        // left Leg Link
        jacobian->block<3, 4>(0, 10) = full_jacobian.block<3, 4>(3, joint_start_index_[ee]+6);
        jacobian->block<3, 4>(3, 10) = full_jacobian.block<3, 4>(0, joint_start_index_[ee]+6);
        break;
    }
}

void DyrosBoltModel::getCenterOfMassPosition(Eigen::Vector3d* position)
{
    // *position = rd_.com_pos;
    RigidBodyDynamics::Math::Vector3d com, com_dot;
    com.setZero();
    VectorXd qddot;
    qddot.setZero(MODEL_WITH_VIRTUAL_DOF);
    double mass;
    ///////////////////////////////////////////model//..q..qdot..qddot..mass..com..comdot.comddot..angularmomentum..change_of_angularmomentum..updatekin
    RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_virtual_, q_virtual_dot_, &qddot, mass, com, &com_dot, NULL, NULL, NULL, false);

    *position = com;
}

void DyrosBoltModel::getCenterOfMassPositionDot(Eigen::Vector3d* position_dot)
{
    // *position_dot = rd_.com_vel;
    RigidBodyDynamics::Math::Vector3d com, com_dot;
    com.setZero();
    VectorXd qddot;
    qddot.setZero(MODEL_WITH_VIRTUAL_DOF);
    double mass;

    RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_virtual_, q_virtual_dot_, &qddot, mass, com, &com_dot, NULL, NULL, NULL, false);
    //Purpose: to compute the center of mass position, total mass, and optionally other related dynamics of the model.


    *position_dot = com_dot; 
}

void DyrosBoltModel::getInertiaMatrixDoF(Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF> *inertia)
{
    // *inertia = rd_.A_;
}

void DyrosBoltModel::getInertiaMatrixlegDoF(Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF> *leg_inertia)
{
//   leg_inertia->block<MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF>(0, 0) = rd_.A_.block<MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF>(0, 0);
    // *leg_inertia = rd_.A_;
}

}