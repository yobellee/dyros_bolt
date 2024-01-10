#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "dyros_bolt_lib/dyros_bolt.h"
#include "math_type_define.h"

using namespace std;
using namespace Eigen;

class LinkData
{

public:
  // Update link i of rbdl link id. name : link name, mass : link mass, xipos : local center of mass position
  void Initialize(RigidBodyDynamics::Model &model_, int id_);

  bool CheckName(RigidBodyDynamics::Model &model_);

  // Update xpos, xipos, rotm.
  void UpdatePosition(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_);

  // update link velocity(6D, translation and rotation) from jacobian matrix Jac.
  void UpdateVW(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_, const Eigen::VectorVQd &q_dot_virtual_);

  void GetPointPos(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_, const Eigen::VectorVQd &q_dot_virtual_, Eigen::Vector3d &local_pos, Eigen::Vector3d &global_pos, Eigen::Vector6d &global_velocity6D);

  // Update COM jacobian
  void UpdateJacobian(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_);

  // Update COM jac + jac + vel
  void UpdateJacobian(RigidBodyDynamics::Model &model_, const Eigen::VectorQVQd &q_virtual_, const Eigen::VectorVQd &q_dot_virtual_);

  // set link Trajectory of id i.
  void SetTrajectory(Eigen::Vector3d position_desired, Eigen::Vector3d velocity_desired, Eigen::Matrix3d rotation_desired, Eigen::Vector3d rotational_velocity_desired);

  // set realtime trajectory of link from quintic spline.
  void SetTrajectoryQuintic(double current_time, double start_time, double end_time);

  // set realtime trajectory of link from quintic spline.
  void SetTrajectoryQuintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_desired);

  // set realtime trajectory of link from quintic spline.
  void SetTrajectoryQuintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d pos_desired);

  // set realtime trajectory of link from quintic spline.
  void SetTrajectoryQuintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired);

  void SetTrajectoryQuintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d acc_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired, Eigen::Vector3d acc_des);

  // set realtime trajectory of link from cubic spline.
  void SetTrajectoryCubic(double current_time, double start_time, double end_time);

  // set realtime trajectory of link from cubic spline.
  void SetTrajectoryCubic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_desired);

  // set realtime trajectory of link from cubic spline.
  void SetTrajectoryCubic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d pos_desired);

  // set realtime trajectory of link from cubic spline.
  void SetTrajectoryCubic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired);

  void SetTrajectoryLinear(double current_time, double accel_time, double start_time, double end_time);

  void SetTrajectoryLinear(double current_time, double accel_time, double start_time, double end_time, Eigen::Vector3d position_desired, Eigen::Vector3d position_init);

  // set realtime trajectory of rotation of link
  void SetTrajectoryRotation(double current_time, double start_time, double end_time, bool local_);

  // set realtime trajectory of rotation of link
  void SetTrajectoryRotation(double current_time, double start_time, double end_time, Eigen::Matrix3d rot_desired, bool local_);

  // quat trajectory using slerp
  void SetTrajectoryRotation(double current_time, double start_time, double end_time);

  // set link initial position and rotation. initial position for task control.
  void SetInitialWithPosition();

  void SetInitialWithTrajectory();

  void SetGain(double pos_p, double pos_d, double pos_a, double rot_p, double rot_d, double rot_a);

  Eigen::Matrix6Vf jac;
  Eigen::Matrix6Vf jac_com;

  Eigen::Matrix6Vd Jac();
  Eigen::Matrix6Vd JacCOM();

  // void Get_PointPos(Eigen::VectorQVQd &q_virtual_, Eigen::VectorVQd &q_dot_virtual, Eigen::Vector3d &local_pos, Eigen::Vector3d &global_pos, Eigen::Vector6d &global_velocity6D);

  // constant variables
  int id;
  double mass;
  // std::string name;

  // local COM position of body
  Eigen::Vector3d com_position;

  // inertial matrix
  Eigen::Matrix3d inertia;

  // local sensor point
  Eigen::Vector3d contact_point;
  Eigen::Vector3d sensor_point;

  // changing variables
  // rotation matrix
  Eigen::Matrix3d rotm;

  // global position of body
  Eigen::Vector3d xpos;

  // global COM position of body
  Eigen::Vector3d xipos;

  // global position of sensor at body
  Eigen::Vector3d xpos_sensor;

  // cartesian velocity of body
  Eigen::Vector3d v;

  Eigen::Vector3d vi;

  // rotational velocity of body
  Eigen::Vector3d w;

  // fstar of current link
  Eigen::Vector6d fstar;

  double roll, pitch, yaw;
  double roll_traj, pitch_traj, yaw_traj;
  double roll_init, pitch_init, yaw_init;

  // realtime traj of cartesian & orientation.
  //)) traj is outcome of cubic or quintic function, which will be used to make fstar!
  //  x : cartesian coordinate traj (3x1)
  //  v : cartesian velocity (3x1)
  //  r : rotational matrix of current orientation (3x3)
  //  w : rotational speed of current orientation (3x1)

  Eigen::Vector3d x_traj;
  Eigen::Vector3d v_traj;
  Eigen::Vector3d a_traj;

  Eigen::Matrix3d r_traj;
  Eigen::Vector3d w_traj;
  Eigen::Vector3d ra_traj;

  Eigen::Vector3d x_traj_local;
  Eigen::Vector3d v_traj_local;
  Eigen::Matrix3d r_traj_local;
  Eigen::Vector3d w_traj_local;

  Eigen::Vector3d x_init;
  Eigen::Vector3d v_init;
  Eigen::Matrix3d rot_init;
  Eigen::Vector3d w_init;

  Eigen::Vector3d xi_init;
  Eigen::Vector3d vi_init;

  Eigen::Vector3d x_init_local;
  Eigen::Vector3d v_init_local;
  Eigen::Matrix3d rot_init_local;
  Eigen::Vector3d w_init_local;

  Eigen::Vector3d x_task_init;
  Eigen::Vector3d v_task_init;
  Eigen::Vector3d a_task_init;
  Eigen::Matrix3d r_task_init;
  Eigen::Vector3d w_task_init;

  Eigen::Vector3d x_desired;
  Eigen::Matrix3d rot_desired;

  Eigen::Vector3d pos_p_gain;
  Eigen::Vector3d pos_d_gain;
  Eigen::Vector3d pos_a_gain;

  Eigen::Vector3d rot_p_gain;
  Eigen::Vector3d rot_d_gain;
  Eigen::Vector3d rot_a_gain;

  Eigen::Vector3d max_p_acc_;
  Eigen::Vector3d max_p_vel_;

  // RigidBodyDynamics::Model *model;
  Eigen::MatrixXd j_temp;
};

class EndEffector : public LinkData
{
public:
  void InitializeEE(LinkData &lk_, float x_length, float y_length, float min_force, float friction_ratio, float friction_ratio_z);
  void UpdateLinkData(LinkData &lk_);
  // Set Contact point, Contact jacobian
  void SetContact(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_);

  MatrixXd GetZMPConstMatrix();
  MatrixXd GetForceConstMatrix();
  // cartesian velocity of contact point at body
  Eigen::Vector3d v_contact;
  // cartesian velocity of contact point at body
  Eigen::Vector3d w_contact;
  Eigen::Matrix6Vf jac_contact;
  // global position of contact point at body
  Eigen::Vector3d xpos_contact;
  // local contact point

  double friction_ratio;
  double friction_ratio_z;

  double cs_x_length;
  double cs_y_length;

  Eigen::Vector6d contactForce;

  Eigen::Vector3d zmp;

  double contact_force_minimum;

  bool contact;
};

class TaskSpace
{
public:
  int task_dof_;

  MatrixXd J_task_;
  VectorXd f_star_;
  MatrixXd J_kt_;
  MatrixXd Null_task;
  MatrixXd Lambda_task_;
  VectorXd torque_h_;

  VectorXd f_star_qp_;
  VectorXd contact_qp_;

  // CQuadraticProgram hqp_;
  
  TaskSpace();
  TaskSpace(int task_dof);

  void Update(const MatrixXd &J_task, const VectorXd &f_star);
};