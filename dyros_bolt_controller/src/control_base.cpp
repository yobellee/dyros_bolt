#include "dyros_bolt_controller/control_base.h"

namespace dyros_bolt_controller
{


ControlBase::ControlBase(ros::NodeHandle &nh, double Hz) :
  is_first_boot_(true), Hz_(Hz), control_mask_{}, total_dof_(DyrosBoltModel::HW_TOTAL_DOF),shutdown_flag_(false),
  joint_controller_(q_, q_dot_filtered_, control_time_),
  custom_controller_(model_, q_, q_dot_filtered_, Hz, control_time_),
  rl_controller_(q_, q_dot_, mujoco_virtual_dot_, base_quat_, Hz, control_time_),
  joint_control_as_(nh, "/dyros_bolt/joint_control", false)
{
  
  makeIDInverseList();

  //joint_control_as_: represents the action server that handles 'joint_control' actions
  joint_control_as_.start();// begin processing incoming action goals

  //nh:representing the node that will handle this publisher,Topic name: "/dyros_bolt/joint_state", Que size:3 ->3messages can be buffered at once
  /*
  Summary: This initializes the 'joint_state_pub_' publisher to
  - Publish messages of type JointState on the /dyros_bolt/joint_state topic.
  - Use the nh node handle for interacting with the ROS system.
  - Maintain a queue of up to 3 messages to handle potential bursts in message traffic.
  */
  joint_state_pub_.init(nh, "/dyros_bolt/joint_state", 3);//joint_state_pub_: publisher

  //msg: message being prepared for publishing
  //msg의 name, angle, velocity, current, error는 모두 8개의 요소 가진 vector
  joint_state_pub_.msg_.name.resize(DyrosBoltModel::HW_TOTAL_DOF);
  joint_state_pub_.msg_.angle.resize(DyrosBoltModel::HW_TOTAL_DOF);
  joint_state_pub_.msg_.velocity.resize(DyrosBoltModel::HW_TOTAL_DOF);
  joint_state_pub_.msg_.current.resize(DyrosBoltModel::HW_TOTAL_DOF);
  joint_state_pub_.msg_.error.resize(DyrosBoltModel::HW_TOTAL_DOF);

  joint_robot_state_pub_.init(nh, "/joint_states", 5);
  joint_robot_state_pub_.msg_.name.resize(DyrosBoltModel::HW_TOTAL_DOF);
  joint_robot_state_pub_.msg_.position.resize(DyrosBoltModel::HW_TOTAL_DOF);
  joint_robot_state_pub_.msg_.velocity.resize(DyrosBoltModel::HW_TOTAL_DOF);
  joint_robot_state_pub_.msg_.effort.resize(DyrosBoltModel::HW_TOTAL_DOF);

  nh.getParam("Kp", pos_kp);//pos_kp라는 variable에, Kp라는 parameter가 저장된다.
  nh.getParam("Kv", pos_kv);
  nh.getParam("K_tau", k_tau);
  // for (size_t i = 0; i < k_tau.size(); ++i) {
  //   k_tau[i] = 0.22;
  // }

  for (int i=0; i< DyrosBoltModel::HW_TOTAL_DOF; i++)
  {
      joint_state_pub_.msg_.name[i] = DyrosBoltModel::JOINT_NAME[i];
  }
  for (int i=0; i< DyrosBoltModel::HW_TOTAL_DOF; i++)
  {
      joint_robot_state_pub_.msg_.name[i] = DyrosBoltModel::JOINT_NAME[i];
  }

  //"/dyros_bolt/joint_command 토픽"을 subscribe하는 joint_command_sub_이라는 subscribe class 생성.| buffer size:3 | &ControlBase::jointCommandCallback: Address of the callback function to execute when a message arrives | "/dyros_bolt/joint_command": topic name. | this: pointer to the instance of the "ControlBase" class->this앞의 항에 해당하는 함수가 ControlBase의 객체에 접근할 수 있도록 하기 위해서.
  joint_command_sub_ = nh.subscribe("/dyros_bolt/joint_command", 3, &ControlBase::jointCommandCallback, this);
  custom_command_sub_ = nh.subscribe("/dyros_bolt/custom_command",3, &ControlBase::customCommandCallback,this);
  rl_command_sub_ = nh.subscribe("/dyros_bolt/rl_command",3, &ControlBase::rlCommandCallback,this); //이전에 conflict는 Topic name이 같았기 때문
  shutdown_command_sub_ = nh.subscribe("/dyros_bolt/shutdown_command", 1, &ControlBase::shutdownCommandCallback,this);
  parameterInitialize();
  model_.test(); //test를 통해서, 대충 예상한 값으로 나오는지 확인.
}





void ControlBase::makeIDInverseList()
{
  for(int i=0;i<DyrosBoltModel::HW_TOTAL_DOF; i++)
  {
    joint_id_[i] = DyrosBoltModel::JOINT_ID[i];
    //위 배열은 다음과 같이 구성: 1,2,3,4,5,6,7,8

    joint_id_inversed_[DyrosBoltModel::JOINT_ID[i]] = i;
    //위 배열은 다음과 같이 구성: joint_id_inversed_[]={0,0,1,2,3,4,5,6}
   
  }
}

void ControlBase::parameterInitialize()
{
  q_.setZero();
  q_dot_.setZero();
  q_dot_filtered_.setZero();
  torque_.setZero();
  desired_torque_.setZero();
//   left_foot_ft_.setZero();
//   left_foot_ft_.setZero();
  desired_q_.setZero();
  extencoder_init_flag_ = false;
}

void ControlBase::readDevice()
{
  ros::spinOnce();//It processes a single round of callbacks for incoming messages. Essentially, it checks for new data (messages) that might have arrived from subscribed topics,   services, or actions.

  // Action
  if (joint_control_as_.isNewGoalAvailable())
  {
    jointControlActionCallback(joint_control_as_.acceptNewGoal());
    //click jointControlActionCallback!
  }
}

void ControlBase::update()
{
  //q_ext_는 component가 12개인 vector --> 얘 쓰레기 코드래, 안 쓴데, q_ext(실제 q_에 노이즈 낀거임)는 나의 고려사항이 아님
  // q_ 는 요소 8개인 vector-> 6개 모터 + pinjoint 2개
  //vector 자기 자신을 dot product했을 때 0이 나오는 것은 벡터의 모든 요소가 0이라는 소리. --> 0인 벡터가 있는지 체크하는 if문 
  if(extencoder_init_flag_ == false && q_ext_.transpose()*q_ext_ !=0 && q_.transpose()*q_ !=0)
  {
    for (int i=0; i<12; i++)
      extencoder_offset_(i) = q_(i)-q_ext_(i);

    extencoder_init_flag_ = true;// offsets have been initialized
  }

  if(extencoder_init_flag_ == true)
  {
    q_ext_offset_ = q_ext_ + extencoder_offset_;
  }

  //imu_data_ is written in Quaternion  
  /*to convert the quaternion (which consists of imu_data_.x(), imu_data_.y(), imu_data_.z(), imu_data_.w()) 
  into Euler angles stored in imu_grav_rpy_(순서대로 roll,pitch, yaw)*/
  DyrosMath::toEulerAngle(imu_data_.x(), imu_data_.y(), imu_data_.z(), imu_data_.w(), imu_grav_rpy_(0), imu_grav_rpy_(1), imu_grav_rpy_(2));
  model_.updateSensorData(right_foot_ft_, left_foot_ft_, q_ext_offset_, accelometer_, gyro_, imu_grav_rpy_);

// DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF = 14 = x,y,z,alpha, beta, gamma + 8개의 joint
//x,y,z,alpha,beta,gamma: 움직이는 몸통의 위치 정보(pelvis(골반) coordinate)
  Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF, 1> q_vjoint; //'q_vjoint'는 14개의 요소 가진 vector//원래 이 템플릿은 6개의 term 필요한데, 3개만 있는 이유 --> 나머지는 default value로 자동
  Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF, 1> q_vjoint_dot, q_vjoint_ddot;
  q_vjoint.setZero();
  q_vjoint_dot.setZero();
  q_vjoint.segment<DyrosBoltModel::MODEL_DOF>(6) = q_.head<DyrosBoltModel::MODEL_DOF>(); 
  //'q_vjoint'는 pelvis coord. 포함한 14개의 요소로 이루어짐. 'q_'는 8개의 joint 정보 있음
  //이 코드는 'q_vjoint'의 "pelvis coord. 이후의 7번째 요소~14번째의 요소"를, 'q_'의 값으로 설정하는 코드.


  // q_dot_filtered_ = q_dot_; --> we don't have to consider q_dot_filtered in this scope
  DyrosMath::lowPassFilter<DyrosBoltModel::HW_TOTAL_DOF>(q_dot_, q_dot_filtered_, 1.0 / Hz_, 0.05);
  q_vjoint_dot.segment<DyrosBoltModel::MODEL_DOF>(6) = q_dot_filtered_.head<DyrosBoltModel::MODEL_DOF>();

  q_vjoint_ddot.setZero(); //because our primary focus is on position and velocity control rather than precise acceleration tracking. --> Also, setting acc. as zero can lead to increased stability and robustness

  model_.updateKinematics(q_vjoint, q_vjoint_dot, q_vjoint_ddot);  // Update end effector positions and Jacobians
}



void ControlBase::compute()
{
  // start_time_에서 end_time_까지 가기 위한 중간중간의 desired_q 값 계속 계산하는거.-> 상황에 따라서, 선택하는 compute가 다름
  joint_controller_.compute();
  custom_controller_.compute();
  rl_controller_.compute();

  //ControlMask to help coordinate these controllers by specifying which joints each controller is responsible for at any given moment.
  joint_controller_.updateControlMask(control_mask_);
  custom_controller_.updateControlMask(control_mask_);
  rl_controller_.updateControlMask(control_mask_);
    //control_mask_[8]=[7,0,0,0,0,0,0,0]

  joint_controller_.writeDesired(control_mask_, desired_q_);
  custom_controller_.writeDesired(control_mask_, desired_q_); 
  rl_controller_.writeDesired(control_mask_, desired_torque_);

  // Torque Control --> 'PD control' + 'feedforward'
  // std::cout << "desired_q_ : " << desired_q_.transpose() << std::endl;
  // for (int i = 0; i < DyrosBoltModel::MODEL_DOF; i++)// 8번 반복 
  // 여기 그냥 아예 지워: why--> 앞에서 "rl_controller_.writeDesired(control_mask_, desired_torque_);"로 torque값 받으니까.
  // {
  //   desired_torque_[i] = pos_kp[i] * (desired_q_[i] - q_[i]) + pos_kv[i] * (q_dot_filtered_[i]) + model_.command_Torque(i);// don't think about filterd
  //   // desired_torque_[i] = desired_q_(i);
  //   // desired_torque_[i] = model_.command_Torque(i);
  //   // std::cout << "desired_torque_[i] : " << desired_torque_.transpose() << std::endl;
  // }

  tick_ ++;//'tick': compute 함수가 몇번 불렸는지 count해주는 놈.
  control_time_ = tick_ / Hz_;// keeps track of the elapsed time since the start of the control loop in seconds.
}

void ControlBase::reflect()
{
  // dyros_bolt_msgs::WalkingState msg;
  joint_robot_state_pub_.msg_.header.stamp = ros::Time::now(); //assigns the current time to the stamp field of the header in the joint_robot_state(JointState클래스의 객체) message
/* 
- header: provide information about the state of the joints at a particular point in time
- stamp : records the exact time when the message data is relevant or was captured. Important for synchronizing data across different parts of a robotic system --> timestamp means recording time 
- ros::Time::now() : returns the current time according to the ROS clock
*/

// HW_TOTAL_DOF = 8
  for (int i=0; i<DyrosBoltModel::HW_TOTAL_DOF; i++)
  {
    joint_state_pub_.msg_.angle[i] = q_(i); //current q --> joint 8개의 각도
    joint_state_pub_.msg_.velocity[i] = q_dot_(i);
    joint_state_pub_.msg_.current[i] = torque_(i); //current joint torque
  }

  for (int i=0; i<DyrosBoltModel::HW_TOTAL_DOF; i++)
  {
    joint_robot_state_pub_.msg_.position[i] = q_(i);
    joint_robot_state_pub_.msg_.velocity[i] = q_dot_filtered_(i); 
    joint_robot_state_pub_.msg_.effort[i] = desired_torque_(i);
  }

  //'trylock()': To ensure that the publisher is not currently being accessed by another thread.
  //'unlockAndPublish()': To publish the message to the ROS topic in a thread-safe manner
  if(joint_state_pub_.trylock())
  {
    joint_state_pub_.unlockAndPublish();
  }
  if(joint_robot_state_pub_.trylock())
  {
    joint_robot_state_pub_.unlockAndPublish();
  }

  //Checking if the action server status
  if(joint_control_as_.isActive())// determine whether there is an ongoing action that the action server 'joint_control_as_' is handling
  {
    bool all_disabled = true;//모든 joint가 disabled되었는지 확인하는 용도
    for (int i=0; i<DyrosBoltModel::HW_TOTAL_DOF; i++)
    {
      if (joint_controller_.isEnabled(i))
      {
        all_disabled = false;
      }
    }

    //To declare the action as successfully completed if all joints are disabled.
    if (all_disabled)
    {
      joint_control_result_.is_reached = true;//indicating that the goal has been reached.
      joint_control_as_.setSucceeded(joint_control_result_);//Informs the action server that the goal has been successfully completed
    }
  }
  // if (walking_controller_.walking_state_send == true)
  //   {
  //     walkingState_msg.data = walking_controller_.walking_end_;
  //     walkingstate_command_pub_.publish(walkingState_msg);
  //   }
}




void ControlBase::jointControlActionCallback(const dyros_bolt_msgs::JointControlGoalConstPtr &goal) // this 'goal' data contains joint 'command's
{/*
"command"라는 객체 안에는 다음의 멤버가 있음.
  header: metadata about the message, such as a timestamp and frame id
  name: names of the joints that are to be controlled-> 8개의 joint--> Pin joint는 90도로 유지
  position: Stores the target positions for each joint specified in the name vector 
            --> 각 joint 얼마나 회전할지를 말하는듯
  duration:  Stores the duration over which each joint should move to reach its target position. This allows for smooth transitions and speed control.
*/
  for (unsigned int i=0; i<goal->command.name.size(); i++)
  {//goal->command.name.size(): The number of joint commands(8개= pinjoint까지 command 받아-pinjoint는 90도로 유지) in the goal message.
    if(model_.isPossibleIndex(goal->command.name[i]))
    {
      joint_controller_.setTarget(model_.getIndex(goal->command.name[i]), goal->command.position[i], goal->command.duration[i]);
      //'command.duration[i]' 동안, 'command.name[i]'이라는 joint의 위치를, 'command.position[i]'까지 planning하려고 target 설정.
      // 이때, start_time_, end_time_, start_q_, start_q_dot_, target_q_ 변수가 초기화돼. 참고: 'goal->command.position[i]'가 i번째 joint의 'target_q_'값을 초기화 해줘(target_q_(joint_number) = goal->command.position[i];)

      joint_controller_.setEnable(model_.getIndex(goal->command.name[i]), true);
      //activates or deactivates a joint at a specific index, allowing or preventing it from moving
      //jont_number가 total_dof_보다 작으면, true로 -->각각의 joint를 enable하는 단계라고 생각해라.
    }
  }
  joint_control_feedback_.percent_complete = 0.0;//action being initialized. --> this might be updated periodically as the joints move towards their targets.
}

void ControlBase::jointCommandCallback(const dyros_bolt_msgs::JointCommandConstPtr& msg)
{
  for (unsigned int i=0; i<msg->name.size(); i++)
  {
    if(model_.isPossibleIndex(msg->name[i]))
    {
      joint_controller_.setTarget(model_.getIndex(msg->name[i]), msg->position[i], msg->duration[i]);
      joint_controller_.setEnable(model_.getIndex(msg->name[i]), true);
    }
  }
}

void ControlBase::customCommandCallback(const dyros_bolt_msgs::CustomCommandConstPtr &msg)
{
  if(msg->custom_mode == dyros_bolt_msgs::CustomCommand::WALK)
  {
    custom_controller_.setEnable(true);
    custom_controller_.setTarget(msg->first_foot_step, msg->x, msg->y, msg->z, msg->theta, msg->step_length_x, msg->step_length_y);
    std::cout << "Custom Walking Command Received" << std::endl;
  }
  else
  {
    custom_controller_.setEnable(false);
  }
}


// 박사님 제가 이게 conflict를 일으킨다 해서 지웠거든요?? --> conflict원인 target subscribe한 것도 똑같은 Topic name이었음.
void ControlBase::rlCommandCallback(const std_msgs::BoolConstPtr& msg)
{
  rl_controller_.setEnable(msg->data);
}

void ControlBase::shutdownCommandCallback(const std_msgs::StringConstPtr &msg)
{
  if (msg->data == "Shut up, BOLT.")
  {
    shutdown_flag_ = true;
  }
}

}