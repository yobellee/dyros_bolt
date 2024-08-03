#include "dyros_bolt_controller/custom_controller.h"
#include "dyros_bolt_controller/dyros_bolt_model.h"

namespace dyros_bolt_controller
{
    // std::ofstream output("/home/yong/Desktop/data.txt");
void CustomController::setEnable(bool enable)
{
    walking_enable_ = enable;
    desired_q_ = current_q_;
}

// void CustomController::setTarget()
// {
//     parameterSetting();
// }

void CustomController::setTarget(bool is_right_foot_swing, double x, double y, double z, double theta,
                                  double step_length_x, double step_length_y)
{
    target_x_ = x;
    target_y_ = y;
    target_z_ = z;
    target_theta_ = theta;
    step_length_x_ = step_length_x;
    step_length_y_ = step_length_y;
    is_right_foot_swing_ = is_right_foot_swing;  

    parameterSetting();
}

void CustomController::parameterSetting()
{
    t_double1_ = 0.10*hz_; 
    t_double2_ = 0.10*hz_;
    t_rest_init_ = 0.05*hz_;
    t_rest_last_ = 0.05*hz_;
    t_total_= 1.2*hz_;
    t_temp_ = 3.0*hz_;
    t_last_ = t_total_ + t_temp_ ; 
    t_start_ = t_temp_ + 1 ;
    
    t_start_real_ = t_start_ + t_rest_init_;

    current_step_num_ = 0;
    foot_height_ = 0.025; 
    
    walking_tick_ = 0;
}

void CustomController::compute()
{
    if(walking_enable_ == true)//determine whether the walking or motion sequence should be active. False면 will not attempt to compute or perform walking motion. Instead, it will hold its current position.
    {
        updateInitialState(); //Initializing the state of the robot when the walking begins or when a step change occurs --> sets the initial conditions for the center of mass (CoM), pelvis, and feet positions based on the current step information.

        getRobotState(); //Update the controller with the current state of the robot
        
        if(motion_end_)//check if the current motion sequence has ended.
        {
            // getComTrajectory();
            // getPelvTrajectory();
            // getFootTrajectory();
            // supportToFloat();
            circling_motion();//발 계속 굴리는 거 생각

            computeIK(pelv_traj_float_, lfoot_traj_float_, rfoot_traj_float_, q_des); //computes the inverse kinematics (IK) to determine the desired joint positions (q_des) based on the current trajectories of the pelvis and feet



            // output << walking_tick_ <<"\t" << lfoot_traj_float_.translation()(2) <<"\t" << lfoot_traj_float_.translation()(0) << "\t" 
            //                                << rfoot_traj_float_.translation()(2) <<"\t" << rfoot_traj_float_.translation()(0) << "\t"
            //                                << q_des(0) << "\t" << q_des(1) << "\t" << q_des(2) << "\t" << q_des(3) << "\t" << q_des(4) << "\t" << q_des(5) << "\t" << q_des(6) << "\t" << q_des(7) << "\n";


            for(int i=0; i<8; i++)
            { desired_q_(i) = q_des(i); } 
            // std::cout << "desired_q_ : " << desired_q_.transpose() << std::endl;

            //updating the timing for the next step in the walking sequence
            updateNextStepTime();//'walking_tick_'이 함수에서 update
      }
      else
      {
            desired_q_ = current_q_;
      }
    }
    else
    {
        desired_q_ = current_q_;
    }
}

void CustomController::circling_motion()
{ //'pelv_traj_float_': representing the trajectory of the pelvis
  pelv_traj_float_.translation().setZero();
  pelv_traj_float_.linear().setIdentity();

  //x-z plane에서 circle 그린다고 생각
  rfoot_traj_float_.translation()(0) = -0.025 * cos(0.5*M_PI*walking_tick_/hz_);
  rfoot_traj_float_.translation()(1) = -0.1235;//y 방향으로 발끝이 안 움직이게 lock
  rfoot_traj_float_.translation()(2) = -0.401123114 + 0.025 * sin(0.5*M_PI*walking_tick_/hz_);

  lfoot_traj_float_.translation()(0) = -0.025 * cos(0.5*M_PI*(walking_tick_/hz_ - 1.0));
  lfoot_traj_float_.translation()(1) =  0.1235;
  lfoot_traj_float_.translation()(2) = -0.401123114 + 0.025 * sin(0.5*M_PI*(walking_tick_/hz_ - 1.0));

  lfoot_traj_float_.linear().setIdentity();
  rfoot_traj_float_.linear().setIdentity();
}

void CustomController::updateControlMask(unsigned int *mask)
{
    if(walking_enable_)//walking mode activated.
    {
        for (int i=0; i<total_dof_; i++)
        {
            //PRIORITY = 8 = 00001000 [in binary]
            // if mask[i] is '00000001', then '(mask[i] | PRIORITY)' results in 'mask[i]' being '00001001' 
            mask[i] = (mask[i] | PRIORITY);//sets the 'PRIORITY' bit in each element of the mask array
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

void CustomController::writeDesired(const unsigned int *mask, VectorQd& desired_q)
{
    for(unsigned int i=0; i<total_dof_; i++)
    {     
        if( mask[i] >= PRIORITY && mask[i] < PRIORITY * 2 )
        {
            desired_q(i) = desired_q_(i);
        }         
    }
}

void CustomController::updateInitialState()
{//'walking_tick_': counter or time-step variable used to track the progression of the walking cycle. Increments at each time step or control loop iteration to indicate how far along the robot is in its current walking motion.--> walking cycle의 current phase(start, middle, end)를 알려줘
    if(walking_tick_ == 0)//beginning of the working process OR a new walking sequence
    {
        calculateFootStepTotal();// ex: 앞으로 5m 전진하기 위해서는 'total footstep'이 얼마나 필요한지 check

        com_float_init_ = model_.getCurrentCom(); // initial COM is obtained from the robot model// 3차원 벡터

        pelv_float_init_.setIdentity(); //initial pelvis transormation = set to identity matrix. Meaning pelvis is at the origin with no initial orientation --> pelvis 기준으로 다른 부분들의 좌표를 표현하려고

        //getting the current transformation matrices of the left and right feet from the robot model// 'world -> endeffector' 로 transform하는 matrix  
        lfoot_float_init_ = model_.getCurrentTransform((DyrosBoltModel::EndEffector)(0));
        rfoot_float_init_ = model_.getCurrentTransform((DyrosBoltModel::EndEffector)(1)); 

        Eigen::Isometry3d ref_frame;// right foot이 바닥에 닿는지, left foot이 바닥에 닿는지에 따라 달라져. 

        //foot_step_() 안의 첫 번째 요소의 의미, 현재의 step(지금 첫 step이니까 0)
        if(foot_step_(0, 6) == 0)  //right foot support - 바닥에 첨 닿는게 오른발
        { ref_frame = rfoot_float_init_; }
        else if(foot_step_(0, 6) == 1) //left foot support
        { ref_frame = lfoot_float_init_; }
        //ref_frame을 바닥에 닿는 애의 좌표계로 하겠다.


        //ref frame에서 본~
        lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);//Tranform Matrix of "ref_frame->lfoot_float_init"
        rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);//Tranform Matrix of "ref_frame->rfoot_float_init"

        pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame);//Sets the initial transformation of the pelvis relative to the support foot's frame
        //Transformation Matrix of "ref_frame->world frame"
        
        com_support_init_ = pelv_support_init_.linear()*com_float_init_ + pelv_support_init_.translation();
        //[com_x, com_y, com_z, 1] = T(ref->world) * [com_x, com_y, com_z, 1]^T
        //좌항의 com vector는 ref_frame 기준 / 우항의 com vector는 world frame 기준


        //converting rotation matrices to Euler angles.
        pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear()); 
        rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear()); 
        lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

        supportfoot_float_init_.setZero();//6D vector
        swingfoot_float_init_.setZero();//6D vector

        if(foot_step_(0,6) == 1) //left suppport foot
        {
        //Support foot initialization
        for(int i=0; i<2; i++)
            supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);//발이 완전히 바닥에 닿아서 --> z축의 병진운동은 고려x
        for(int i=0; i<3; i++)
            supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

        for(int i=0; i<2; i++)
            swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
        for(int i=0; i<3; i++)
            swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

        //below means that it ensures the robot starts from the correct position
        supportfoot_float_init_(0) = 0.0;
        swingfoot_float_init_(0) = 0.0;
        }

        else //right support foot
        {
        for(int i=0; i<2; i++)
            supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
        for(int i=0; i<3; i++)
            supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

        for(int i=0; i<2; i++)
            swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
        for(int i=0; i<3; i++)
            swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

        supportfoot_float_init_(0) = 0.0;
        swingfoot_float_init_(0) = 0.0;
        }


        pelv_support_start_ = pelv_support_init_;// To reference this starting pose throughout the walking cycle.
        total_step_num_ = foot_step_.col(1).size(); //storing the total number of steps --> important because knowing the total number of steps allows the robot to plan its trajectory accurately.
    }


    else if(current_step_num_ != 0 && walking_tick_ == t_start_) // step change 
    {  //this block is executed during a step change in the robot's walking cycle
        Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF+1, 1> q_temp;
        Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF, 1>  qdot_temp, qddot_temp;
        q_temp.setZero();
        qdot_temp.setZero();
        qddot_temp.setZero();

        //'.segment<8>(6)': 8개의 요소, starting index:6 
        q_temp.segment<8>(6) = current_q_.segment<8>(0);//q_temp vetor의 '7번째 요소~14번째요소'를 current_q_vector의 8개 값으로 둔다.
        qdot_temp.segment<8>(6)= current_qdot_.segment<8>(0);  
        // q_temp.segment<8>(6) = desired_q_not_compensated_.segment<8>(0);
        
        model_.updateKinematics(q_temp, qdot_temp, qddot_temp);

        //위에랑 SAME
        lfoot_float_init_ = model_.getCurrentTransform((DyrosBoltModel::EndEffector)(0));
        rfoot_float_init_ = model_.getCurrentTransform((DyrosBoltModel::EndEffector)(1));
        com_float_init_ = model_.getCurrentCom();
        pelv_float_init_.setIdentity();

        Eigen::Isometry3d ref_frame;

        if(foot_step_(current_step_num_, 6) == 0)  //right foot support
        { ref_frame = rfoot_float_init_; }
        else if(foot_step_(current_step_num_, 6) == 1)
        { ref_frame = lfoot_float_init_; }

        pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame);
        com_support_init_ = pelv_support_init_.linear()*com_float_init_ + pelv_support_init_.translation(); 
        pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear()); 

        lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
        rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);    
        rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear()); 
    }

}

void CustomController::getRobotState()
{
    Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF, 1> q_temp;
    Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF, 1>  qdot_temp, qddot_temp;
    q_temp.setZero();
    qdot_temp.setZero(); 
    qddot_temp.setZero(); 

    q_temp.segment<8>(6) = current_q_.segment<8>(0);//q_temp에서 뒤의 8개 요소들은, 6개의 조인트와 2개의 핀조인트에 해당   
    qdot_temp.segment<8>(6)= current_qdot_.segment<8>(0);

    model_.updateKinematics(q_temp, qdot_temp, qddot_temp);
    com_float_current_ = model_.getCurrentCom();
    com_float_current_dot_= model_.getCurrentComDot();
    //Transformation matrix of 'worldframe->footframe'
    lfoot_float_current_ = model_.getCurrentTransform((DyrosBoltModel::EndEffector)0); 
    rfoot_float_current_ = model_.getCurrentTransform((DyrosBoltModel::EndEffector)1);

    //support foot에 따라 ref_frame이 바뀌는 거 생각
    if(foot_step_(current_step_num_, 6) == 0) 
    { supportfoot_float_current_ = rfoot_float_current_; }
    else if(foot_step_(current_step_num_, 6) == 1)
    { supportfoot_float_current_ = lfoot_float_current_; }

    //transformation matrix of 'supportfootframe -> otherframe' 
    pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_);
    pelv_float_current_.setIdentity();
    lfoot_support_current_ = DyrosMath::multiplyIsometry3d(pelv_support_current_,lfoot_float_current_);
    rfoot_support_current_ = DyrosMath::multiplyIsometry3d(pelv_support_current_,rfoot_float_current_);     
    com_support_current_ =  DyrosMath::multiplyIsometry3dVector3d(pelv_support_current_, com_float_current_);

}

void CustomController::calculateFootStepTotal()
{
  double initial_rot = 0.0;
  double final_rot = 0.0;
  double initial_drot = 0.0;
  double final_drot = 0.0;

  //타겟 포지션으로 로봇 머리 방향 돌리기
  initial_rot = atan2(target_y_, target_x_);

  //drot: 한 스텝 당 돌리는 양
  if(initial_rot > 0.0)
    initial_drot = 10*DEG2RAD;
  else
    initial_drot = -10*DEG2RAD;

  //돌리는 속도에 필요한 스텝수 계산(한 스텝당 10씩)
  unsigned int initial_total_step_number = initial_rot/initial_drot;
  //10으로 나누고 남은 각도 ex)65 = 6(init_tot_step_num) * 10(init_rot) + 5(init_residual)
  double initial_residual_angle = initial_rot - initial_total_step_number*initial_drot;

  //도착한 후에 로봇을 돌리는 각도와 속도(한 스텝 당 돌리는 양) 설정
  final_rot = target_theta_ - initial_rot; 
  if(final_rot > 0.0)
    final_drot = 10*DEG2RAD;
  else
    final_drot = -10*DEG2RAD;

  //돌리는 속도에 필요한 스텝 수 계산(한 스텝당 10씩)
  unsigned int final_total_step_number = final_rot/final_drot;
  //마지막에 돌리는 과정에서 남는 각도 ex)25 = 2(fin_tot_step_num) * 10(fin_rot) + 5(fin_residual) 
  double final_residual_angle = final_rot - final_total_step_number*final_drot;
  double length_to_target = sqrt(target_x_*target_x_ + target_y_*target_y_);
  double dlength = step_length_x_;
  unsigned int middle_total_step_number = length_to_target/dlength;
  //로봇이 일정 보폭으로 직선을 걷는 과정에서 남는 거리
  double middle_residual_length = length_to_target - middle_total_step_number*dlength;

  if(length_to_target == 0)
  {
    middle_total_step_number = 5;
    dlength = 0;
  }

  unsigned int number_of_foot_step;

  int del_size;

  del_size = 1;
  //로봇이 총 움직이는 스텝 수 계산
  number_of_foot_step = initial_total_step_number*del_size + middle_total_step_number*del_size + final_total_step_number*del_size;

  //처음 목표 방향으로 돌 때 필요한 추가 걸음 수
  //만약 회전용 총 걸음 수가 0이 아니거나 걸음 당 10도로 남은 각도가 0.0001도 이상이면 발동
  if(initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
  {
    //만약 처음 회전 총 스텝 수가 2로 나눠지면 두 스텝 추가
    if(initial_total_step_number % 2 == 0)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
    {
      //2로 나눠지지 않으면 만약 남은 각도가 0.001도 이상일 때 세 스텝 추가
      if(abs(initial_residual_angle)>= 0.0001)
        number_of_foot_step = number_of_foot_step + 3*del_size;
      //그 외에 한 스텝 추가
      else
        number_of_foot_step = number_of_foot_step + del_size;
    }
  }


  //처음과 끝지점을 걸어갈 때 필요한 추가 걸음 수
  if(middle_total_step_number != 0 || abs(middle_residual_length)>=0.0001)
  {
    if(middle_total_step_number % 2 == 0)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
    {
      if(abs(middle_residual_length)>= 0.0001)
        number_of_foot_step = number_of_foot_step + 3*del_size;
      else
        number_of_foot_step = number_of_foot_step + del_size;
    }
  }

  //마지막 목표 방향으로 돌 때 필요한 추가 걸음 수
  if(final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
  {
    if(abs(final_residual_angle) >= 0.0001)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
      number_of_foot_step = number_of_foot_step + del_size;
  }

  //로봇의 foot step(왼발 오른발 통합)을 스텝수 x 7 매트릭스로 사이즈 변환
  // 0, 1, 2: foot step의 x,y,z 좌표
  // 3, 4: 좌표계가 x,y축을 기준으로 돌아간 각도 땅과 붙어있기 때문에 0이다.
  // 5: 좌표계가 z축을 기준으로 돌아간 각도
  // 6: 움직여야 하는 발 지정( 0이면 오른발이 움직이고 왼발 고정 | 1이면 왼발이 움직이고 오른발 고정)
  foot_step_.resize(number_of_foot_step, 7);
  foot_step_.setZero();
  //foot_step_support_frame_도 같은 크기로 사이즈 변환
  foot_step_support_frame_.resize(number_of_foot_step, 7);
  foot_step_support_frame_.setZero();

  int index = 0;
  int temp, temp2, temp3, is_right;

  if(is_right_foot_swing_ == true)
    is_right = 1;
  else
    is_right = -1;

  //오른발이 먼저 스윙하게 세팅
  temp = -is_right;
  temp2 = -is_right;
  temp3 = -is_right;


  if(initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
  {
    for (int i =0 ; i < initial_total_step_number; i++)
    {
      temp *= -1;
      foot_step_(index,0) = temp*0.1235*sin((i+1)*initial_drot);
      foot_step_(index,1) = -temp*0.1235*cos((i+1)*initial_drot);
      foot_step_(index,5) = (i+1)*initial_drot;
      //[0이면 right foot support(오른발이 스윙), 1이면 left foot support(왼발이 스윙)] 1이기 때문에 왼발이 스윙
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index++;
    }

    if(temp == is_right) //temp가 1이면 왼발이 이제 움직일 차례
    {
      if(abs(initial_residual_angle) >= 0.0001)
      {
        temp *= -1;

        foot_step_(index,0) = temp*0.1235*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.1235*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*0.1235*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.1235*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*0.1235*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.1235*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

      }
      else
      {
        temp *= -1;

        foot_step_(index,0) = temp*0.1235*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.1235*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;
      }
    }
    else if(temp == -is_right) //temp가 0이면 오른발이 이제 움직일 차례
    {
      temp *= -1;

      foot_step_(index,0) = temp*0.1235*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,1) = -temp*0.1235*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index ++;

      temp *= -1;

      foot_step_(index,0) = temp*0.1235*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,1) = -temp*0.1235*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index ++;
    }
  }

  if(middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001)
  {
    for (int i = 0 ; i < middle_total_step_number; i++)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(i+1)) + temp2*sin(initial_rot)*(0.1235);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(i+1)) - temp2*cos(initial_rot)*(0.1235);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index ++;
    }

    if(temp2 == is_right)
    {
      if(abs(middle_residual_length) >= 0.0001)
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1235);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1235);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;

        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1235);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1235);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1235);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1235);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;
      }
      else
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1235);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1235);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;
      }
    }
    else if(temp2 == -is_right)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1235);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1235);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index++;

      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1235);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1235);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index++;
    }
  }

  double final_position_x = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length);
  double final_position_y = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length);

  if(final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
  {
    for(int i = 0 ; i < final_total_step_number; i++)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.1235*sin((i+1)*final_drot + initial_rot);
      foot_step_(index,1) = final_position_y - temp3*0.1235*cos((i+1)*final_drot + initial_rot);
      foot_step_(index,5) = (i+1)*final_drot + initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }

    if(abs(final_residual_angle) >= 0.0001)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.1235*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*0.1235*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;

      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.1235*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*0.1235*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }
    else
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.1235*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*0.1235*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }
  }
}

void CustomController::floatToSupportFootstep()
{
  Eigen::Isometry3d reference;

  if(current_step_num_ == 0) //시작할 때
  {
    if(foot_step_(0,6) == 0) //오른발이 support
    {
      //오른발이 reference 매트릭스로 됨(x,z좌표를 0으로 설정), rotation은 z방향 각도를 측정 후 z방향으로 얼만큼 돌았는 지로 지정
      reference.translation() = rfoot_float_init_.translation();
      reference.translation()(2) = 0.0;
      reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rfoot_float_init_.linear())(2));
      reference.translation()(0) = 0.0;
    }
    else //반대면 왼발이 support
    {
      reference.translation() = lfoot_float_init_.translation();
      reference.translation()(2) = 0.0;
      reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(lfoot_float_init_.linear())(2));
      reference.translation()(0) = 0.0;
    }
  }

  else //시작 스텝이 아닐 때
  {

    //reference의 rotation 매트릭스는  foot step yaw에 의해 만들어짐
    reference.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_-1,5));
    //reference의 position 매트릭스는 foot step의 x,y,z로 이루어짐
    for(int i=0 ;i<3; i++)
      reference.translation()(i) = foot_step_(current_step_num_-1,i);
  }

  Eigen::Vector3d temp_local_position;
  Eigen::Vector3d temp_global_position;

  for(int i = 0; i < total_step_num_; i++)
  { 
    //발 자국의 다음 스텝 global position 받아오기
    for(int j = 0; j < 3; j ++)
      temp_global_position(j) = foot_step_(i,j);

    //글로벌 포지션에서 reference의 포지션 벡터와 로테이션 매트릭스를 통해 foot_step들의 상대좌표 구하기
    temp_local_position = reference.linear().transpose()*(temp_global_position - reference.translation());

    //foot_step의 support frame을 기준으로의 좌표 (x,y,z,roll,pitch,yaw)
    for(int j=0; j<3; j++)
      foot_step_support_frame_(i,j) = temp_local_position(j);

    foot_step_support_frame_(i,3) = foot_step_(i,3);
    foot_step_support_frame_(i,4) = foot_step_(i,4);

    if(current_step_num_ == 0)
    { foot_step_support_frame_(i,5) = foot_step_(i,5) - supportfoot_float_init_(5); }
    else
    { foot_step_support_frame_(i,5) = foot_step_(i,5) - foot_step_(current_step_num_-1,5);}
  }

  //temp global postiion에 supportfoot의 float_init_좌표로 재설정
  for(int j=0;j<3;j++)
    temp_global_position(j) = supportfoot_float_init_(j);

  //temp local postion을 reference좌표계에 대한 좌표로 수정
  temp_local_position = reference.linear().transpose()*(temp_global_position - reference.translation());

  //support_foot의 supprot하는 발 좌표(x,y,z)는 temp local position에서 받아온다
  //r,p,y는 support_float_init과 foot_step으로부터 받아온다.
  for(int j=0;j<3;j++)
    supportfoot_support_init_(j) = temp_local_position(j);

  supportfoot_support_init_(3) = supportfoot_float_init_(3);
  supportfoot_support_init_(4) = supportfoot_float_init_(4);

  if(current_step_num_ == 0)
    supportfoot_support_init_(5) = 0;
  else
    supportfoot_support_init_(5) = supportfoot_float_init_(5) - foot_step_(current_step_num_-1,5);

}

void CustomController::getComTrajectory()
{
  unsigned int planning_step_number = 3;
  unsigned int norm_size = 0;
  
  if(current_step_num_ >= total_step_num_ - planning_step_number)
    norm_size = (t_last_ - t_start_ + 1)*(total_step_num_ - current_step_num_) + 20*hz_;
  else
    norm_size = (t_last_ - t_start_ + 1)*(planning_step_number); 
  if(current_step_num_ == 0)
    norm_size = norm_size + t_temp_ + 1;
 
  comGenerator(norm_size, planning_step_number);

  if(current_step_num_ == 0)
  {
    com_desired_(0) = ref_com_(walking_tick_,0);
    com_desired_(1) = ref_com_(walking_tick_,1);
  }
  else
  {
    com_desired_(0) = ref_com_(walking_tick_ - t_start_, 0);
    com_desired_(1) = ref_com_(walking_tick_ - t_start_, 1);
  }

  com_desired_(2) = pelv_support_start_.translation()(2); 

}

void CustomController::comGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{ 
  ref_com_.resize(norm_size, 2); 
  Eigen::VectorXd temp_cx;
  Eigen::VectorXd temp_cy;
  
  unsigned int index = 0;

  if(current_step_num_ == 0)  
  {
    for (int i = 0; i <= t_temp_; i++) 
    { 
      ref_com_(i,0) = com_support_init_(0); //DyrosMath::cubic(i, 0, 2*hz_,com_support_init_(0), 0, 0, 0);
      ref_com_(i,1) = com_support_init_(1);
      if(i >= 2*hz_)
      { 
        Eigen::Vector3d ref_y_com ;
        ref_y_com = (DyrosMath::QuinticSpline(i, 2*hz_, 3*hz_,com_support_init_(1), 0, 0, com_support_init_(1), 0.345914/hz_,0));
        ref_com_(i,1) = ref_y_com(0);
      }
      index++;
    }    
  }
  if(current_step_num_ >= total_step_num_ - planning_step_num)
  {  
    for(unsigned int i = current_step_num_; i < total_step_num_; i++)
    {
      onestepCom(i, temp_cx, temp_cy);
     
      for(unsigned int j = 0; j < t_total_; j++)
      {
        ref_com_(index + j, 0) = temp_cx(j);
        ref_com_(index + j, 1) = temp_cy(j);    
      }
      index = index + t_total_;
    }
    
    for(unsigned int j = 0; j < 20*hz_; j++)
    {
      ref_com_(index + j, 0) = ref_com_(index -1, 0);
      ref_com_(index + j, 1) = ref_com_(index -1, 1);
    }

    if((current_step_num_ == total_step_num_ - 1)) 
    { Eigen::Vector3d ref_y_com ;
            
      for(int i = 0; i < 240; i++)
      {
        ref_y_com = DyrosMath::QuinticSpline(i+239, 120, 479, 0.015299, -1.7347e-18, 9.3508e-06, 0.1235, 0, 0);
        ref_com_(index + i, 1) = ref_y_com(0) ;
      }
    }

    index = index + 20*hz_;      
  }
  else 
  { 
    for(unsigned int i = current_step_num_; i < current_step_num_ + planning_step_num; i++)  
    {
      onestepCom(i, temp_cx, temp_cy);
      for (unsigned int j = 0; j < t_total_; j++) 
      {
        ref_com_(index+j,0) = temp_cx(j);
        ref_com_(index+j,1) = temp_cy(j);
      }      
      index = index + t_total_; 
    }   
  }   
}

void CustomController::onestepCom(unsigned int current_step_number, Eigen::VectorXd& temp_cx, Eigen::VectorXd& temp_cy)
{
  temp_cx.resize(t_total_);  
  temp_cy.resize(t_total_);
  temp_cx.setZero();
  temp_cy.setZero();

  double A = 0, B = 0, Cx1 = 0, Cx2 = 0, Cy1 = 0, Cy2 = 0, Kx = 0, Ky = 0, wn = 0 ;
  if(current_step_number == 0)
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = -(foot_step_support_frame_(current_step_number, 1) )/2 ;
    B =  (-supportfoot_support_init_(0) + foot_step_support_frame_(current_step_number, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*(0.45)));
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45)); 
    Cx1 = Kx - B;
    Cx2 = Kx/(wn*0.15);
    Cy1 = Ky - A;
    Cy2 = Ky/(wn*0.15);
        
    for(int i = 0; i < t_total_; i++)
    {
      temp_cx(i) = DyrosMath::cubic(i, 0, t_total_-1,0 ,0.05 , 0, Kx/(t_rest_init_ + t_double1_));
      if(i >= 0 && i < (t_rest_init_ + t_double1_))
      {  
        temp_cy(i) = com_support_init_(1) + Ky / (t_rest_init_ + t_double1_) * (i+1);
      }
      else if(i >= (t_rest_init_ + t_double1_) && i < t_total_ - t_rest_last_ - t_double2_ )
      {
        temp_cy(i) = A + com_support_init_(1) + Cy1 *cosh(wn*(i/hz_ - 0.15)) + Cy2*sinh(wn*(i/hz_-0.15)) ;
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_  && i < t_total_)  
      {
        temp_cy(i) = Ky + (supportfoot_support_init_(1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    }    
  }
  else if(current_step_number == 1)
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = (foot_step_support_frame_(current_step_number-1, 1) - supportfoot_support_init_(1))/2 ;
    B = foot_step_support_frame_(current_step_number-1, 0) - (supportfoot_support_init_(0) + foot_step_support_frame_(current_step_number-1, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*0.45));
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45)); 
    Cx1 = Kx - B;
    Cx2 = Kx/(wn*0.15);
    Cy1 = Ky - A;
    Cy2 = Ky/(wn*0.15);    
    for(int i = 0; i < t_total_; i++)
    {
      if(i >= 0 && i < (t_rest_init_ + t_double1_))
      { 
        temp_cx(i) = (foot_step_support_frame_(current_step_number-1, 0) + supportfoot_support_init_(0))/2 + Kx / (t_rest_init_+ t_double1_) * (i+1);
        temp_cy(i) = (foot_step_support_frame_(current_step_number-1, 1) + supportfoot_support_init_(1))/2 + Ky / (t_rest_init_+ t_double1_) * (i+1);
      }
      else if(i >= (t_rest_init_ + t_double1_) && i < (t_total_ - t_rest_last_ - t_double2_))  
      {
        temp_cx(i) = (supportfoot_support_init_(0) + foot_step_support_frame_(current_step_number-1, 0))/2 + Cx1 *cosh(wn*(i/hz_ - 0.15)) + Cx2*sinh(wn*(i/hz_-0.15)) + B;
        temp_cy(i) = A + (supportfoot_support_init_(1) + foot_step_support_frame_(current_step_number-1, 1))/2 + Cy1 *cosh(wn*(i/hz_ - 0.15)) + Cy2*sinh(wn*(i/hz_-0.15)) ;
      }
      else if(i >= (t_total_ - t_rest_last_ - t_double2_) && i < t_total_)  
      { 
        temp_cx(i) = (foot_step_support_frame_(current_step_number, 0)+ foot_step_support_frame_(current_step_number-1, 0)) /2 -Kx + Kx/(t_rest_last_ + t_double2_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_cy(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    //   com_x_data += std::to_string(temp_cx(i)) + "\n";
    //   com_y_data += std::to_string(temp_cy(i)) + "\n";
    }
  }
  else
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = (foot_step_support_frame_(current_step_number-1, 1) - foot_step_support_frame_(current_step_number-2, 1))/2 ;
    B = foot_step_support_frame_(current_step_number-1, 0) - (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*0.45));
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45)); 
    Cx1 = Kx - B;
    Cx2 = Kx/(wn*0.15);
    Cy1 = Ky - A;
    Cy2 = Ky/(wn*0.15);
    for(int i = 0; i < t_total_; i++)
    {
      if(i >= 0 && i < (t_rest_init_ + t_double1_))
      {
        temp_cx(i) = (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 + Kx/(t_rest_init_ + t_double1_)*(i);
        temp_cy(i) = (foot_step_support_frame_(current_step_number-2, 1) + foot_step_support_frame_(current_step_number-1, 1))/2 + Ky/(t_rest_init_ + t_double1_)*(i);
      }            
      else if(i >= (t_rest_init_ + t_double1_) && i < (t_total_ - t_rest_last_ - t_double2_)) 
      {
        temp_cx(i) = (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 + Cx1 *cosh(wn*(i/hz_ - 0.15)) + Cx2*sinh(wn*(i/hz_-0.15)) + B;
        temp_cy(i) = A + (foot_step_support_frame_(current_step_number-2, 1) + foot_step_support_frame_(current_step_number-1, 1))/2 + Cy1 *cosh(wn*(i/hz_ - 0.15)) + Cy2*sinh(wn*(i/hz_-0.15)) ;
         
      }
      else if(i >= (t_total_ - t_rest_last_ - t_double2_) && i < t_total_)  
      {
        temp_cx(i) = (foot_step_support_frame_(current_step_number, 0)+ foot_step_support_frame_(current_step_number-1, 0)) /2 -Kx + Kx/(t_rest_last_ + t_double2_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_cy(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 - Ky/(t_rest_last_ + t_double2_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
      
      if(i >= (t_rest_init_ + t_double1_) && (current_step_num_ == total_step_num_ - 1) && i < t_total_ ) 
      { 
        Eigen::Vector3d ref_x_com ;
        ref_x_com = DyrosMath::QuinticSpline(i, 30, 239, (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 + Kx, 0.143853/hz_, -1.125e-5, (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 + B, 0, 0);//com_support_init_(1)+com_offset_(1), 0.289384/hz_,0));
        temp_cx(i) = ref_x_com(0);
      }
      if( i >= 120 && i < t_total_ && (current_step_num_ == total_step_num_ - 1))  
      { 
        Eigen::Vector3d ref_y_com ; 
        ref_y_com = DyrosMath::QuinticSpline(i, 120, 479, 0.015299, (Cy1 *sinh(wn*(120/hz_ - 0.15))*wn/hz_ + Cy2*cosh(wn*(120/hz_-0.15))*wn/hz_), (Cy1 *cosh(wn*(120/hz_ - 0.15))*wn/hz_*wn/hz_ + Cy2*sinh(wn*(120/hz_-0.15))*wn/hz_*wn/hz_), 0.1235, 0, 0);
        temp_cy(i) = ref_y_com(0);
      }
    //   com_x_data += std::to_string(temp_cx(i)) + "\n";
    //   com_y_data += std::to_string(temp_cy(i)) + "\n";
    } 
  }
}

void CustomController::getPelvTrajectory()
{
    pelv_traj_support_.setIdentity();
    pelv_traj_support_.translation()(0) = pelv_support_current_.translation()(0) + 1.0*(com_desired_(0) - com_support_current_(0));
    pelv_traj_support_.translation()(1) = pelv_support_current_.translation()(1) + 1.0*(com_desired_(1) - com_support_current_(1));
}   

void CustomController::getFootTrajectory()
{
  Eigen::Vector6d target_swing_foot;

  for(int i=0; i<6; i++)
  { target_swing_foot(i) = foot_step_support_frame_(current_step_num_,i); }
 
  if(walking_tick_ < t_start_ + t_rest_init_ + t_double1_) 
  {  
    if(foot_step_(current_step_num_,6) == 1)  
    { 
      lfoot_traj_support_.translation().setZero();
      lfoot_traj_euler_support_.setZero();

      rfoot_traj_support_.translation() = rfoot_support_init_.translation();
      rfoot_traj_support_.translation()(2) = 0;
      rfoot_traj_euler_support_ = rfoot_support_euler_init_;
    }     
    else if(foot_step_(current_step_num_,6) == 0)  
    { 
      rfoot_traj_support_.translation().setZero();
      rfoot_traj_euler_support_.setZero();

      lfoot_traj_support_.translation() = lfoot_support_init_.translation();
      lfoot_traj_support_.translation()(2) = 0;
      lfoot_traj_euler_support_ = lfoot_support_euler_init_; 
    }     

    lfoot_traj_support_.linear() = DyrosMath::rotateWithZ(lfoot_traj_euler_support_(2))*DyrosMath::rotateWithY(lfoot_traj_euler_support_(1))*DyrosMath::rotateWithX(lfoot_traj_euler_support_(0));
    rfoot_traj_support_.linear() = DyrosMath::rotateWithZ(rfoot_traj_euler_support_(2))*DyrosMath::rotateWithY(rfoot_traj_euler_support_(1))*DyrosMath::rotateWithX(rfoot_traj_euler_support_(0));
  }
  
  else if(walking_tick_ >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_ < t_start_ + t_total_ - t_double2_ - t_rest_last_)  
  {   
    if(foot_step_(current_step_num_,6) == 1) 
    {
      lfoot_traj_support_.translation() = lfoot_support_init_.translation();             
      lfoot_traj_euler_support_.setZero(); 

      lfoot_traj_support_.linear() = DyrosMath::rotateWithZ(lfoot_traj_euler_support_(2))*DyrosMath::rotateWithY(lfoot_traj_euler_support_(1))*DyrosMath::rotateWithX(lfoot_traj_euler_support_(0));
      
      if(walking_tick_ < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0) 
      { rfoot_traj_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_+ t_rest_init_ + t_double1_,t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0,0,foot_height_,0.0,0.0); }  
      else
      { rfoot_traj_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,target_swing_foot(2),0.0,0.0); }
      
      for(int i=0; i<2; i++)  
      { rfoot_traj_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_ + t_double1_ , t_start_+t_total_-t_rest_last_-t_double2_, rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0); } 
      
      rfoot_traj_euler_support_(0) = 0;
      rfoot_traj_euler_support_(1) = 0;
      rfoot_traj_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_ + t_rest_init_ + t_double1_,t_start_ + t_total_ - t_rest_last_ - t_double2_,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      rfoot_traj_support_.linear() = DyrosMath::rotateWithZ(rfoot_traj_euler_support_(2))*DyrosMath::rotateWithY(rfoot_traj_euler_support_(1))*DyrosMath::rotateWithX(rfoot_traj_euler_support_(0));
    }
    else if(foot_step_(current_step_num_,6) == 0) 
    { 
      rfoot_traj_support_.translation() = rfoot_support_init_.translation(); 
      rfoot_traj_euler_support_.setZero(); 

      rfoot_traj_support_.linear() = DyrosMath::rotateWithZ(rfoot_traj_euler_support_(2))*DyrosMath::rotateWithY(rfoot_traj_euler_support_(1))*DyrosMath::rotateWithX(rfoot_traj_euler_support_(0));
 
      if(walking_tick_ < t_start_ + t_rest_init_ + t_double1_ + (t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0)
      { lfoot_traj_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,0,foot_height_,0.0,0.0); }
      else
      { lfoot_traj_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,target_swing_foot(2),0.0,0.0); }
         
      for(int i=0; i<2; i++)
      { lfoot_traj_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0); }

      lfoot_traj_euler_support_(0) = 0;
      lfoot_traj_euler_support_(1) = 0;  
      lfoot_traj_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      lfoot_traj_support_.linear() = DyrosMath::rotateWithZ(lfoot_traj_euler_support_(2))*DyrosMath::rotateWithY(lfoot_traj_euler_support_(1))*DyrosMath::rotateWithX(lfoot_traj_euler_support_(0));
    } 
  }
  else 
  { 
    if(foot_step_(current_step_num_,6) == 1) 
    {
      lfoot_traj_euler_support_.setZero();
      lfoot_traj_support_.linear() = DyrosMath::rotateWithZ(lfoot_traj_euler_support_(2))*DyrosMath::rotateWithY(lfoot_traj_euler_support_(1))*DyrosMath::rotateWithX(lfoot_traj_euler_support_(0));
      
      for(int i=0; i<3; i++)
      {
        rfoot_traj_support_.translation()(i) = target_swing_foot(i);
        rfoot_traj_euler_support_(i) = target_swing_foot(i+3);
      }
      rfoot_traj_support_.linear() = DyrosMath::rotateWithZ(rfoot_traj_euler_support_(2))*DyrosMath::rotateWithY(rfoot_traj_euler_support_(1))*DyrosMath::rotateWithX(rfoot_traj_euler_support_(0));
    }
    else if (foot_step_(current_step_num_,6) == 0) 
    {
      rfoot_traj_euler_support_.setZero();
      rfoot_traj_support_.linear() = DyrosMath::rotateWithZ(rfoot_traj_euler_support_(2))*DyrosMath::rotateWithY(rfoot_traj_euler_support_(1))*DyrosMath::rotateWithX(rfoot_traj_euler_support_(0));

      for(int i=0; i<3; i++)
      {
        lfoot_traj_support_.translation()(i) = target_swing_foot(i);
        lfoot_traj_euler_support_(i) = target_swing_foot(i+3);
      }
      lfoot_traj_support_.linear() = DyrosMath::rotateWithZ(lfoot_traj_euler_support_(2))*DyrosMath::rotateWithY(lfoot_traj_euler_support_(1))*DyrosMath::rotateWithX(lfoot_traj_euler_support_(0));
    }
  }
}


void CustomController::supportToFloat()
{
    pelv_traj_float_ = DyrosMath::inverseIsometry3d(pelv_traj_support_)*pelv_traj_support_;  
    lfoot_traj_float_ = DyrosMath::inverseIsometry3d(pelv_traj_support_)*lfoot_traj_support_;
    rfoot_traj_float_ = DyrosMath::inverseIsometry3d(pelv_traj_support_)*rfoot_traj_support_;
}


// void CustomController::computeIK(Eigen::Isometry3d float_pelv_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector8d& q_des)
// {
//     Eigen::Vector3d L_r, R_r;
//     Eigen::Matrix3d L_Hip_rot, R_Hip_rot;

//     Eigen::Vector3d L_1, L_2, L_3, R_1, R_2, R_3;

//     L_1 << 0, +0.0145, -0.0386;
//     R_1 << 0, -0.0145, -0.0386;

//     L_2 << 0, +0.0374, -0.2;
//     R_2 << 0, -0.0374, -0.2;
    
//     L_3 << 0, +0.008, -0.2;
//     R_3 << 0, -0.008, -0.2;

//     L_Hip_rot = float_pelv_transform.rotation().transpose() * float_lleg_transform.rotation();
//     R_Hip_rot = float_pelv_transform.rotation().transpose() * float_rleg_transform.rotation();

//     L_r = float_lleg_transform.translation() - float_pelv_transform.translation();
//     R_r = float_rleg_transform.translation() - float_pelv_transform.translation();


//     q_des(0) = atan2(L_Hip_rot(2,1), L_Hip_rot(1,1));
//     q_des(1) = asin((L_r(0) + L_3(2)*L_Hip_rot(0,0)) / L_2(2));
//     q_des(2) = atan2(L_Hip_rot(0,0), L_Hip_rot(0,2)) - q_des(1);

//     q_des(4) = atan2(R_Hip_rot(2,1), R_Hip_rot(1,1));
//     q_des(5) = asin((R_r(0) + R_3(2)*R_Hip_rot(0,0)) / R_2(2));
//     q_des(6) = atan2(R_Hip_rot(0,0), R_Hip_rot(0,2)) - q_des(4);

//     q_des(3) = 0;
//     q_des(7) = 0;
// }  

void CustomController::computeIK(Eigen::Isometry3d float_pelv_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector8d& q_des)
{
    Eigen::Vector3d L_r, R_r; //vectors from ankle to hip
    Eigen::Vector3d L_hip, R_hip; //Positions of the left and right hips

    Eigen::Matrix3d R_knee_ankle_Y_rot, L_knee_ankle_Y_rot; //Rotation matrices for knee and ankle joints.
    Eigen::Matrix3d pelv_lhipx_rot, pelv_rhipx_rot; //Rotation matrices for hip joints.
    Eigen::Matrix3d L_hip_rot, R_hip_rot; //Rotation matrices for the hips.

    Eigen::Vector3d L_1, L_2, L_3, L_4, R_1, R_2, R_3, R_4; //Link vectors for the left and right legs.
    Eigen::Vector3d L_Y_offset, R_Y_offset;//Y offsets for the left and right legs

    //L_upper and L_lower are the length of z component of the link 3 and 4
    double L_C = 0, R_C = 0, L_alpha = 0, R_alpha = 0, L_upper = 0.2 , L_lower = 0.2;//'L_C', 'R_C': Lengths of the vectors from ankle to hip. //'L_alpha', 'R_alpha': Knee angles


    //pelvis to hip_x
    L_1 << 0, +0.0636, 0;
    R_1 << 0, -0.0636, 0;

    //hip_x to hip_y
    L_2 << 0, +0.0145, -0.0386;
    R_2 << 0, -0.0145, -0.0386;

    //hip_y to knee_y
    L_3 << 0, +0.0374, -0.2;
    R_3 << 0, -0.0374, -0.2;
    
    //knee_y to ankle_y
    L_4 << 0, +0.008, -0.2;
    R_4 << 0, -0.008, -0.2;

    L_Y_offset << 0, L_3(1) + L_4(1), 0;
    R_Y_offset << 0, R_3(1) + R_4(1), 0;

    //get the rotation matrix of the hip_x joint
    pelv_lhipx_rot = DyrosMath::rotateWithX(q_des(0));//Rotation matrix for the left hip around the x-axis.
    pelv_rhipx_rot = DyrosMath::rotateWithX(q_des(4));//Rotation matrix for the right hip around the x-axis.

    //Caculate the Vetor from  the float to the hip_y
    L_hip = float_pelv_transform.translation() + float_pelv_transform.rotation() * (L_1 + pelv_lhipx_rot * L_2);// Position of the left hip in the floating base frame.
    R_hip = float_pelv_transform.translation() + float_pelv_transform.rotation() * (R_1 + pelv_lhipx_rot * R_2); //Position of the right hip in the floating base frame.

    //calculate the relative vector from the ankle to the hip_y
    //used for caculating knee, and anhle angle by using the law of cosine 2 & the law of sine 2
    L_r = float_lleg_transform.rotation().transpose() * (L_hip - float_lleg_transform.translation()) - pelv_lhipx_rot * L_Y_offset;// Vector from the left ankle to the left hip.
    R_r = float_rleg_transform.rotation().transpose() * (R_hip - float_rleg_transform.translation()) - pelv_rhipx_rot * R_Y_offset;// Vector from the right ankle to the right hip.

    //calcualte the length of vector L_r and R_r [l3]
    L_C = sqrt( pow(L_r(0),2) + pow(L_r(1),2) + pow(L_r(2),2) );
    R_C = sqrt( pow(R_r(0),2) + pow(R_r(1),2) + pow(R_r(2),2) );

    //calculate the angle of alpha(angle1) using the law of sine 2
    //l3/sin(angle3) = l1/sin(angle1) [l1=L_up, l3=L_C, angle1 = alpha, angle3 = PI + q_des(2)]
    // double asin_argument;

    L_alpha = asin(L_upper / L_C * sin(M_PI + q_des(2)));//knee angle for the left leg
    R_alpha = asin(L_upper / R_C * sin(M_PI + q_des(6)));//knee angle for the right leg

    //initialize the hip_rot matrix [hip_rot = hipx_rot * hipy_rot]
    L_hip_rot.setZero(); R_hip_rot.setZero();

    //float_ankle_rot(float to ankle) = float_pelv_rot * hipx_rot * hip_y_rot * knee_rot * ankle_rot
    //knee_rot = rot_y(angle of knee[q_des(2)]), ankle_rot = rot_y(angle of ankle[q_des(3)])
    //knee_rot * ankle_rot = rot_y(q_des(2) + q_des(3))
    //transpose matrix of rot (angle) = rot(-angle) -> (knee_rot * ankle_rot)^T = rot_y(-q_des(2) - q_des(3))
    L_knee_ankle_Y_rot = DyrosMath::rotateWithY(-q_des(2)-q_des(3));//Rotation matrix for the left knee and ankle
    R_knee_ankle_Y_rot = DyrosMath::rotateWithY(-q_des(6)-q_des(7));

    L_hip_rot = float_pelv_transform.rotation().transpose() * float_lleg_transform.rotation() * L_knee_ankle_Y_rot;//rotation matrix for the left hip
    R_hip_rot = float_pelv_transform.rotation().transpose() * float_rleg_transform.rotation() * R_knee_ankle_Y_rot;

    //Left hipx, hipy, knee, ankle angle

    //qdes(0)(angle of hip_x) & qdes(1)(angle of hip_y) are calculated by hip_rot matrix [hip_rot = hipx_rot * hipy_rot]
    //            |    c1    0      s1  |   
    // hip_rot =  |  s0*s1   c0  -s0*c1 |   (0 = qdes(0), 1 = q_des(1))
    //            | -c0*s1   s0   c0*c1 |
    //q_des(0) = atan2(R32,R22)
    q_des(0) = atan2(L_hip_rot(2,1), L_hip_rot(1,1));//Angle of the left hip around the x-axis.
    //q_des(1) = atan2(-R31,R33)
    q_des(1) = atan2(-L_hip_rot(2,0), L_hip_rot(2,2));//Angle of the left hip around the y-axis.
    //by the law of cosine 2 -> l3^2 = l1^2 + l2^2 - 2*l1*l2*cos(angle3) [l1=L_up, l2=L_low, l3=L_C, angle3 = PI + q_des(2)]
    q_des(2) = acos((pow(L_upper,2) + pow(L_lower,2) - pow(L_C,2)) / (2 * L_upper * L_lower)) - M_PI;//Angle of the left knee.
    //q_des(3) can be geomatically obtained using vector L_r & alpha
    q_des(3) = -atan2(L_r(0), sqrt(pow(L_r(1),2) + pow(L_r(2),2))) + L_alpha;//Angle of the left ankle.


    //Right hipx, hipy, knee, ankle angle
    q_des(4) = atan2(R_hip_rot(2,1), R_hip_rot(1,1));
    q_des(5) = atan2(-R_hip_rot(2,0), R_hip_rot(2,2));
    q_des(6) = acos((pow(L_upper,2) + pow(L_lower,2) - pow(R_C,2)) / (2 * L_upper * L_lower)) - M_PI;
    q_des(7) = -atan2(R_r(0), sqrt(pow(R_r(1),2) + pow(R_r(2),2))) + R_alpha;
}

void CustomController::updateNextStepTime()
{
    if(walking_tick_ == t_last_) 
    { 
        if(current_step_num_ != total_step_num_-1)
        { 
        t_start_ = t_last_ + 1 ; 
        t_start_real_ = t_start_ + t_rest_init_; 
        t_last_ = t_start_ + t_total_ -1;  
        current_step_num_ ++; 
        }
    }
    if(current_step_num_ == total_step_num_-1 && walking_tick_ >= t_last_ + t_total_)
    {
        walking_enable_ = false;
    }

    walking_tick_ ++;
}





}
