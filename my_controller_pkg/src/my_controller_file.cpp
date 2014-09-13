
#include "my_controller_pkg/my_controller_file.h"
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <unistd.h>
#include <std_msgs/Float64MultiArray.h>
namespace my_controller_ns {



void MyControllerClass::chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  traj_recieve_ = msg->data;
  traj_length_ = (msg->data.size()-2)/10.0;
  printf("\n'%f'", traj_length_);
  printf("\n'%f'", traj_recieve_[msg->data.size()-2]);
  printf("\n'%f'", traj_recieve_[msg->data.size()-1]);
  //Joint positions 7 x n
  traj_pos_ = new float*[num_of_joints_];
  for (int i = 0; i < num_of_joints_; ++i) {
    traj_pos_[i] = new float[traj_length_];
    //printf("\n");
    for (int j = 0; j < traj_length_; ++j) {
      traj_pos_[i][j] = traj_recieve_[i * traj_length_ + j];
      //printf("'%f'", traj_pos_[i][j]);
    }
  }

  //End effector forces 3 x n
  traj_force_ = new float*[3];
  for (int i = 0; i < 3; ++i) {
    traj_force_[i] = new float[traj_length_];
    for (int j = 0; j < traj_length_; ++j) {
      traj_force_[i][j] = traj_recieve_[num_of_joints_ * traj_length_ + i * traj_length_ + j]; // with offset 7xn
    }
  }


  //Whether or not to use eff
  use_force_ = traj_recieve_[10 * traj_length_ + 1];

  if(use_force_ == 0.0) {
    printf("\n not using force");
  } else {
    printf("\n using force");
  }

  //Compute Rate
  float secs = traj_recieve_[10 * traj_length_];
  rate_ = 1000.0 * secs / float(traj_length_);

  printf("\nsecs '%f'", secs);
  printf("\nlength '%f'", float(traj_length_));
  printf("\nrate '%f'", rate_);


  traj_count_ = 0;
  traj_count_sub_ = 0;
  printf("\nGot Data\n");
}

/// Controller initialization in non-realtime
bool MyControllerClass::init(pr2_mechanism_model::RobotState *robot,
                            ros::NodeHandle &n)
{
  
  robot_ = robot;


  std::string root_name, tip_name;
  if (!n.getParam("root_name", root_name))
  {
    ROS_ERROR("No root name given in namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("tip_name", tip_name))
  {
    ROS_ERROR("No tip name given in namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  // Construct a chain from the root to the tip and prepare the kinematics.                                                                                                       
  // Note the joints must be calibrated.                                                                                                                                          
  if (!chain_.init(robot, root_name, tip_name))
  {
    ROS_ERROR("Jacobian could not use the chain from '%s' to '%s'",
              root_name.c_str(), tip_name.c_str());
    return false;
  }


  // Construct the kdl solvers in non-realtime.                                                                                                                                   
  chain_.toKDL(kdl_chain_);
  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

  // Resize (pre-allocate) the variables in non-realtime.                                                                                                                         
  q_.resize(kdl_chain_.getNrOfJoints());
  J_.resize(kdl_chain_.getNrOfJoints());

  if(!get_num_of_joints(n)) {
    return false;
  }

  if(!get_pid_multiplier(n)) {
    return false;
  }

  if(!init_state_and_pid(n)) {
    return false;
  }
  
  traj_pos_ = new float*[1];
  traj_pos_[0] = new float[1];
  traj_length_ = 0;
  traj_count_ = 0;
  traj_count_sub_ = 0.0;
  traj_delay_ = 0;

  temp = new char*;
  int nu = 0;
  int& num = nu;
  
  ros::init(num, temp, "my_controller");
  
  sub = n.subscribe("/joint_positions_forces_secs", 1000, &MyControllerClass::chatterCallback, this);
  jacobian_pub = n.advertise<std_msgs::Float64MultiArray>("jacobian", 100);
  ROS_ERROR("first pub");
  pid_pub = n.advertise<std_msgs::Float64MultiArray>("pid_torques", 100);
  ROS_ERROR("second pub");
  jtf_pub = n.advertise<std_msgs::Float64MultiArray>("jtf_torques", 100);
  ROS_ERROR("third pub");

  
  return true;
}

bool MyControllerClass::get_num_of_joints(ros::NodeHandle &n) {
    if(!n.getParam("num_of_joints", num_of_joints_)) {
      ROS_ERROR("No parameter named 'num_of_joints'");
      return false;
    }
    return true;
  }

bool MyControllerClass::get_pid_multiplier(ros::NodeHandle &n) {
    if(!n.getParam("pid_multiplier", pid_multiplier_)) {
      ROS_ERROR("No parameter named 'pid_multiplier'");
      return false;
    }
    return true;
  }

bool MyControllerClass::init_state_and_pid(ros::NodeHandle &n) {


    //Master Slave
    std::string joint_const("joint_name_");
    std::string pid_const("pid_parameters_");
    std::string joint_name;

    joint_state_ = new pr2_mechanism_model::JointState*[num_of_joints_]; 
    joint_names_ = new std::string[num_of_joints_];
    pid_controller_ = new control_toolbox::Pid[num_of_joints_];
    current_pos_ = new float[num_of_joints_];
    init_pos_ = new float[num_of_joints_];

    desired_effort_ = new float[num_of_joints_];
    pid_effort_ = new float[num_of_joints_];
    ROS_ERROR("init arrays");

    for(int i = 1; i <= num_of_joints_; i++) {
      if(!n.getParam((joint_const + int_to_string(i)).c_str(), joint_name)) {
        ROS_ERROR("No joint number %d in namespace: '%s'", i, n.getNamespace().c_str());
        return false;
      }

      joint_state_[i - 1] = robot_->getJointState(joint_name);
      joint_names_[i - 1] = joint_name;

      if(!joint_state_[i - 1]) {
        ROS_ERROR("MyController could not find joint name '%s'", joint_name.c_str());
        return false;
      }

      if(!pid_controller_[i - 1].init(ros::NodeHandle(n, (pid_const + int_to_string(i)).c_str()))) {
        ROS_ERROR("MyController could no construct PID controller for joint '%s'", joint_name.c_str());
        return false;
      }
    }

    return true;
  }


/// Controller startup in realtime
void MyControllerClass::starting()
{
  for(int i = 1; i <= num_of_joints_; i++) {
    init_pos_[i-1] = joint_state_[i-1]->position_;
    time_of_last_cycle_ = robot_->getTime();
    pid_controller_[i-1].reset();
  }
}


/// Controller update loop in realtime
void MyControllerClass::update()
{
  ros::spinOnce();
  ros::Duration dt = robot_->getTime() - time_of_last_cycle_;
  time_of_last_cycle_ = robot_->getTime();


  // Get the current joint positions and compute Jacobian (at this location)                                                                           
  chain_.getPositions(q_);
  jnt_to_jac_solver_->JntToJac(q_, J_);
  std_msgs::Float64MultiArray jacobian_msg;
  jacobian_msg.data.resize(42);
  for (unsigned int i = 0 ; i < 6 ; i++)
  {
    for (unsigned int j = 0 ; j < num_of_joints_ ; j++)
    {
      jacobian_msg.data[i*7+j] = J_(i,j);
    }
  }
  jacobian_pub.publish(jacobian_msg);


  if(traj_length_ == 0) {//traj_pos_[0][0] < 0.01 && traj_pos_[0][0] > -0.01) {
    //printf("\nNo Trajectory");
    return;
  }

  if(traj_count_ >= traj_length_ - 1) {
    traj_count_ = traj_length_ - 2;
    traj_count_sub_ = rate_;
  }


  


  // Shoulder Pan Joint, Shouler Lift Joint, Upper Arm Roll Joint, Elbow Flex Joint, Forearm Roll Joint, Wrist Flex Joint, Wrist Roll Joint
  for(int i = 0; i < num_of_joints_; i++)
  {
    float current_pos = joint_state_[i]->position_;
    float desired_pos = traj_pos_[i][traj_count_] + (traj_pos_[i][traj_count_+1] - traj_pos_[i][traj_count_]) * traj_count_sub_ / rate_;
    float pid_effort = pid_controller_[i].updatePid(current_pos-desired_pos, dt);
    pid_effort_[i] = pid_effort;
    if(use_force_ == 0.0) {
      joint_state_[i]->commanded_effort_ = pid_effort;
    } else {
      // desired_effort = Jac^transpose * end_effector_force
      for (unsigned int i = 0; i < q_.rows(); i++)
      {
        desired_effort_[i] = 0;
        for (unsigned int j=0; j<3; j++) {
          float force = traj_force_[j][traj_count_] + (traj_force_[j][traj_count_+1] - traj_force_[j][traj_count_]) * traj_count_sub_ / rate_;
          desired_effort_[i] += (J_(j,i) * force);
        }        
      }
      float final_effort = (1.0/float(pid_multiplier_)) * pid_effort + desired_effort_[i];
      joint_state_[i]->commanded_effort_ = final_effort;
    }
  }
  ROS_ERROR("Updated arrays");
  if(use_force_ != 0.0) {
    std_msgs::Float64MultiArray pid_msg;
    pid_msg.data.resize(7);

    std_msgs::Float64MultiArray jtf_msg;
    jtf_msg.data.resize(7);

  
    for (unsigned int i = 0 ; i < num_of_joints_ ; i++)
    {
      pid_msg.data[i] = pid_effort_[i];
      jtf_msg.data[i] = desired_effort_[i];
    }

    pid_pub.publish(pid_msg);
    jtf_pub.publish(jtf_msg);
    ROS_ERROR("published");
  }

  traj_count_sub_++;
  if(traj_count_sub_ >= rate_ - 1.0) {
    //printf("\n Time Step '%d'", traj_count_);
    traj_count_++;
    traj_count_sub_ = 0.0;
    
  }



}

/// Controller stopping in realtime
void MyControllerClass::stopping()
{}


std::string MyControllerClass::int_to_string(int value) {
  std::ostringstream os;
  os << value;
  return os.str();
}


} // namespace
/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(my_controller_pkg,MyControllerPlugin, 
                         my_controller_ns::MyControllerClass, 
                         pr2_controller_interface::Controller)