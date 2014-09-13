
#include "slave_master_controller_pkg/slave_master_controller_file.h"
#include <pluginlib/class_list_macros.h>

namespace slave_master_controller_ns {


/// Controller initialization in non-realtime
bool SlaveMasterControllerClass::init(pr2_mechanism_model::RobotState *robot,
                            ros::NodeHandle &n)
{
  robot_ = robot;
  if(!get_num_of_joints(n)) {
    return false;
  }

  if(!init_state_and_pid(n)) {
    return false;
  }
  




  //Jacobian
  /*
  robot_ = robot;
  // Get the root and tip link names from parameter server.                                                                                                           
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
    ROS_ERROR("MyCartController could not use the chain from '%s' to '%s'",
              root_name.c_str(), tip_name.c_str());
    return false;
  }
  // Construct the kdl solvers in non-realtime.                                                                                                                                   
  chain_.toKDL(kdl_chain_);
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

  // Resize (pre-allocate) the variables in non-realtime.                                                                                                                         
  q_.resize(kdl_chain_.getNrOfJoints());
  q0_.resize(kdl_chain_.getNrOfJoints());
  qdot_.resize(kdl_chain_.getNrOfJoints());
  tau_.resize(kdl_chain_.getNrOfJoints());
  J_.resize(kdl_chain_.getNrOfJoints());
  printf("finished init");
  */
  return true;
}

bool SlaveMasterControllerClass::get_num_of_joints(ros::NodeHandle &n) {
    if(!n.getParam("num_of_joints", num_of_joints_)) {
      ROS_ERROR("No parameter named 'num_of_joints'");
      return false;
    }
    return true;
  }

bool SlaveMasterControllerClass::init_state_and_pid(ros::NodeHandle &n) {


    //Master Slave
    std::string joint_const("joint_name_");
    std::string pid_const("pid_parameters_");
    std::string joint_name;

    joint_state_ = new pr2_mechanism_model::JointState*[num_of_joints_]; 
    joint_names_ = new std::string[num_of_joints_];
    pid_controller_ = new control_toolbox::Pid[num_of_joints_];
    current_pos_ = new double[num_of_joints_];
    init_pos_ = new double[num_of_joints_];

    for(int i = 1; i <= num_of_joints_; i++) {
      if(!n.getParam((joint_const + int_to_string(i)).c_str(), joint_name)) {
        ROS_ERROR("No joint number %d in namespace: '%s'", i, n.getNamespace().c_str());
        return false;
      }

      joint_state_[i - 1] = robot_->getJointState(joint_name);
      joint_names_[i - 1] = joint_name;

      if(!joint_state_[i - 1]) {
        ROS_ERROR("SlaveMasterController could not find joint name '%s'", joint_name.c_str());
        return false;
      }

      if(!pid_controller_[i - 1].init(ros::NodeHandle(n, (pid_const + int_to_string(i)).c_str()))) {
        ROS_ERROR("SlaveMasterController could no construct PID controller for joint '%s'", joint_name.c_str());
        return false;
      }
    }

    return true;
  }


/// Controller startup in realtime
void SlaveMasterControllerClass::starting()
{
  //Master Slave
  
  for(int i = 1; i <= num_of_joints_; i++) {
    init_pos_[i-1] = joint_state_[i-1]->position_;
    time_of_last_cycle_ = robot_->getTime();
    pid_controller_[i-1].reset();
  }
  
  /*
  // Jacobian
  // Get the current joint values to compute the initial tip location.
  time_of_last_cycle_ = robot_->getTime();                                                                                                    
  chain_.getPositions(q0_);
  jnt_to_pose_solver_->JntToCart(q0_, x0_);
  printf("finished start");
  */
}


/// Controller update loop in realtime
void SlaveMasterControllerClass::update()
{

  ros::Duration dt = robot_->getTime() - time_of_last_cycle_;
  time_of_last_cycle_ = robot_->getTime();



  // Jacobian
  // Get the current joint positions and velocities.
  /*                                                                                         
  chain_.getPositions(q_);
  chain_.getVelocities(qdot_);

  // Compute the forward kinematics and Jacobian (at this location).                                                                                                              
  jnt_to_jac_solver_->JntToJac(q_, J_);

  for (unsigned int i = 0 ; i < 6 ; i++)
  {
    xdot_(i) = 0;
    printf("\n");
    for (unsigned int j = 0 ; j < kdl_chain_.getNrOfJoints() ; j++)
    {
      printf("'%f'",J_(i,j));
    }
  }
  */
  
  //Shoulder Pan Joint
  double l_shoulder_pan_desired_pos = init_pos_[0] + 0.1 * sin(ros::Time::now().toSec());
  double l_shoulder_pan_current_pos = joint_state_[0]->position_;
  double r_shoulder_pan_current_pos = joint_state_[1]->position_;
  double r_shoulder_pan_desired_pos = - l_shoulder_pan_current_pos; //negate join angles due to symmytry along y
  joint_state_[1]->commanded_effort_ = pid_controller_[1].updatePid(r_shoulder_pan_current_pos-r_shoulder_pan_desired_pos, dt);

  //Shouler Lift Joint
  double l_shoulder_lift_desired_pos = init_pos_[2] + 0.1 * sin(ros::Time::now().toSec());
  double l_shoulder_lift_current_pos = joint_state_[2]->position_;
  double r_shoulder_lift_current_pos = joint_state_[3]->position_;
  double r_shoulder_lift_desired_pos = l_shoulder_lift_current_pos; //no need to negate here
  joint_state_[3]->commanded_effort_ = pid_controller_[3].updatePid(r_shoulder_lift_current_pos-r_shoulder_lift_desired_pos, dt);

  //Upper Arm Roll Joint
  double l_upperarm_roll_desired_pos = init_pos_[4] + 0.1 * sin(ros::Time::now().toSec());
  double l_upperarm_roll_current_pos = joint_state_[4]->position_;
  double r_upperarm_roll_current_pos = joint_state_[5]->position_;
  double r_upperarm_roll_desired_pos = -l_upperarm_roll_current_pos; //negate join angles due to symmytry along y
  joint_state_[5]->commanded_effort_ = pid_controller_[5].updatePid(r_upperarm_roll_current_pos-r_upperarm_roll_desired_pos, dt);
  
  //Elbow Flex Joint
  double l_elblow_flex_desired_pos = init_pos_[6] + 1 * sin(ros::Time::now().toSec());
  double l_elblow_flex_current_pos = joint_state_[6]->position_;
  double r_elblow_flex_current_pos = joint_state_[7]->position_;
  double r_elblow_flex_desired_pos = l_elblow_flex_current_pos; 
  joint_state_[7]->commanded_effort_ = pid_controller_[7].updatePid(r_elblow_flex_current_pos-r_elblow_flex_desired_pos, dt);

  //Forearm Roll Joint
  double l_forearm_roll_desired_pos = init_pos_[8] + 0.1 * sin(ros::Time::now().toSec());
  double l_forearm_roll_current_pos = joint_state_[8]->position_;
  double r_forearm_roll_current_pos = joint_state_[9]->position_;
  double r_forearm_roll_desired_pos = -l_forearm_roll_current_pos; //negate join angles due to symmytry along y
  joint_state_[9]->commanded_effort_ = pid_controller_[9].updatePid(r_forearm_roll_current_pos-r_forearm_roll_desired_pos, dt);

  //Wrist Flex Joint
  double l_wrist_flex_desired_pos = init_pos_[10] + 0.1 * sin(ros::Time::now().toSec());
  double l_wrist_flex_current_pos = joint_state_[10]->position_;
  double r_wrist_flex_current_pos = joint_state_[11]->position_;
  double r_wrist_flex_desired_pos = l_wrist_flex_current_pos; 
  joint_state_[11]->commanded_effort_ = pid_controller_[11].updatePid(r_wrist_flex_current_pos-r_wrist_flex_desired_pos, dt);

  

  //Wrist Roll Joint
  double l_wrist_roll_desired_pos = init_pos_[12] + 0.1 * sin(ros::Time::now().toSec());
  double l_wrist_roll_current_pos = joint_state_[12]->position_;
  double r_wrist_roll_current_pos = joint_state_[13]->position_;
  double r_wrist_roll_desired_pos = -l_wrist_roll_current_pos; //negate join angles due to symmytry along y
  joint_state_[13]->commanded_effort_ = pid_controller_[13].updatePid(r_wrist_roll_current_pos-r_wrist_roll_desired_pos, dt);

  

}

/// Controller stopping in realtime
void SlaveMasterControllerClass::stopping()
{}


std::string SlaveMasterControllerClass::int_to_string(int value) {
  std::ostringstream os;
  os << value;
  return os.str();
}


} // namespace
/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(slave_master_controller_pkg,SlaveMasterControllerPlugin, 
                         slave_master_controller_ns::SlaveMasterControllerClass, 
                         pr2_controller_interface::Controller)