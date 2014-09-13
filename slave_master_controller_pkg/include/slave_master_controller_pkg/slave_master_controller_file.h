#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/chain.h>
#include <pr2_mechanism_model/robot.h>
#include <control_toolbox/pid.h>
#include <ros/ros.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace slave_master_controller_ns{

class SlaveMasterControllerClass: public pr2_controller_interface::Controller
{
private:

  // Master Slave
  pr2_mechanism_model::JointState** joint_state_;
  std::string* joint_names_;
  int num_of_joints_;
  double* init_pos_;
  double* current_pos_;
  control_toolbox::Pid* pid_controller_;
  pr2_mechanism_model::RobotState *robot_;
  ros::Time time_of_last_cycle_;


  
  //Jacobian
  // The chain of links and joints                                                                                                                                                
  pr2_mechanism_model::Chain chain_;
  KDL::Chain kdl_chain_;

  // KDL Solvers performing the actual computations                                                                                                                               
  boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  // The variables (which need to be pre-allocated).                                                                                                                              
  KDL::JntArray  q_;            // Joint positions                                                                                                                                
  KDL::JntArray  q0_;           // Joint initial positions                                                                                                                        
  KDL::JntArrayVel  qdot_;      // Joint velocities                                                                                                                               
  KDL::JntArray  tau_;          // Joint torques                                                                                                                                  

  KDL::Frame     x_;            // Tip pose                                                                                                                                       
  KDL::Frame     xd_;           // Tip desired pose                                                                                                                               
  KDL::Frame     x0_;           // Tip initial pose                                                                                                                               

  KDL::Twist     xerr_;         // Cart error                                                                                                                                     
  KDL::Twist     xdot_;         // Cart velocity                                                                                                                                  
  KDL::Wrench    F_;            // Cart effort                                                                                                                                    
  KDL::Jacobian  J_;            // Jacobian

  // Master Slave


public:
  virtual bool init(pr2_mechanism_model::RobotState *robot,
                   ros::NodeHandle &n);
  bool get_num_of_joints(ros::NodeHandle &n);
  bool init_state_and_pid(ros::NodeHandle &n);
  std::string int_to_string(int value);
  void starting();
  void update();
  void stopping();
};
} 