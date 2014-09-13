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

#include <std_msgs/Float64MultiArray.h>

#include <unistd.h>
namespace my_controller_ns{

class MyControllerClass: public pr2_controller_interface::Controller
{
private:

  pr2_mechanism_model::JointState** joint_state_;
  std::string* joint_names_;
  int num_of_joints_;
  int pid_multiplier_;
  int num_of_r_joints_;
  float* init_pos_;
  float* current_pos_;
  float* desired_effort_;
  float* pid_effort_;

  control_toolbox::Pid* pid_controller_;
  pr2_mechanism_model::RobotState *robot_;
  ros::Time time_of_last_cycle_;

  std::vector<double> traj_recieve_;
  float **traj_pos_;
  float **traj_force_;
  int traj_count_;
  float traj_count_sub_;
  float rate_;
  int use_force_;
  int traj_length_;
  int traj_delay_;


  char** temp;
  std::string st;
  ros::Subscriber sub;
  ros::Publisher jacobian_pub;
  ros::Publisher pid_pub;
  ros::Publisher jtf_pub;


  // The chain of links and joints                                                                                                                                                
  pr2_mechanism_model::Chain chain_;
  KDL::Chain kdl_chain_;

  // KDL Solvers performing the actual computations                                                                                                                               
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  // The variables (which need to be pre-allocated).                                                                                                                              
  KDL::JntArray  q_;            // Joint positions                                                                                                                                
  KDL::Jacobian  J_;            // Jacobian

public:
  virtual bool init(pr2_mechanism_model::RobotState *robot,
                   ros::NodeHandle &n);
  bool get_num_of_joints(ros::NodeHandle &n);
  bool get_pid_multiplier(ros::NodeHandle &n);
  bool init_state_and_pid(ros::NodeHandle &n);
  std::string int_to_string(int value);
  void chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void starting();
  void update();
  void stopping();
};
} 