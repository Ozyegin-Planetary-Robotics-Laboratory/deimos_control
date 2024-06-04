#ifndef DEIMOS_HW_HPP
#define DEIMOS_HW_HPP

#include <map>
#include <array>
#include <vector>
#include <tmotor/MotorFeedback.h>
#include <tmotor/MotorCommand.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>

/**
 * @brief The RoboticArm class
 * 
 * This class is the main class of the robotic arm. It is responsible for interfacing with the TMotor AK60 library and the ROS control system. 
 * It also handles the transmission interfaces for mechanical reduction.
 */
class RoboticArm : public hardware_interface::RobotHW {
public:
  RoboticArm();
  void write();

private:
  /* Robot configuration */
  static const size_t m_number_of_joints;
  static const std::vector<double> m_joint_reductions;
  static const std::vector<std::string> m_joint_names;
  ros::NodeHandle m_nh;
  std::vector<ros::Publisher> m_pubs;
  std::vector<ros::Subscriber> m_subs;
  hardware_interface::JointStateInterface m_jnt_state_interface;
  hardware_interface::PosVelAccJointInterface m_posvelacc_joint_interface;
  bool m_shutdown;
  std::vector<double> m_arr_curr_pos;
  std::vector<double> m_arr_curr_vel;
  std::vector<double> m_arr_curr_eff;
  std::vector<double> m_arr_cmd_pos;
  std::vector<double> m_arr_cmd_vel;
  std::vector<double> m_arr_cmd_acc;

  void motorFeedbackCallback(const tmotor::MotorFeedback::ConstPtr& msg, int idx);
};

#endif // DEIMOS_HW_HPP