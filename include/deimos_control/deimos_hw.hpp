#ifndef DEIMOS_HW_HPP
#define DEIMOS_HW_HPP

#include <map>
#include <array>
#include <vector>
#include <tmotor.hpp>
#include "../dynamixel_sdk/dynamixel_sdk.h"
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>

#define TMOTORS_FIRST_ID 11 
#define CAN_NETWORK "can0"

#define PROTOCOL_VERSION                1.0
#define DXL_ID                          1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"

#define ADDR_MX_TORQUE_ENABLE           24
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_MOVING_SPEED            32
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_MOVING                  46

// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_MOVING                   1

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MOVING_STATUS_THRESHOLD     10

namespace deimos_control
{
  /**
   * @brief The RoboticArm class
   * 
   * This class is the main class of the robotic arm. It is responsible for interfacing with the TMotor AK60 library and the ROS control system. 
   * It also handles the transmission interfaces for mechanical reduction.
   */
  class RoboticArm : public hardware_interface::RobotHW
  {
  public:
    RoboticArm();
    void write();
    void read();

  private:
    /* Robot configuration */
    enum MotorType
    {
      DYNAMIXEL,
      AK60
    };

    static const size_t m_number_of_joints;
    static const std::vector<double> m_joint_reductions;
    static const std::vector<std::string> m_joint_names;
    std::vector<TMotor::AKManager> m_ak_managers;
    dynamixel::PortHandler *m_port_handler;
    dynamixel::PacketHandler *m_packet_handler;
    ros::NodeHandle m_nh;
    hardware_interface::JointStateInterface m_jnt_state_interface;
    hardware_interface::PosVelJointInterface m_posvel_joint_interface;
    hardware_interface::PosVelAccJointInterface m_posvelacc_joint_interface;
    std::vector<double> m_arr_curr_pos;
    std::vector<double> m_arr_curr_vel;
    std::vector<double> m_arr_curr_eff;
    std::vector<double> m_arr_cmd_pos;
    std::vector<double> m_arr_cmd_vel;
    std::vector<double> m_arr_cmd_acc;
  }; // class RoboticArm
} // namespace deimos_control



#endif // DEIMOS_HW_HPP