#include "../include/dynamixel_sdk/dynamixel_sdk.h"
#include "../include/deimos_control/deimos_hw.hpp"

const size_t deimos_control::RoboticArm::m_number_of_joints(4);
const std::vector<double> deimos_control::RoboticArm::m_joint_reductions = {29, 44, 20, 10};
const std::vector<std::string> deimos_control::RoboticArm::m_joint_names = {"joint1", "joint2", "joint3", "joint4"};

deimos_control::RoboticArm::RoboticArm():
  m_port_handler(dynamixel::PortHandler::getPortHandler(DEVICENAME)),
  m_packet_handler(dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION)),
  m_nh("~")
{ 
  // Initialize TMotor AK60s
  m_arr_cmd_acc.resize(m_number_of_joints+1, 0.0);
  m_arr_cmd_pos.resize(m_number_of_joints+1, 0.0);
  m_arr_cmd_vel.resize(m_number_of_joints+1, 0.0);
  m_arr_curr_eff.resize(m_number_of_joints+1, 0.0);
  m_arr_curr_pos.resize(m_number_of_joints+1, 0.0);
  m_arr_curr_vel.resize(m_number_of_joints+1, 0.0);
  m_ak_managers.reserve(m_number_of_joints+1);
  for (int i = 0; i < m_number_of_joints; i++)
  {
    m_arr_curr_pos[i] = 0.0;
    m_arr_curr_vel[i] = 0.0;
    m_arr_curr_eff[i] = 0.0;
    m_arr_cmd_pos[i] = 0.0;
    m_arr_cmd_vel[i] = 0.0;
    m_arr_cmd_acc[i] = 0.0;
    hardware_interface::PosVelAccJointHandle posvelacc_handle(m_jnt_state_interface.getHandle(m_joint_names[i]), &m_arr_cmd_pos[i], &m_arr_cmd_vel[i], &m_arr_cmd_acc[i]);
    hardware_interface::JointStateHandle state_handle(m_joint_names[i], &m_arr_curr_pos[i], &m_arr_curr_vel[i], &m_arr_curr_eff[i]);
    m_posvelacc_joint_interface.registerHandle(posvelacc_handle);
    m_jnt_state_interface.registerHandle(state_handle);
    m_ak_managers.emplace_back(i + TMOTORS_FIRST_ID);
    m_ak_managers[i].connect(CAN_NETWORK);
  }

  // Initialize Dynamixel motors
  hardware_interface::PosVelJointHandle posvel_handle(m_jnt_state_interface.getHandle("joint5"), &m_arr_cmd_pos[4], &m_arr_cmd_vel[4]);
  hardware_interface::JointStateHandle state_handle("joint5", &m_arr_curr_pos[4], &m_arr_curr_vel[4], &m_arr_curr_eff[4]);
  m_posvel_joint_interface.registerHandle(posvel_handle);
  m_jnt_state_interface.registerHandle(state_handle);

  if (!m_port_handler->openPort())
  {
    ROS_ERROR("Failed to open the port!");
  }
  if (!m_port_handler->setBaudRate(1000000))
  {
    ROS_ERROR("Failed to change the baudrate!");
  }

  registerInterface(&m_posvelacc_joint_interface);
  registerInterface(&m_posvel_joint_interface);
  registerInterface(&m_jnt_state_interface);
}

void deimos_control::RoboticArm::write()
{
  for (int i = 0; i < m_number_of_joints; i++)
  {
    m_ak_managers[i].sendPositionVelocityAcceleration(m_arr_cmd_pos[i], m_arr_cmd_vel[i], m_arr_cmd_acc[i]);
  }
  uint8_t dxl_write_error = 0;
  int dxl_write_result = m_packet_handler->write2ByteTxRx(m_port_handler, DXL_ID, ADDR_MX_GOAL_POSITION,  static_cast <uint16_t> (m_arr_cmd_pos[4]/360.0)*4096, &dxl_write_error);
  if (dxl_write_result != COMM_SUCCESS)
  {
    ROS_ERROR("%s", m_packet_handler->getTxRxResult(dxl_write_result));
  }
  else if (dxl_write_error != 0)
  {
    ROS_ERROR("%s", m_packet_handler->getRxPacketError(dxl_write_error));
  }
}

void deimos_control::RoboticArm::read()
{
  for (int i = 0; i < m_number_of_joints; i++)
  {
    m_arr_curr_pos[i] = m_ak_managers[i].getPosition();
    m_arr_curr_vel[i] = m_ak_managers[i].getVelocity();
    m_arr_curr_eff[i] = m_ak_managers[i].getCurrent();
  }
  uint8_t dxl_read_error = 0;
  uint16_t dxl_present_position = 0;
  int dxl_comm_result = m_packet_handler->read2ByteTxRx(m_port_handler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_read_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("%s", m_packet_handler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_read_error != 0)
  {
    ROS_ERROR("%s", m_packet_handler->getRxPacketError(dxl_read_error));
  }
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "deimos_robot");
  
  deimos_control::RoboticArm deimos;
  controller_manager::ControllerManager cm(&deimos);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(10.0);

  while (ros::ok())
  {
    const ros::Time     time   = ros::Time::now();
    const ros::Duration period = time - prev_time;

    deimos.read();
    cm.update(time, period);
    deimos.write();

    rate.sleep();
  }
}