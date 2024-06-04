#include "../include/deimos_control/deimos_hw.hpp"

const size_t RoboticArm::m_number_of_joints(4);
const std::vector<double> RoboticArm::m_joint_reductions = {29, 29, 20, 10};
const std::vector<std::string> RoboticArm::m_joint_names = {"joint1", "joint2", "joint3", "joint4"};

RoboticArm::RoboticArm():
  m_shutdown(false),
  m_nh("/")
{ 
  // Initialize member arrays.
  m_arr_cmd_acc.resize(m_number_of_joints, 0.0);
  m_arr_cmd_pos.resize(m_number_of_joints, 0.0);
  m_arr_cmd_vel.resize(m_number_of_joints, 0.0);
  m_arr_curr_eff.resize(m_number_of_joints, 0.0);
  m_arr_curr_pos.resize(m_number_of_joints, 0.0);
  m_arr_curr_vel.resize(m_number_of_joints, 0.0);
  for (int i = 0; i < m_number_of_joints; i++) {
    m_arr_curr_pos[i] = 0.0;
    m_arr_curr_vel[i] = 0.0;
    m_arr_curr_eff[i] = 0.0;
    m_arr_cmd_pos[i] = 0.0;
    m_arr_cmd_vel[i] = 0.0;
    m_arr_cmd_acc[i] = 0.0;
    std::string feedback_topic = "actuator" + std::to_string(i+1) + "/motor_feedback";
    std::string command_topic = "actuator" + std::to_string(i+1) + "/motor_command";
    m_pubs.emplace_back(m_nh.advertise<tmotor::MotorCommand>(command_topic, 1));
    m_subs.emplace_back(m_nh.subscribe<tmotor::MotorFeedback>(feedback_topic, 1, boost::bind(&RoboticArm::motorFeedbackCallback, this, _1, i)));
    hardware_interface::PosVelAccJointHandle posvelacc_handle(m_jnt_state_interface.getHandle(m_joint_names[i]), &m_arr_cmd_pos[i], &m_arr_cmd_vel[i], &m_arr_cmd_acc[i]);
    hardware_interface::JointStateHandle state_handle(m_joint_names[i], &m_arr_curr_pos[i], &m_arr_curr_vel[i], &m_arr_curr_eff[i]);
    m_posvelacc_joint_interface.registerHandle(posvelacc_handle);
    m_jnt_state_interface.registerHandle(state_handle);
  }
  registerInterface(&m_posvelacc_joint_interface);
  registerInterface(&m_jnt_state_interface);
}

void RoboticArm::write() {
  for (size_t i = 0; i < m_number_of_joints; i++) {
    tmotor::MotorCommand msg;
    msg.position = m_arr_cmd_pos[i];
    msg.velocity = m_arr_cmd_vel[i];
    msg.acceleration = m_arr_cmd_acc[i];
    m_pubs[i].publish(msg);
  }
}

void RoboticArm::motorFeedbackCallback(const tmotor::MotorFeedback::ConstPtr& msg, int idx) {
  m_arr_curr_pos[idx] = msg->position;
  m_arr_curr_vel[idx] = msg->velocity;
  m_arr_curr_eff[idx] = msg->current;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "deimos_robot");
  RoboticArm deimos;
  controller_manager::ControllerManager cm(&deimos);
  ros::Time prev_time = ros::Time(0);
  ros::Rate rate(30.0); // 10 Hz rate
  while (ros::ok()) {
    const ros::Time time = ros::Time::now();
    ros::spinOnce();
    cm.update(time, time - prev_time);
    prev_time = time;
    deimos.write();
    rate.sleep();
  }
}