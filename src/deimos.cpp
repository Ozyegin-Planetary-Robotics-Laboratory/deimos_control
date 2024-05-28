#include <map>
#include <array>
#include <vector>
#include <tmotor/MotorFeedback.h>
#include <tmotor/MotorCommand.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>

/* Robot configuration */
static const int number_of_joints = 4;
static const std::vector<double> joint_reductions = {29, 29, 20, 10};
static const std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4"};

/**
 * @brief The RoboticArm class
 * 
 * This class is the main class of the robotic arm. It is responsible for interfacing with the TMotor AK60 library and the ROS control system. 
 * It also handles the transmission interfaces for mechanical reduction.
 */
class RoboticArm : public hardware_interface::RobotHW {
public:
  RoboticArm():
    m_shutdown(false),
    nh("/")
  { 
    /* Create the actual joint command and state interfaces alongside transmission interfaces  */
    for (int i = 0; i < number_of_joints; i++) {
      a_curr_pos[i] = 0.0;
      a_curr_vel[i] = 0.0;
      a_curr_eff[i] = 0.0;
      a_cmd_pos[i] = 0.0;
      a_cmd_vel[i] = 0.0;
      a_cmd_acc[i] = 0.0;
      std::string feedback_topic = "actuator" + std::to_string(i+1) + "/motor_feedback";
      std::string command_topic = "actuator" + std::to_string(i+1) + "/motor_command";
      pubs.emplace_back(nh.advertise<tmotor::MotorCommand>(command_topic, 1));
      subs.emplace_back(nh.subscribe<tmotor::MotorFeedback>(feedback_topic, 1, boost::bind(&RoboticArm::motorFeedbackCallback, this, _1, i)));
      hardware_interface::PosVelAccJointHandle posvelacc_handle(jnt_state_interface.getHandle(joint_names[i]), &a_cmd_pos[i], &a_cmd_vel[i], &a_cmd_acc[i]);
      hardware_interface::JointStateHandle state_handle(joint_names[i], &a_curr_pos[i], &a_curr_vel[i], &a_curr_eff[i]);
      posvelacc_joint_interface.registerHandle(posvelacc_handle);
      jnt_state_interface.registerHandle(state_handle);
    }
    registerInterface(&posvelacc_joint_interface);
    registerInterface(&jnt_state_interface);
  }

  void write() {
    for (int i = 0; i < number_of_joints; i++) {
      tmotor::MotorCommand msg;
      msg.position = a_cmd_pos[i];
      msg.velocity = a_cmd_vel[i];
      msg.acceleration = a_cmd_acc[i];
      pubs[i].publish(msg);
    }
  }

private:
  ros::NodeHandle nh;
  std::vector<ros::Publisher> pubs;
  std::vector<ros::Subscriber> subs;
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PosVelAccJointInterface posvelacc_joint_interface;
  bool m_shutdown;
  double a_curr_pos[number_of_joints];
  double a_curr_vel[number_of_joints];
  double a_curr_eff[number_of_joints];
  double a_cmd_pos[number_of_joints];
  double a_cmd_vel[number_of_joints];
  double a_cmd_acc[number_of_joints];

  void motorFeedbackCallback(const tmotor::MotorFeedback::ConstPtr& msg, int idx) {
    a_curr_pos[idx] = msg->position;
    a_curr_vel[idx] = msg->velocity;
    a_curr_eff[idx] = msg->current;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "deimos_robot");
  RoboticArm deimos;
  controller_manager::ControllerManager cm(&deimos);
  ros::Time prev_time = ros::Time(0);
  while (ros::ok()) {
    const ros::Time time = ros::Time::now();
    ros::spinOnce();
    cm.update(time, time - prev_time);
    prev_time = time;
    deimos.write();
    sleep(0.05);
  }
  return 0;
}