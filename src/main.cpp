#include <cmath>
#include <thread>
#include <chrono>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tmotor.hpp>

static const float MOTOR_MAX_VELOCITY = 5.0f;
std::vector<float> velocity_commands(4, 0.0f);
std::vector<float> reduction_numbers(4, 0.0f);

void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
void initializeMotors(std::vector<TMotor::AKManager> &motors);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick");
    ros::NodeHandle nh;
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy> ("joy", 10, &joyCallback);  

    std::vector <TMotor::AKManager> motors;
    try
    {
        initializeMotors(motors);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Failed to initialize motors: %s", e.what());
        return 1;
    }

    double freq;
    ros::param::get("control_frequency", freq);
    if (freq <= 0)
    {
        ROS_ERROR("Invalid control frequency");
        return 1;
    }
    freq = std::max(1.0, freq);
    const double beta = 0.03;
    const double decay_constant = 1.0f - exp(-beta*freq);
    ros::Rate control_frequency(freq);
    while (ros::ok())
    {
        control_frequency.sleep();
        ros::spinOnce();
        for (size_t i = 0; i < 4; ++i)
        {
            motors[i].sendVelocity(velocity_commands[i]);
            // velocity_commands[i] *= decay_constant;
        }
    }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{

    if (msg->axes.size() < 6)
    {
        ROS_ERROR("Joystick message has less than 6 axes");
        return;
    }

    if (msg->buttons.size() < 12)
    {
        ROS_ERROR("Joystick message has less than 1 button");
        return;
    }

    const bool toggle_eof = msg->buttons[0];
    const float dynamixel_cmd = msg->axes[2];
    const float end_effector_cmd = msg->axes[1];

    if (!toggle_eof)
    {   
        velocity_commands[0] = std::max(-MOTOR_MAX_VELOCITY, std::min(MOTOR_MAX_VELOCITY, -msg->axes[2]*MOTOR_MAX_VELOCITY)) * reduction_numbers[0];
        velocity_commands[1] = std::max(-MOTOR_MAX_VELOCITY, std::min(MOTOR_MAX_VELOCITY, msg->axes[1]*MOTOR_MAX_VELOCITY)) * reduction_numbers[1];
        velocity_commands[2] = std::max(-MOTOR_MAX_VELOCITY, std::min(MOTOR_MAX_VELOCITY, msg->axes[0]*MOTOR_MAX_VELOCITY)) * reduction_numbers[2];
        velocity_commands[3] = std::max(-MOTOR_MAX_VELOCITY, std::min(MOTOR_MAX_VELOCITY, msg->axes[5]*MOTOR_MAX_VELOCITY)) * reduction_numbers[3];
    }
    else
    {
        velocity_commands[0] = 0.0f;
        velocity_commands[1] = 0.0f;
        velocity_commands[2] = 0.0f;
        velocity_commands[3] = 0.0f;
    }

}

void initializeMotors(std::vector<TMotor::AKManager> &motors)
{
    std::vector<int> motor_ids;
    ros::param::get("motor_ids", motor_ids);
    if (motor_ids.size() != 4)
    {
        throw std::runtime_error("Invalid motor ids");
    }
    std::string can_interface;
    ros::param::get("can_interface", can_interface);
    if (can_interface != "can0" && can_interface != "vcan0")
    {
        throw std::runtime_error("Invalid CAN interface.");
    }
    ros::param::get("reduction_numbers", reduction_numbers);
    if (reduction_numbers.size() != 4)
    {
        throw std::runtime_error("Invalid reduction numbers");
    }
    motors.reserve(4);
    for (std::vector<int>::const_iterator id = motor_ids.begin(); id != motor_ids.end(); ++id)
    {
        motors.emplace_back(*id);
        motors.back().connect(can_interface.c_str());
    }
}