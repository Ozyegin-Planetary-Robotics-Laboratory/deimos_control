#include <thread>
#include <chrono>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tmotor.hpp>

static const float MOTOR_MAX_VELOCITY = 5.0f;
std::vector<float> velocity_commands(4, 0.0);

void joyCallback(sensor_msgs::JoyConstPtr &msg)
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

        velocity_commands[0] = msg->axes[2];
        velocity_commands[1] = msg->axes[1];
        velocity_commands[2] = msg->axes[0];
        velocity_commands[3] = msg->axes[5];
    }
    else
    {
        velocity_commands[0] = 0.0;
        velocity_commands[1] = 0.0;
        velocity_commands[2] = 0.0;
        velocity_commands[3] = 0.0;
        // to do - dynamixel and eof
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick");
    ros::NodeHandle nh;

    std::vector<int> motor_ids;
    ros::param::get("motor_ids", motor_ids);
    if (motor_ids.size() != 4)
    {
        ROS_ERROR("Motor ids must be a list of 4 integers");
        return 1;
    }
    std::string can_interface;
    ros::param::get("can_interface", can_interface);
    if (can_interface != "can0" && can_interface != "vcan0")
    {
        ROS_ERROR("CAN interface must be either can0 or can1");
        return 1;
    }
    std::vector <TMotor::AKManager> motors;

    motors.reserve(4);
    for (std::vector<int>::const_iterator id = motor_ids.begin(); id != motor_ids.end(); ++id)
    {
        motors.emplace_back(*id);
        motors.back().connect(can_interface.c_str());
    }

    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy> ("joy", 10, &joyCallback);    
    ros::Rate command_frequency(10.0);
    while (ros::ok())
    {
        command_frequency.sleep();
        ros::spinOnce();
        for (size_t i = 0; i < 4; ++i)
        {
            motors[i].sendVelocity(velocity_commands[i]);
            velocity_commands[i] *= 0.95;
        }
    }
}