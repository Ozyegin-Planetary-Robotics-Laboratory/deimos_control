#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tmotor.hpp>

float motor_velocity = 0.0f;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    motor_velocity = joy->axes[0] * 100;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick");
    ros::NodeHandle nh;
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy> ("joy", 10, &joyCallback);
    ROS_INFO("Subscription successful.");

    TMotor::AKManager motor(16);
    motor.connect("can0");
    ROS_INFO("Connected to the can network.");

    ros::Rate command_frequency(10.0);
    while (ros::ok())
    {
        command_frequency.sleep();
        motor.sendVelocity(motor_velocity);
        ros::spinOnce();
        ROS_INFO("Sending command: %f\n", motor_velocity);
    }
}