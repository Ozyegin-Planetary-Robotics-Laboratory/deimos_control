#include <ros/ros.h>
#include <deimos_control/deimos_controller.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "deimos_controller");
    ROS_INFO("Starting Deimos controller node...");
    deimos_control::DeimosController controller;
    try
    {
        ROS_INFO("Running Deimos controller...");
        controller.run();
    }
    catch(const std::exception& e)
    {
        ROS_WARN("Program terminated due to some error.");
        ROS_ERROR("Exception caught: %s", e.what());
        return 1;
    }
    ROS_INFO("Deimos controller node stopped.");
}