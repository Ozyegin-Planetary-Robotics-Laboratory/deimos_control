#include <ros/ros.h>
#include <deimos_control/joystick_ctl.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "deimos_controller");
    deimos_control::DeimosController controller;
    try
    {
        controller.run();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Exception caught: %s", e.what());
        return 1;
    }
}