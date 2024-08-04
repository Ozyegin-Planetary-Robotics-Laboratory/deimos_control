#ifndef H_JOYSTICK_CTL
#define H_JOYSTICK_CTL

#include <cmath>
#include <thread>
#include <chrono>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <deimos_control/dynamixel_abi.h>
#include <tmotor.hpp>

namespace deimos_control
{
class DeimosController
{
public:
    DeimosController() :
        _ak_velocity_commands(4, 0.0),
        _reduction_numbers(4, 0.0),
        _motors(4),
        _ak_max_vel(5.0),
        _dxl_id(1),
        _dxl_max_output(5.0),
        _dxl_output_command(0)
    {}

    void run()
    {
        _joy_sub = _nh.subscribe<sensor_msgs::Joy>("/joy", 1, &DeimosController::_joyCallback, this);
        _initializeControlVariables();
        _initializeAKMotors();
        _initializeDynamixel();
        _controlLoop();
    }

private:
    ros::NodeHandle _nh;
    ros::Subscriber _joy_sub;
    double _decay_constant;
    int _dxl_max_output;
    double _ak_max_vel;
    double _control_f;
    uint8_t _dxl_id;
    uint16_t _dxl_output_command;
    std::vector<double> _ak_velocity_commands;
    std::vector<double> _reduction_numbers;
    std::vector<TMotor::AKManager> _motors;
    dynamixel::PortHandler *_port_handler;
    dynamixel::PacketHandler *_packet_handler;

    void _controlLoop()
    {
        ros::Rate rate(_control_f);
        while (ros::ok())
        {
            rate.sleep();
            
            /* Update command. */
            ros::spinOnce();

            /* Read motor states. */
            // to-do

            /* Write to motors. */
            for (size_t i = 0; i < _motors.size(); ++i)
            {
                _motors[i].sendVelocity(_ak_velocity_commands[i]);
            }
            _packet_handler->write2ByteTxRx(_port_handler, _dxl_id, DXL_ADDR_MX_MOVING_SPEED, _dxl_output_command);

        }
    }

    void _joyCallback(const sensor_msgs::JoyConstPtr &msg)
    {
        ROS_WARN("Processing joystick input...");
        if (msg->axes.size() < 6 || msg->buttons.size() < 12) return;
        const bool toggle_eof = msg->buttons[0];
        const float dxl_cmd = std::max(-1.0f, std::min(1.0f, msg->axes[2]));
        const float eof_cmd = std::max(-1.0f, std::min(1.0f, msg->axes[1]));
        _ak_velocity_commands[0] = toggle_eof ? 0.0f : std::max(-_ak_max_vel, std::min(_ak_max_vel, -msg->axes[2]*_ak_max_vel)) * _reduction_numbers[0];
        _ak_velocity_commands[1] = toggle_eof ? 0.0f : std::max(-_ak_max_vel, std::min(_ak_max_vel,  msg->axes[1]*_ak_max_vel)) * _reduction_numbers[1];
        _ak_velocity_commands[2] = toggle_eof ? 0.0f : std::max(-_ak_max_vel, std::min(_ak_max_vel, -msg->axes[5]*_ak_max_vel)) * _reduction_numbers[2];
        _ak_velocity_commands[3] = toggle_eof ? 0.0f : std::max(-_ak_max_vel, std::min(_ak_max_vel,  msg->axes[5]*_ak_max_vel)) * _reduction_numbers[3];
        if      (dxl_cmd >  0.1) _dxl_output_command = static_cast<uint16_t>  (dxl_cmd*_dxl_max_output);
        else if (dxl_cmd < -0.1) _dxl_output_command = static_cast<uint16_t> (-dxl_cmd*_dxl_max_output) | 0b0000001000000000;
        else                     _dxl_output_command = 0;
    }

    void _initializeControlVariables()
    {
        ROS_WARN("Initializing control variables...");
        const double beta = 0.03;
        double control_frequency;
        ros::param::get("control_frequency", control_frequency);
        _control_f = std::min(std::max(1.0, control_frequency), 100.0);
        _decay_constant = 1.0 - exp(-beta*_control_f);
        ROS_WARN("Control variables initialized.");
        ros::param::get("ak60/max_vel", _ak_max_vel);
        _ak_max_vel = std::max(5.0, _ak_max_vel);
        ros::param::get("dxl/max_out", _dxl_max_output);
        _dxl_max_output = std::max(1, std::min(1023, _dxl_max_output));
        ros::param::get("ak60/reductions", _reduction_numbers);
        if (_reduction_numbers.size() != 4) throw std::runtime_error("Invalid AK60 reduction numbers");
        ROS_WARN("AK60 max velocity: %f", _ak_max_vel);
        ROS_WARN("Dynamixel max output: %d", _dxl_max_output);
        ROS_WARN("AK60 reduction numbers: %f, %f, %f, %f", _reduction_numbers[0], _reduction_numbers[1], _reduction_numbers[2], _reduction_numbers[3]);
    }

    void _initializeAKMotors()
    {
        std::vector<int> motor_ids;
        ros::param::get("ak60/ids", motor_ids);
        if (motor_ids.size() != 4) throw std::runtime_error("Invalid motor ids");

        std::string can_interface;
        ros::param::get("ak60/can_interface", can_interface);
        if (can_interface != "can0" && can_interface != "vcan0") throw std::runtime_error("Invalid CAN interface.");

        ROS_WARN("Connecting to AK60 motors...");
        for (std::vector<int>::const_iterator id = motor_ids.begin(); id != motor_ids.end(); ++id)
        {
            _motors.emplace_back(*id);
            _motors.back().connect(can_interface.c_str());
        }
        ROS_WARN("AK60 motors connected!");
    }

    void _initializeDynamixel()
    {   
        // std::string device_name;
        // ros::param::get("dxl/port_name", device_name);
        // ROS_WARN("Device name: %s", device_name.c_str());
        _port_handler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
        _packet_handler = dynamixel::PacketHandler::getPacketHandler(1.0);

        // int dxl_baudrate, dxl_id;
        // double dxl_reduction;
        // ros::param::get("dxl/baudrate", dxl_baudrate);
        // ros::param::get("dxl/id", dxl_id);
        // _dxl_id = std::min(std::max(1, dxl_id), 255);
        ROS_WARN("Connecting to Dynamixel motor...");
        if (!_port_handler->openPort()) throw std::runtime_error("Failed to open the port!");
        if (!_port_handler->setBaudRate(56700)) throw std::runtime_error("Failed to set the baud rate!");
        if (_packet_handler->ping(_port_handler, 1) != COMM_SUCCESS) throw std::runtime_error("Failed to ping the motor!");
        // {
        //     ROS_WARN("Unable to find the designated Dynamixel motor with id %d, searching all IDs.", _dxl_id);
        //     _dxl_id = 1;
        //     for (uint8_t id = 1; id < 255; id++)
        //     {
        //         ROS_WARN("Trying ID: %d", id);
        //         result = _packet_handler->ping(_port_handler, id);
        //         if (result == COMM_SUCCESS)
        //         {
        //             ROS_WARN("Dynamixel motor found with ID: %d", id);
        //             _dxl_id = id;
        //             break;
        //         }
        //     }
        //     if (result != COMM_SUCCESS) throw std::runtime_error("Unable to find the motor.");
        // }
        ROS_WARN("Dynamixel is connected!");
        int result = _packet_handler->write2ByteTxRx(_port_handler, _dxl_id, DXL_ADDR_MX_CW_ANGLE_LIMIT, 0);
        if (result != COMM_SUCCESS) throw std::runtime_error("Failed to set the CW angle limit!");
        result = _packet_handler->write2ByteTxRx(_port_handler, _dxl_id, DXL_ADDR_MX_CCW_ANGLE_LIMIT, 0);
        if (result != COMM_SUCCESS) throw std::runtime_error("Failed to set the CCW angle limit!");
        ROS_WARN("Dynamixel motor initialized.");
    }
};

}; // namespace deimos_control

#endif // H_JOYSTICK_CTL