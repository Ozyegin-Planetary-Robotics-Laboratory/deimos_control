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
    int _dxl_id;
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
        const double beta = 0.03;
        double control_frequency;
        ros::param::get("control_frequency", control_frequency);
        _control_f = std::min(std::max(1.0, control_frequency), 100.0);
        _decay_constant = 1.0 - exp(-beta*_control_f);

        ros::param::get("ak60/max_vel", _ak_max_vel);
        _ak_max_vel = std::max(5.0, _ak_max_vel);
        ros::param::get("dyxl/max_out", _dxl_max_output);
        _dxl_max_output = std::max(1, std::min(1023, _dxl_max_output));
        ros::param::get("ak60/reductions", _reduction_numbers);
        if (_reduction_numbers.size() != 4) throw std::runtime_error("Invalid AK60 reduction numbers");
    }

    void _initializeAKMotors()
    {
        std::vector<int> motor_ids;
        ros::param::get("ak60/ids", motor_ids);
        if (motor_ids.size() != 4) throw std::runtime_error("Invalid motor ids");

        std::string can_interface;
        ros::param::get("ak60/can_interface", can_interface);
        if (can_interface != "can0" && can_interface != "vcan0") throw std::runtime_error("Invalid CAN interface.");

        for (std::vector<int>::const_iterator id = motor_ids.begin(); id != motor_ids.end(); ++id)
        {
            _motors.emplace_back(*id);
            _motors.back().connect(can_interface.c_str());
        }
    }

    void _initializeDynamixel()
    {   
        std::string device_name;
        ros::param::get("dxl/device_name", device_name);
        _port_handler = dynamixel::PortHandler::getPortHandler(device_name.c_str());
        _packet_handler = dynamixel::PacketHandler::getPacketHandler(DXL_PROTOCOL_VERSION);

        int dxl_baudrate;
        double dxl_reduction;
        ros::param::get("dxl/baudrate", dxl_baudrate);
        ros::param::get("dxl/id", _dxl_id);
        if (!_port_handler->openPort()) throw std::runtime_error("Failed to open the port!");
        if (!_port_handler->setBaudRate(dxl_baudrate)) throw std::runtime_error("Failed to set the baud rate!");
        uint8_t err;
        int result = _packet_handler->ping(_port_handler, std::min(std::max(0, _dxl_id), 255), &err);
        if (result != COMM_SUCCESS) throw std::runtime_error("Failed to ping the motor!");
        result = _packet_handler->write2ByteTxRx(_port_handler, _dxl_id, DXL_ADDR_MX_CW_ANGLE_LIMIT, 0, &err);
        if (result != COMM_SUCCESS) throw std::runtime_error("Failed to set the CW angle limit!");
        result = _packet_handler->write2ByteTxRx(_port_handler, _dxl_id, DXL_ADDR_MX_CCW_ANGLE_LIMIT, 0, &err);
        if (result != COMM_SUCCESS) throw std::runtime_error("Failed to set the CCW angle limit!");
    }
};

}; // namespace deimos_control

#endif // H_JOYSTICK_CTL