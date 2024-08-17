#ifndef JOYSTICK_CTL_HPP
#define JOYSTICK_CTL_HPP

#include <cmath>
#include <thread>
#include <chrono>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
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
    _nh(),
    _dxl_id(1),
    _dxl_max_output(5.0),
    _dxl_output_command(0),
    _reduction_numbers(4, 0.0),
    _ak_max_vel(5.0),
    _ak_velocity_commands(4, 0.0)
    {
      _joy_sub = _nh.subscribe<sensor_msgs::Joy>("joy", 1, &DeimosController::_joyCallback, this);
    }

    void run()
    {
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
    std::string _can_interface;
    std::vector<double> _ak_velocity_commands;
    std::vector<double> _reduction_numbers;
    TMotor::AKManager _motors[4];
    dynamixel::PortHandler *_port_handler;
    dynamixel::PacketHandler *_packet_handler;

    void _controlLoop()
    {
      ROS_INFO("Starting control loop at %f Hz frequency...", _control_f);
      ros::Rate loop_frequency(_control_f);
      while (ros::ok())
      {
        loop_frequency.sleep();

        ros::spinOnce();

        /* Read motor states. */
        // to-do

        /* Write to motors. */
        for (size_t i = 0; i < 4; ++i)
        {
          try
          {
            _motors[i].sendVelocity(_ak_velocity_commands[i]);
          }
          catch(TMotor::CANSocketException& e)
          {
            ROS_WARN("Motor %d unable to send command: %s", _motors[i].getMotorID(), e.what());
            connectMotor(i);
          }
        }
        _packet_handler->write2ByteTxRx(_port_handler, _dxl_id, DXL_ADDR_MX_MOVING_SPEED, _dxl_output_command);
      }
    }

    void _joyCallback(const sensor_msgs::JoyConstPtr &msg)
    {
      if (msg->axes.size() < 6 || msg->buttons.size() < 12)
        return;

      const bool toggle_eof = msg->buttons[0];
      const float dxl_cmd = std::max(-1.0f, std::min(1.0f, msg->axes[2]));
      const float eof_cmd = std::max(-1.0f, std::min(1.0f, msg->axes[1]));

      if (toggle_eof)
      {
        _ak_velocity_commands[0] = 0.0f;
        _ak_velocity_commands[1] = 0.0f;
        _ak_velocity_commands[2] = 0.0f;
        _ak_velocity_commands[3] = 0.0f;
      }
      else
      {
        _ak_velocity_commands[0] = std::max(-_ak_max_vel, std::min(_ak_max_vel, -msg->axes[2] * _ak_max_vel)) * _reduction_numbers[0];
        _ak_velocity_commands[1] = std::max(-_ak_max_vel, std::min(_ak_max_vel, msg->axes[1] * _ak_max_vel)) * _reduction_numbers[1];
        _ak_velocity_commands[2] = std::max(-_ak_max_vel, std::min(_ak_max_vel, -msg->axes[5] * _ak_max_vel)) * _reduction_numbers[2];
        _ak_velocity_commands[3] = std::max(-_ak_max_vel, std::min(_ak_max_vel, msg->axes[5] * _ak_max_vel)) * _reduction_numbers[3];
        ROS_INFO("AK60 velocities: %f, %f, %f, %f", _ak_velocity_commands[0], _ak_velocity_commands[1], _ak_velocity_commands[2], _ak_velocity_commands[3]);
        ROS_INFO("Axes message: %f", msg->axes[5]);
      }
      
      if (dxl_cmd > 0.1)
      {
        _dxl_output_command = static_cast<uint16_t> ( dxl_cmd * _dxl_max_output);
      }
      else if (dxl_cmd < -0.1)
      {
        _dxl_output_command = static_cast<uint16_t> (-dxl_cmd * _dxl_max_output) | (1 << 10);
      }
      else
      {
        _dxl_output_command = 0;
      }
    }

    void _initializeControlVariables()
  {
      ROS_INFO("Initializing control variables...");
      const double beta = 0.03;
      double control_frequency;
      ros::param::get("control_frequency", control_frequency);
      _control_f = std::min(std::max(1.0, control_frequency), 100.0);
      _decay_constant = 1.0 - exp(-beta * _control_f);
      ROS_INFO("Control variables initialized.");
      ros::param::get("ak60/max_vel", _ak_max_vel);
      _ak_max_vel = std::max(5.0, _ak_max_vel);
      ros::param::get("dxl/max_out", _dxl_max_output);
      _dxl_max_output = std::max(1, std::min(1023, _dxl_max_output));
      ros::param::get("ak60/reductions", _reduction_numbers);
      if (_reduction_numbers.size() != 4)
      {
        throw std::runtime_error("Invalid AK60 reduction numbers");
      }
      ROS_INFO("AK60 max velocity: %f", _ak_max_vel);
      ROS_INFO("Dynamixel max output: %d", _dxl_max_output);
      ROS_INFO("AK60 reduction numbers: %f, %f, %f, %f", _reduction_numbers[0], _reduction_numbers[1], _reduction_numbers[2], _reduction_numbers[3]);
    }

    void _initializeAKMotors()
    {
      std::vector<int> motor_ids;
      ros::param::get("ak60/ids", motor_ids);
      if (motor_ids.size() != 4)
      {
        throw std::runtime_error("Invalid motor ids");
      }
      for (size_t i = 0; i < 4; i++)
      {
        _motors->setMotorID(motor_ids[i]);
      }
      
      std::string can_interface;
      ros::param::get("ak60/can_interface", can_interface);
      if (can_interface != "can0" && can_interface != "vcan0")
      {
        throw std::runtime_error("Invalid CAN interface.");
      }
      _can_interface = can_interface;
  
      ROS_INFO("Connecting to AK60 motors...");
      connectMotors();
      ROS_INFO("AK60 motors connected!");
    }

    void _initializeDynamixel()
    {
      std::string device_name;
      ros::param::get("dxl/port_name", device_name);
      ROS_INFO("Device name: %s", device_name.c_str());
      _port_handler = dynamixel::PortHandler::getPortHandler(device_name.c_str());
      _packet_handler = dynamixel::PacketHandler::getPacketHandler(1.0);

      int dxl_baudrate, dxl_id, result(-1);
      double dxl_reduction;
      ros::param::get("dxl/baudrate", dxl_baudrate);
      ros::param::get("dxl/id", dxl_id);
      _dxl_id = std::min(std::max(1, dxl_id), 255);
      ROS_INFO("Connecting to Dynamixel motor..."); 
      if (!_port_handler->openPort())
      {
        throw std::runtime_error("Failed to open the port!");
      }
      if (!_port_handler->setBaudRate(56700))
      {
        throw std::runtime_error("Failed to set the baud rate!");
      }
      if ((result = _packet_handler->ping(_port_handler, 1)) != COMM_SUCCESS)
      {
        throw std::runtime_error("Failed to ping the motor!");
      }
      
      while (result != COMM_SUCCESS)
      {
        ROS_INFO("Unable to find the designated Dynamixel motor with id %d, searching all IDs.", _dxl_id);
        _dxl_id = 1;
        for (uint8_t id = 1; id < 255 && result != COMM_SUCCESS; id++)
        {
          ROS_INFO("Trying ID: %d", id);
          result = _packet_handler->ping(_port_handler, id);
          if (result == COMM_SUCCESS)
          {
            ROS_INFO("Dynamixel motor found with ID: %d", id);
            _dxl_id = id;
          }
        }
        if (result != COMM_SUCCESS)
        {
          throw std::runtime_error("Unable to find the motor.");
        }
      }

      ROS_INFO("Dynamixel is connected!");
      result = _packet_handler->write2ByteTxRx(_port_handler, _dxl_id, DXL_ADDR_MX_CW_ANGLE_LIMIT, 0);
      if (result != COMM_SUCCESS)
      {
        throw std::runtime_error("Failed to set the CW angle limit!");
      }
      result = _packet_handler->write2ByteTxRx(_port_handler, _dxl_id, DXL_ADDR_MX_CCW_ANGLE_LIMIT, 0);
      if (result != COMM_SUCCESS)
      {
        throw std::runtime_error("Failed to set the CCW angle limit!");
      }
      ROS_INFO("Dynamixel motor initialized.");
    }

    void connectMotor(size_t i)
    {
      ROS_INFO("Connecting to motor %d through the CAN interface: %s", _motors[i].getMotorID(), _can_interface.c_str());
      bool connected = false;
      while (!connected)
      {
        try
        {
          _motors[i].connect(_can_interface.c_str());
          connected = true;
        }
        catch(TMotor::CANSocketException& e)
        {
          ROS_WARN("Motor %d unable to reconnect: %s, retrying...", _motors[i].getMotorID(), e.what());
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
      }
      
    }

    void connectMotors()
    {
      ROS_INFO("Connecting to motors through the CAN interface.");
      for (size_t i = 0; i < 4; i++)
      {
        try
        {
          _motors[i].connect(_can_interface.c_str());
        }
        catch(TMotor::CANSocketException& e)
        {
          ROS_WARN("Motor %d unable to reconnect: %s", _motors[i].getMotorID(), e.what());
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          i--; // retry
        }
      }
    }

  }; // class DeimosController
}; // namespace deimos_control

#endif // JOYSTICK_CTL_HPP