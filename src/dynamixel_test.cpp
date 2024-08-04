#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

bool initialize_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
  if (!portHandler->openPort())
  {
    std::cerr << "Failed to open the port!" << std::endl;
    return false;
  }
  if (!portHandler->setBaudRate(56700))
  {
    std::cerr << "Failed to set the baud rate!" << std::endl;
    return false;
  }
  uint8_t err;
  int result = packetHandler->ping(portHandler, 1, &err);
  if (result != COMM_SUCCESS)
  {
    std::cerr << "Failed to ping the motor!" << std::endl;
    return false;
  }
  std::cout << "Motor is connected!" << std::endl;
  return true;
}

int main(int argc, char *argv[])
{
    dynamixel::PortHandler *port_handler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
    dynamixel::PacketHandler *packet_handler = dynamixel::PacketHandler::getPacketHandler(1.0);
    if (!initialize_dynamixel(port_handler, packet_handler))
    {
        return 1;
    }
}