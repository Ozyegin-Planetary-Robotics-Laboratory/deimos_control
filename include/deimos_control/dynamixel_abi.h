#ifndef DYNAMIXEL_STATS_H
#define DYNAMIXEL_STATS_H

#define DXL_STDIN_FILENO                    0
#define DXL_PROTOCOL_VERSION                1.0

#define DXL_ADDR_MX_CW_ANGLE_LIMIT          6
#define DXL_ADDR_MX_CCW_ANGLE_LIMIT         8
#define DXL_ADDR_MX_TORQUE_ENABLE           24
#define DXL_ADDR_MX_GOAL_POSITION           30
#define DXL_ADDR_MX_MOVING_SPEED            32
#define DXL_ADDR_MX_PRESENT_POSITION        36
#define DXL_ADDR_MX_PRESENT_SPEED           38
#define DXL_ADDR_MX_PRESENT_LOAD            40
#define DXL_ADDR_MX_PRESENT_VOLTAGE         42
#define DXL_ADDR_MX_PRESENT_TEMPERATURE     43
#define DXL_ADDR_MX_MOVING                  46

#define DXL_LEN_MX_GOAL_POSITION            2
#define DXL_LEN_MX_PRESENT_POSITION         2
#define DXL_LEN_MX_MOVING_SPEED             2
#define DXL_LEN_MX_MOVING                   1

#define DXL_TORQUE_ENABLE                   1
#define DXL_TORQUE_DISABLE                  0 
#define DXL_MOVING_STATUS_THRESHOLD     10

#endif // DYNAMIXEL_STATS_H