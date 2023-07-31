#pragma once

#include "Arduino.h"
#include <atomic>
#include "pin_definitions.h"
#include "MitMotor.h"


enum /*class*/ ErrorMecanum4WD : uint8_t {
    ERROR_OK = 0x00,
    ERROR_WHEEL_0 = 0x01,
    ERROR_WHEEL_1 = 0x02,
    ERROR_WHEEL_2 = 0x04,
    ERROR_WHEEL_3 = 0x08,
    ERROR_MCP2515 = 0x10,
    ERROR_GOAL_NOT_ACHIEVED = 0X20
};

struct SimpleCmdVel{
    volatile std::atomic<float> linear_x;
    volatile std::atomic<float> linear_y;
    volatile std::atomic<float> angular_z;
    SimpleCmdVel(float _linear_x, float _linear_y, float _angular_z) : linear_x{_linear_x}, linear_y{_linear_y}, angular_z{_angular_z}{}
    void setVelocities(float _linear_x, float _linear_y, float _angular_z);
};

void mecanum4WD_enableMotorsAutoMode();
void mecanum4WD_disableMotorsAutoMode();
void mecanum4WD_updateVelControl(float elapsed_time_seconds);
ErrorMecanum4WD mecanum4WD_initialize();
ErrorMecanum4WD mecanum4WD_stopRobotBlocking(unsigned long timeout_ms);
ErrorMecanum4WD mecanum4WD_turnOn();
ErrorMecanum4WD mecanum4WD_turnOff();
void mecanum4WD_debugPrintStatus();


extern SimpleCmdVel mecanum4WD_velocity_setpoint;
extern SimpleCmdVel mecanum4WD_velocity_read;