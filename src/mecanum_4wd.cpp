#include "mecanum_4wd.h"
#include <array>


//Robot Description
#define NUM_WHEELS 4
#define WHEEL_RADIUS 0.1016f    //8-in wheels
#define lx 0.3092f  //309.2 mm
#define ly 0.225f   //225 mm

//Limits (Limit the max or min speed)
#define LIMIT_VELOCITY_SETPOINT_LINEAR_X 1.5f
#define LIMIT_VELOCITY_SETPOINT_LINEAR_Y 1.5f
#define LIMIT_VELOCITY_SETPOINT_ANGULAR_Z 1.5f


//Arrays of velocities
float wheels_angular_velocity_setpoint[NUM_WHEELS]{0.0, 0.0, 0.0, 0.0};
float wheels_angular_velocity_output[NUM_WHEELS]{0.0, 0.0, 0.0, 0.0};
// float wheels_angular_velocity_setpoint[NUM_WHEELS]{0.0, 0.0, 0.0};
// float wheels_angular_velocity_output[NUM_WHEELS]{0.0, 0.0, 0.0};

SimpleCmdVel mecanum4WD_velocity_setpoint{0.0, 0.0, 0.0};
SimpleCmdVel mecanum4WD_velocity_read{0.0, 0.0, 0.0};



//Motor pins
const MitMotor::MotorType AK_10_REVERSED{-18.0f, 18.0f, 1.0f, -1.0f};
CanMotor * wheel_motors[] = {
    new MitMotor(MitMotor::AK_10, CS_3, INT_3, "Wheel 0", SPI, false),
    new MitMotor(AK_10_REVERSED, CS_4, INT_4, "Wheel 1", SPI, false),
    new MitMotor(AK_10_REVERSED, CS_5, INT_5, "Wheel 2", SPI, false),
    new MitMotor(AK_10_REVERSED, CS_6, INT_6, "Wheel 3", SPI, false)
};

//Interruptioin pins
void(*mecanum4WD_motors_interrupt_handlers[NUM_WHEELS])() = {
    [](){wheel_motors[0]->handleInterrupt();},
    [](){wheel_motors[1]->handleInterrupt();},
    [](){wheel_motors[2]->handleInterrupt();},
    [](){wheel_motors[3]->handleInterrupt();}
};



//----------Control variables and definitions
#define KI 2.0f
float error[NUM_WHEELS]{0.0, 0.0, 0.0, 0.0};
float error_sum[NUM_WHEELS]{0.0, 0.0, 0.0, 0.0};
// float error[NUM_WHEELS]{0.0, 0.0, 0.0};
// float error_sum[NUM_WHEELS]{0.0, 0.0, 0.0};



//----Forward function declarations (emulate "private member functions")
void mecanum4WD_updateInverseKinematics();



//----------------------------------------------------------------------Function definitions-------------------------------------------------------------------------


void SimpleCmdVel::setVelocities(float _linear_x, float _linear_y, float _angular_z){
        linear_x = constrain(_linear_x,   -LIMIT_VELOCITY_SETPOINT_LINEAR_X,  LIMIT_VELOCITY_SETPOINT_LINEAR_X);
        linear_y = constrain(_linear_y,   -LIMIT_VELOCITY_SETPOINT_LINEAR_Y,  LIMIT_VELOCITY_SETPOINT_LINEAR_Y);
        angular_z = constrain(_angular_z, -LIMIT_VELOCITY_SETPOINT_ANGULAR_Z, LIMIT_VELOCITY_SETPOINT_ANGULAR_Z);
        // linear_x = _linear_x;
        // linear_y = _linear_y;
        // angular_z = _angular_z;
}



//Operator overloads to be able to set bits in ErrorMecanum4WD enums.
inline ErrorMecanum4WD operator |(ErrorMecanum4WD a, ErrorMecanum4WD b)
{
    return static_cast<ErrorMecanum4WD>(static_cast<int>(a) | static_cast<int>(b));
}

inline ErrorMecanum4WD operator &(ErrorMecanum4WD a, ErrorMecanum4WD b)
{
    return static_cast<ErrorMecanum4WD>(static_cast<int>(a) & static_cast<int>(b));
}

inline ErrorMecanum4WD& operator |=(ErrorMecanum4WD& a, ErrorMecanum4WD b)
{
    return a = a | b;
}

inline ErrorMecanum4WD& operator |=(ErrorMecanum4WD& a, int b)
{
    return a = a | static_cast<ErrorMecanum4WD>(b);
}

inline ErrorMecanum4WD& operator &=(ErrorMecanum4WD& a, ErrorMecanum4WD b)
{
    return a = a & b;
}



ErrorMecanum4WD mecanum4WD_turnOn()
{
    ErrorMecanum4WD error_status = ErrorMecanum4WD::ERROR_OK;
    for (uint8_t i = 0; i < NUM_WHEELS; i++)
    {
        if (wheel_motors[i]->turnOn() ) {
            Serial.print("Turned on "); Serial.println(wheel_motors[i]->name());
        }
        else {
            error_status |= (1 << i);
            Serial.print("Failed turning on "); Serial.println(wheel_motors[i]->name());
        }
    }   
    return error_status;
}



ErrorMecanum4WD mecanum4WD_turnOff()
{
    ErrorMecanum4WD error_status = ErrorMecanum4WD::ERROR_OK;
    for (uint8_t i = 0; i < NUM_WHEELS; i++)
    {
        if (wheel_motors[i]->turnOff() ) {
            Serial.print("Turned off "); Serial.println(wheel_motors[i]->name());
        }
        else {
            error_status |= (1 << i);
            Serial.print("Failed turning off "); Serial.println(wheel_motors[i]->name());
        }
    } 
    return error_status;
}



ErrorMecanum4WD mecanum4WD_initialize()
{
    ErrorMecanum4WD error_status = ErrorMecanum4WD::ERROR_OK;
    for (uint8_t i = 0; i < NUM_WHEELS; i++) 
    {
        //MCP2515 initialization (SPI)
        bool was_response_received;
        unsigned long t_ini_ms = millis();
        while (!(was_response_received = wheel_motors[i]->initialize()) and (millis() - t_ini_ms) < 100){}    //Retry during 100 ms per motor.
        if(!was_response_received){
            error_status |= ErrorMecanum4WD::ERROR_MCP2515;
            Serial.print("Unable to initialize "); Serial.print(wheel_motors[i]->name()); Serial.print(" MCP2515");
        }
        else {
            Serial.print("Successfully initialized "); Serial.print(wheel_motors[i]->name()); Serial.println(" MCP2515");
        }
    }
    for (uint8_t i = 0; i < NUM_WHEELS; i++)
    {
        if(!wheel_motors[i]->turnOn())
        {
            error_status |= (1 << i);
            Serial.print("Unable to turn on "); Serial.println(wheel_motors[i]->name());
        }
        else {
            Serial.print("Successfully turned on: "); Serial.println(wheel_motors[i]->name());
        }
    }
    return error_status;
}



/**
 * @brief Stops robot wheels as fast as possible. No sigmoid or acceleration ramp, the velocity is set directly to zero for each motor.
 * Blocking: This function returns until the robot stops or the timeout is exceeded. 
 * 
 * @param timeout_ms Timeout in milliseconds. If this timeout is exceeded, ERROR_GOAL_NOT_ACHIEVED bit is set in the returned error code and
 * the function returns. 
 * @return ErrorMecanum4WD error code.
 */
ErrorMecanum4WD mecanum4WD_stopRobotBlocking(unsigned long timeout_ms)
{
    ErrorMecanum4WD error_status = ErrorMecanum4WD::ERROR_OK;
    bool is_robot_moving = true;
    unsigned long t_ini_ms = millis();
    while(is_robot_moving and ((millis() - t_ini_ms) < timeout_ms) )
    {
        //Send velocity zero to each motor. 
        for (uint8_t i = 0; i < NUM_WHEELS; i++) {
            if (!wheel_motors[i]->setVelocity(0.0, 3.0)){
                error_status |= (1 << i);
            }
        }
        for (uint8_t i = 0; i < NUM_WHEELS; i++) {
            if(!wheel_motors[i]->readMotorResponse(2000)){
                error_status |= (1 << i);
            }
        }
        //Update robot velocities (linear x, linear y, angular z)
        mecanum4WD_updateInverseKinematics();
        //From inverse kinematics result, determine if the robot is stopped. 
        is_robot_moving = (fabs(mecanum4WD_velocity_read.linear_x) > 0.05 
                        or fabs(mecanum4WD_velocity_read.linear_y) > 0.05 
                        or fabs(mecanum4WD_velocity_read.angular_z) > 0.05 );
    }
    if (is_robot_moving) error_status |= ErrorMecanum4WD::ERROR_GOAL_NOT_ACHIEVED;
    return error_status;
}




void    mecanum4WD_enableMotorsAutoMode()
{
    for (uint8_t i = 0; i < NUM_WHEELS; i++)
    {
        Serial.print("Starting auto mode for:"); Serial.println(wheel_motors[i]->name());
        wheel_motors[i]->startAutoMode(mecanum4WD_motors_interrupt_handlers[i]);
    }
}



void mecanum4WD_disableMotorsAutoMode()
{
    for (uint8_t i = 0; i < NUM_WHEELS; i++)
    {
        Serial.print("Disabling auto mode for:"); Serial.println(wheel_motors[i]->name());
        wheel_motors[i]->stopAutoMode();
    }
}



void mecanum4WD_updateVelControl(float elapsed_time_seconds)
{
    //digitalWrite(AUX_PIN_1,!digitalRead(AUX_PIN_1));  //Toggle pin to analize periodicity with the logic analyzer.



    //Constrain mecanum4wd velocity setpoints to reasonable limits. (Built-in constrain() function doesn't work since the setpoint is std::atomic<float>)
    //Most likely redudant, but since SimpleCmdVel members can be assigned directly, we must be sure the setpoints are within the limits. 

    if (mecanum4WD_velocity_setpoint.linear_x > LIMIT_VELOCITY_SETPOINT_LINEAR_X) mecanum4WD_velocity_setpoint.linear_x = LIMIT_VELOCITY_SETPOINT_LINEAR_X;
    else if (mecanum4WD_velocity_setpoint.linear_x < -LIMIT_VELOCITY_SETPOINT_LINEAR_X) mecanum4WD_velocity_setpoint.linear_x = -LIMIT_VELOCITY_SETPOINT_LINEAR_X;

    if (mecanum4WD_velocity_setpoint.linear_y > LIMIT_VELOCITY_SETPOINT_LINEAR_Y) mecanum4WD_velocity_setpoint.linear_y = LIMIT_VELOCITY_SETPOINT_LINEAR_Y;
    else if (mecanum4WD_velocity_setpoint.linear_y < -LIMIT_VELOCITY_SETPOINT_LINEAR_Y) mecanum4WD_velocity_setpoint.linear_y = -LIMIT_VELOCITY_SETPOINT_LINEAR_Y;

    if (mecanum4WD_velocity_setpoint.angular_z > LIMIT_VELOCITY_SETPOINT_ANGULAR_Z) mecanum4WD_velocity_setpoint.angular_z = LIMIT_VELOCITY_SETPOINT_ANGULAR_Z;
    else if (mecanum4WD_velocity_setpoint.angular_z < -LIMIT_VELOCITY_SETPOINT_ANGULAR_Z) mecanum4WD_velocity_setpoint.angular_z = -LIMIT_VELOCITY_SETPOINT_ANGULAR_Z;




    //Kinematic Model (Compute the desired individual wheels angular velocity from the mecanum4wd desired velocity)
    wheels_angular_velocity_setpoint[0] = (1 / WHEEL_RADIUS) * (mecanum4WD_velocity_setpoint.linear_x - mecanum4WD_velocity_setpoint.linear_y - (lx + ly) * mecanum4WD_velocity_setpoint.angular_z);
    wheels_angular_velocity_setpoint[1] = (1 / WHEEL_RADIUS) * (mecanum4WD_velocity_setpoint.linear_x + mecanum4WD_velocity_setpoint.linear_y + (lx + ly) * mecanum4WD_velocity_setpoint.angular_z);
    
    wheels_angular_velocity_setpoint[2] = (1 / WHEEL_RADIUS) * (mecanum4WD_velocity_setpoint.linear_x + mecanum4WD_velocity_setpoint.linear_y - (lx + ly) * mecanum4WD_velocity_setpoint.angular_z);
    wheels_angular_velocity_setpoint[3] = (1 / WHEEL_RADIUS) * (mecanum4WD_velocity_setpoint.linear_x - mecanum4WD_velocity_setpoint.linear_y + (lx + ly) * mecanum4WD_velocity_setpoint.angular_z);
    
    // wheels_angular_velocity_setpoint[2] = (1 / WHEEL_RADIUS) * (mecanum4WD_velocity_setpoint.linear_x - mecanum4WD_velocity_setpoint.linear_y + (lx + ly) * mecanum4WD_velocity_setpoint.angular_z);



    //Wheel velocity control
    for(uint8_t i = 0; i < NUM_WHEELS; i++)
    {
        float wheel_i_velocity = wheel_motors[i]->velocity();

        //Filter non-zero velocity reported from the motor
        if (fabs(wheel_i_velocity) < 0.08) wheel_i_velocity = 0;


        //Option A: Switching between closed-loop controller and open-loop controller.
        if(fabs(wheels_angular_velocity_setpoint[i]) < 0.30 and fabs(wheels_angular_velocity_output[i]) < 0.30) {
            error[i] =  wheels_angular_velocity_setpoint[i] - wheels_angular_velocity_output[i];
        }
        else {
            error[i] =  wheels_angular_velocity_setpoint[i] - wheel_i_velocity;
        }

        //Option B: closed-loop controller
        //error[i] =  wheels_angular_velocity_setpoint[i] - wheel_i_velocity;

        //Option C: open-loop controller
        //error[i] =  wheels_angular_velocity_setpoint[i] - wheels_angular_velocity_output[i];

        error_sum[i] += error[i] * elapsed_time_seconds;
        wheels_angular_velocity_output[i] = (error_sum[i] * KI);

        //Saturation
        wheels_angular_velocity_output[i] = constrain(wheels_angular_velocity_output[i],-15.0, 15.0);

        //Perform Inverse Kinematics
        mecanum4WD_updateInverseKinematics();

        #if defined (DONTACTUATEMECANUM)
        wheels_angular_velocity_output[i] = 0.0f;
        #endif

        if (!wheel_motors[i]->setVelocity(wheels_angular_velocity_output[i], 5.0))
        {
            Serial.print("Message NOT Sent to "); Serial.println(wheel_motors[i]->name());
        }
    }
}


// Inverse Kinematics Algorithm
void mecanum4WD_updateInverseKinematics()
{
    mecanum4WD_velocity_read.linear_x = (   wheel_motors[0]->velocity() + wheel_motors[1]->velocity() + wheel_motors[2]->velocity() + wheel_motors[3]->velocity() ) * WHEEL_RADIUS / 4.0f;
    mecanum4WD_velocity_read.linear_y = ( - wheel_motors[0]->velocity() + wheel_motors[1]->velocity() + wheel_motors[2]->velocity() - wheel_motors[3]->velocity() ) * WHEEL_RADIUS / 4.0f;
    mecanum4WD_velocity_read.angular_z =( - wheel_motors[0]->velocity() + wheel_motors[1]->velocity() - wheel_motors[2]->velocity() + wheel_motors[3]->velocity() ) * WHEEL_RADIUS / (4.0f * (lx + ly));
   
    // mecanum4WD_velocity_read.linear_x = (   wheel_motors[0]->velocity() + wheel_motors[1]->velocity() + wheel_motors[2]->velocity() + wheel_motors[2]->velocity() ) * WHEEL_RADIUS / 4.0f;
    // mecanum4WD_velocity_read.linear_y = ( - wheel_motors[0]->velocity() + wheel_motors[1]->velocity() + wheel_motors[2]->velocity() - wheel_motors[2]->velocity() ) * WHEEL_RADIUS / 4.0f;
    // mecanum4WD_velocity_read.angular_z =( - wheel_motors[0]->velocity() + wheel_motors[1]->velocity() - wheel_motors[2]->velocity() + wheel_motors[2]->velocity() ) * WHEEL_RADIUS / (4.0f * (lx + ly));

}


// Debug wheels velocities
void mecanum4WD_debugPrintStatus()
{
    Serial.print("Setpoints  : Vel x: "); Serial.print(mecanum4WD_velocity_setpoint.linear_x);
    Serial.print("\tVel y: "); Serial.print(mecanum4WD_velocity_setpoint.linear_y);
    Serial.print("\tVel z: "); Serial.println(mecanum4WD_velocity_setpoint.angular_z);

    Serial.print("Actual vals: Vel x: "); Serial.print(mecanum4WD_velocity_read.linear_x);
    Serial.print("\tVel y: "); Serial.print(mecanum4WD_velocity_read.linear_y);
    Serial.print("\tVel z: "); Serial.println(mecanum4WD_velocity_read.angular_z);


    for (uint8_t i = 0; i < NUM_WHEELS; i++)
    {
        Serial.print("\t    ");        Serial.println(wheel_motors[i]->name());         
        Serial.print("\tSetpoint: ");  Serial.println(wheels_angular_velocity_setpoint[i]);
        Serial.print("\tOut Vel:  ");  Serial.println(wheels_angular_velocity_output[i]);
        Serial.print("\tVel read: "); Serial.println(wheel_motors[i]->velocity(), 4);
        Serial.println();
    }

    Serial.print("\n");
}

