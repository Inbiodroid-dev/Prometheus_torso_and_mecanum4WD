/*
 Inbiodroid MCU UDP socket teleoperation state machine:
 This program receives UDP datagrams, containing either SM (state machine) commands
 or (position) control input data, and acts accordingly based upon current state and
 of course, the nature of the module to be controlled. This implementation file focuses
 on the code executed for every command received.

 created 23 Sep 2022
 by Inbiodroid technical team.

 This code is property of Inbiodroid.
 */

// #include <atomic>  // do you need to make your setpoints variables atomic?
//Libraries
#include "StateMachine.h"
#include "PrometheusTeleoperation-MotorsSetup.h"
#include "mecanum_4wd.h"

//PS4 commands, they will be replaced by the pedalboard
#define CMD_UP 1.01f
#define CMD_DOWN 2.01f
#define CMD_LEFT 1.10f
#define CMD_RIGHT 1.20f
#define CMD_CLW 1.03f
#define CMD_CCLW 1.02f
#define CMD_STOP 0.00f



// IMPORTANT: make sure to set the is_outbound_data_ready flag to 'true' everytime the control
// interruption is run; we do not send the tx buffer regardless, lest we flood the network
// unnecesaryly.

//Global variables
int cont = 0;

constexpr float PERIOD_USEC = 1000;
constexpr float T = PERIOD_USEC / 1000000.0;
IntervalTimer myTimer;

//Call interrupts
void periodicInterruptCallback()
{
    //-----Torso
    controlMotors(T);

    //-----Mecanum4WD
    mecanum4WD_updateVelControl(T);
    is_outbound_data_ready = true;
}

/**
 * @brief DISCONNECTED->CONNECTED transition. This function should implement
 * such transition by validating the UDP communications.
 *
 * @return uint8_t StateMachineError code.
 */
uint8_t connect()
{
    // Torso: No action needed
    // Mecanum4WD: No action needed
    // mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, 0.0, 0.0);
    mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, 0.0);
    return (uint8_t)StateMachineError::TRANSITION_OK;
}

/**
 * @brief CONNECTED->VALIDATED transition.This function is intended to validate
 * the embedded systems and actuators (motors).
 * .
 *
 * @return uint8_t StateMachineError code.
 */
uint8_t validate()
{
    //-----Torso:
    uint8_t torso_failed_motors_mask = 0x00;
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        // MCP2515 initialization (SPI)
        bool was_response_received;
        unsigned long t_ini_ms = millis();
        while (!(was_response_received = motors[i]->initialize()) and (millis() - t_ini_ms) < 100) // Retry during 100 ms per motor.
        {
            Debug("Retrying to initialize ");
            Serial.print(motors[i]->name());
            Serial.print(" MCP2515");
        }
        if (!was_response_received)
        {
            torso_failed_motors_mask |= (1 << i);
        }
        // Ensure initial torque is 0.0
        motors[0]->setTorque(0.0, 2000);
        motors[1]->setTorque(0.0, 2000);
        motors[2]->setTorque(0.0, 2000);
        // Motor turn on (CAN)
        if (!motors[i]->turnOn())
        {
            torso_failed_motors_mask |= (1 << i);
            Debug("Not able to turn on ");
            Debugln(motors[i]->name());
            Debugln();
        }
        Serial.print("\tPosition: ");
        Serial.print(motors[i]->position(), 4);
        Serial.print("\tTorque: ");
        Serial.print(motors[i]->torque(), 4);
        Serial.print("\tVelocity: ");
        Serial.println(motors[i]->velocity(), 4);
    }

    //-----Mecanum4WD
    ErrorMecanum4WD mecanum_status = mecanum4WD_initialize();

    // Return transition result
    StateMachineError validation_result = StateMachineError::TRANSITION_OK;
    if (torso_failed_motors_mask != 0 or mecanum_status != ErrorMecanum4WD::ERROR_OK)
    {
        Debug("Resultados inicialización torso: ");
        DebugPln(torso_failed_motors_mask, BIN);
        Debugln();
        Debug("Resultados inicialización mecanum4wd: ");
        DebugPln(mecanum_status, BIN);
        Debugln();
        Debugln("\n\nERROR IN VALIDATION.\n\n");
        validation_result = StateMachineError::TRANSITION_ERROR;
    }
    else
    {
        Debugln("\n\nSUCCESSFUL VALIDATION.\n\n");
    }
    return (uint8_t)validation_result;
}

/**
 * @brief VALIDATED->CALIBRATED transition. This function should perform the
 * required actions to calibrate the module controlled.
 *
 * @return uint8_t StateMachineError code.
 */
uint8_t calibrate()
{
    //-----Torso
    // Set the zero position reference of the torso motors
    uint8_t torso_failed_motors_mask = 0x00;
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        Serial.print(motors[i]->name());
        // if (!motors[i]->setCurrentPositionAsZero())
        // {
        //     torso_failed_motors_mask |= (1 << i);
        //     Serial.println(":\tFailed setting current position as origin");
        // }
        Serial.print("\tPosition: ");
        Serial.print(motors[i]->position(), 4);
        Serial.print("\tTorque: ");
        Serial.print(motors[i]->torque(), 4);
        Serial.print("\tVelocity: ");
        Serial.println(motors[i]->velocity(), 4);
    }

    //-----Mecanum4WD
    // Stop mecanum4WD robot.
    ErrorMecanum4WD is_mecanum4WD_stopped = mecanum4WD_stopRobotBlocking(2000);

    // Return transition result
    StateMachineError calibration_result = StateMachineError::TRANSITION_OK;
    if (torso_failed_motors_mask != 0 or is_mecanum4WD_stopped != ErrorMecanum4WD::ERROR_OK)
    {
        Debug("Resultados calibracion torso: ");
        DebugPln(torso_failed_motors_mask, BIN);
        Debugln();
        Debug("Resultados calibracion mecanum4wd: ");
        DebugPln(is_mecanum4WD_stopped, BIN);
        Debugln();
        Debugln("\n\nERROR IN CALIBRATION.\n\n");
        calibration_result = StateMachineError::TRANSITION_ERROR;
    }
    else
    {
        Debugln("\n\nSUCCESSFUL CALIBRATION.\n\n");
    }
    return (uint8_t)calibration_result;
}

/**
 * @brief CALIBRATED->HOME transition. This function should perform the control
 * required to reach the home secure position from which engage in teleoperation
 * is possible.
 *
 * @return uint8_t StateMachineError code.
 */
uint8_t gotohome()
{
    //-----Torso
    // Setpoints set to home (0.0f)
    Beta = 0.0;
    Gamma = 0.0;
    // Enable torso motors auto mode.
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        theta[i] = -motors[i]->position();
        Serial.print("Starting auto mode for:");
        Serial.println(motors[i]->name());
        motors[i]->startAutoMode(interrupt_handlers[i]);
    }

    //-----Mecanum4WD
    // Setpoints set to home (0.0f)
    // mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, 0.0, 0.0);
    mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, 0.0);
    // Enable mecanum4WD motors auto mode.
    mecanum4WD_enableMotorsAutoMode();

    //-----Torso and Mecanum4WD
    // Enable periodic interrupt.
    myTimer.begin(periodicInterruptCallback, PERIOD_USEC);
    myTimer.priority(200);

    return (uint8_t)StateMachineError::TRANSITION_OK;
}

/**
 * @brief HOME->ENGAGED transition. Ensure here that the transition to engage
 * is clean and safe. After doing this, the data processing/consumption is started.
 *
 * @return uint8_t StateMachineError code.
 */
uint8_t engage()
{
    // System's periodic interrupt (that updates Torso and Mecanum4wd control law) was enabled in calibrate() and should be still active.
    // Setpoints get loaded in updateIncomingData(). The state machine calls it when new data is received while engaged.
    //-----Torso: No further action needed.
    //-----Mecanum4WD: No further action needed.
    return (uint8_t)StateMachineError::TRANSITION_OK;
}

/**
 * @brief ENGAGED->DISENGAGED transition. This function stops the data consumption
 * and halts the robot.
 *
 * @return uint8_t StateMachineError code.
 */
uint8_t disengage()
{
    // System's periodic interrupt (that updates Torso and Mecanum4wd control law) was enabled in calibrate() and should be still active.
    // Set control law setpoint.

    //-----Torso
    // The torso must keep its last setpoints in disengage (pause).

    //-----Mecanum4WD
    // Mecanum4wd must stop in disengage.
    // mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, 0.0, 0.0);
    mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, 0.0);

    return (uint8_t)StateMachineError::TRANSITION_OK;
}

/**
 * @brief DISENGAGED->HOME transition. This function should be performed whenever
 * coming from the disengaged state to go back to home.
 *
 * @return uint8_t StateMachineError code.
 */
uint8_t gobacktohome()
{
    // System's periodic interrupt (that updates Torso and Mecanum4wd control law) was enabled in calibrate() and should be still active.
    // Set control law setpoint.
    //-----Torso
    Beta = 0.0;
    Gamma = 0.0;

    //-----Mecanum4WD
    // mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, 0.0, 0.0);
    mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, 0.0);
    return (uint8_t)StateMachineError::TRANSITION_OK;
}

/**
 * @brief HOME->DISCONNECTED transition. This function is to finish exit the escenario.
 *
 * @return uint8_t StateMachineError code.
 */
uint8_t disconnect()
{
    //-----Torso and Mecanum4WD
    // Disable periodic interrupt.
    myTimer.end();

    //-----Torso
    Debug("\nDisabling Motor");
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        theta[i] = 0.0f;
        tau[i] = 0.0f;
        if (motors[i]->turnOff())
        {
            Serial.print("Turned off ");
            Serial.println(motors[i]->name());
        }
        else
        {
            Debug("Failed turning off ");
            Debugln(motors[i]->name());
        }
    }

    //-----Mecanum4WD
    mecanum4WD_stopRobotBlocking(2000);
    mecanum4WD_turnOff();

    return (uint8_t)StateMachineError::TRANSITION_OK;
}

/**
 * @brief DISCONNECTED->POWEREDOFF transition. This function power down the robot.
 *
 * @return uint8_t StateMachineError code.
 */
uint8_t poweroff()
{
    return (uint8_t)StateMachineError::TRANSITION_OK;
}

/**
 * @brief This function is intended to update the input data coming into the rx buffer
 *
 * @return unint8_t
 */
void updateIncomingData()
{
    // Torso
    // #if !defined(TORSO_REGULATION_TO_ZERO)
    //     Beta = p2pComm._rxDatagram.payload.Q[0];
    //     Gamma = p2pComm._rxDatagram.payload.Q[1];
    // #endif

    // Mecanum4WD

    // mecanum4WD_velocity_setpoint.setVelocities(
    //     p2pComm._rxDatagram.payload.Q[2],
    //     p2pComm._rxDatagram.payload.Q[3],
    //     p2pComm._rxDatagram.payload.Q[4]);

    // // Debug: Print received data.
    // Serial.print("Beta: ");
    // Serial.print(Beta);
    // Serial.print("\n Gamma: ");
    // Serial.println(Gamma);
    // for (uint8_t i = 0; i < NUM_MOTORS; i++)
    // {
    //     Serial.print(tau[i]);
    //     Serial.print("\t");
    // }
    // Serial.println();

    // mecanum4WD_debugPrintStatus();
    return;
}

/**
 * @brief This function is intended to update the output data on the tx buffer.
 * This data is subsequently picked up and sent on the next StateMachine loop.
 *
 */
void updateOutgoingData()
{

    // if (cont > 100)
    // {
    //     for (uint8_t i = 0; i < NUM_MOTORS; i++)
    //     {
    //         Debug(tau[i]);
    //         Debug("\t");
    //     }
    //     Debugln(" ");

    //     mecanum4WD_debugPrintStatus();
    //     cont = 0;
    // }
    // cont += 1;
    //Control commands from the PS4
    Debugln(p2pComm._rxDatagram.payload.Q[0]);

    //Positive direction in the X axis
    if (p2pComm._rxDatagram.payload.Q[0] == CMD_UP)
    {
        mecanum4WD_velocity_setpoint.setVelocities(0.3, 0.0, 0.0);
    }
    //Negative direction in the X axis
    else if (p2pComm._rxDatagram.payload.Q[0] == CMD_DOWN)
    {
        mecanum4WD_velocity_setpoint.setVelocities(-0.3, 0.0, 0.0);
    }
    //Positive direction in the Y axis
    else if (p2pComm._rxDatagram.payload.Q[0] == CMD_LEFT)
    {
        mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.3, 0.0);
    }
    //Negative direction in the Y axis
    else if (p2pComm._rxDatagram.payload.Q[0] == CMD_RIGHT)
    {
        mecanum4WD_velocity_setpoint.setVelocities(0.0, -0.3, 0.0);
    }
    //Positive rotation in the Z axis
    else if (p2pComm._rxDatagram.payload.Q[0] == CMD_CCLW)
    {
        mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, 0.3);
    }
    //Negative rotation in the Z axis
    else if (p2pComm._rxDatagram.payload.Q[0] == CMD_CLW)
    {
        mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, -0.3);
    }
    else
    {
        mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, 0.0);
    }
    /* code here to consume data, i.e. put outgoing data in the tx buffer*/
    is_outbound_data_ready = false; // clear the flag; set it in the interrupt callback
    return;
}

/**
 * @brief If USE_COOPERATIVE_MULTITASKING build flag is set, this function called by
 * the state machine periodically, at the 1/CTRL_LOOP_PERIOD_US microseconds rate.
 *
 * @return uint8_t
 */
uint8_t my_control_law()
{
    // is_outbound_data_ready = true; // clear the flag; set it in the interrupt callback
    return (uint8_t)StateMachineError::TRANSITION_OK;
}

uint8_t bail()
{
    switch (current_state)
    {

    case State::DISCONNECTED:
        /* do something from DISCONNECTED->DISCONNECTED???, but you're already there!*/
        break;

    case State::CONNECTED:
        break;

    case State::VALIDATED:
    case State::CALIBRATED:
        //-----Torso
        Debug("\nDisabling Motor");
        for (uint8_t i = 0; i < NUM_MOTORS; i++)
        {
            if (motors[i]->turnOff())
            {
                Serial.print("Turned off ");
                Serial.println(motors[i]->name());
            }
            else
            {
                Debug("Failed turning off ");
                Debugln(motors[i]->name());
            }
        }
        //-----Mecanum4WD
        mecanum4WD_turnOff();
        break;

    case State::HOME:
        disconnect();
        break;

    case State::ENGAGED:
        disconnect();
        break;

    case State::DISENGAGED:
        disconnect();
        break;

    case State::POWEREDOFF:
        /* what the heck! this doesn't make any sense POWEREDOFF->DISCONNECTED; check your logic *genious*. */
        break;
    default:
        /* */
        break;
    }
    return (uint8_t)StateMachineError::TRANSITION_OK;
}

uint8_t stress()
{
#if defined(ENABLE_STRESST)
    return (uint8_t)StateMachineError::TRANSITION_OK;
#else
    return (uint8_t)StateMachineError::TRANSITION_FAIL;
#endif
}

uint8_t do_something()
{
    is_outbound_data_ready = true; // clear the flag; set it in the interrupt callback
    return (uint8_t)StateMachineError::TRANSITION_OK;
}