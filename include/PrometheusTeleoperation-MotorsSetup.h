#pragma once

#include "MitMotor.h"
#include "pin_definitions.h"


/* Motors declaration*/
// * Motors declaration
CanMotor *motors[] = {
    new MitMotor(MitMotor::AK_10, CS_0, INT_0, "AK_10_1"),
    new MitMotor(MitMotor::AK_10, CS_1, INT_1, "AK_10_2"),
    new MitMotor(MitMotor::AK_10, CS_2, INT_2, "AK_10_3")};

constexpr size_t NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

// * Motors Interrupts
void (*interrupt_handlers[NUM_MOTORS])() = {
    []()
    { motors[0]->handleInterrupt(); },
    []()
    { motors[1]->handleInterrupt(); },
    []()
    { motors[2]->handleInterrupt(); }};

// Control variables
float Beta = 0.0;
float Gamma = 0.0;
float Beta_aux = 0.0;
float Gamma_aux = 0.0;
float Fc = 35;
int counter = 0;
float theta[3] = {0.0, 0.0, 0.0};
float Dq[3] = {0.0, 0.0, 0.0};
float Tau[3] = {0.0, 0.0, 0.0};
float Kp[3] = {3.0, 18.0, 18.0};
float Kd[3] = {1.0, 1.5, 1.5};
float kf[3] = {1.0, 1.0, 1.0};
float tau[3] = {0.0, 0.0, 0.0};
float sat[3] = {4.0, 6.0, 6.0};
float qd[3] = {0.0, 0.0, 0.0};
float bias_x = 0.0;
float bias_y = 0.0;
float bias_z = 0.0;

float x = 0.0;
float y = 0.0;
float z = 0.0;


// Sign Function
float Sign(float number)
{
    if (number < 0)
    {
        return -1.0;
    }
    else
    {
        return 1.0;
    }
}

//Control Loop
void controlMotors(float elapsed_time_seconds)
{
    if (Beta > 0.3)
    {
        Beta = 0.3;
    }
    if (Beta < -0.3)
    {
        Beta = -0.3;
    }
    if (Gamma > 0.3)
    {
        Gamma = 0.3;
    }
    if (Gamma < -0.3)
    {
        Gamma = -0.3;
    }

    Gamma_aux = Gamma;
    Beta_aux = Beta;

    qd[0] = Gamma_aux;
    qd[1] = Beta_aux;
    qd[2] = -Beta_aux;

    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        Dq[i] = Fc * (theta[i] + motors[i]->position());
        theta[i] = theta[i] - elapsed_time_seconds * Dq[i];
    }

    //Control Law for Zero Regulation
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        // tau[i] = -Kp[i] * (motors[i]->position() - qd[i]) - Kd[i] * Dq[i] - kf[i] * Sign(motors[i]->position() - qd[i]) * pow(fabs(motors[i]->position() - qd[i]), 0.5);
        //tau[i] = -Kp[i] * (motors[i]->position() - qd[i]);
        tau[i] = -Kp[i] * (motors[i]->position() - qd[i]) - Kd[i] * Dq[i];
        // tau[i] = 0.0;
    }

    //Send Motor Torque
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {

        if (fabs(tau[i]) > sat[i])
        {
            tau[i] = Sign(tau[i]) * sat[i];
        }

        #if defined (DONTACTUATETORSO)
        tau[i] = 0.0f;
        #endif

        if (!motors[i]->setTorque(tau[i]))
        {
            Serial.print("Message NOT Sent to ");
            Serial.println(motors[i]->name());
        }
    }
}
