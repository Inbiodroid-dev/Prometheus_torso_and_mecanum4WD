#ifndef STATEMACHINE_H
    #define STATEMACHINE_H
    #include "P2pSocket.h"

    enum class State
    {
        POWEREDOFF,
        STRESSTING,
        DISCONNECTED,
        CONNECTED,
        VALIDATED,
        CALIBRATED,
        HOME,
        ENGAGED,
        DISENGAGED
    };

    /**
     * @brief Types of error sent in the returnCode field.
     * 
     */
    enum class StateMachineError
    {
        TRANSITION_OK = 0,
        TRANSITION_DONEALREADY,
        TRANSITION_INVALID,
        TRANSITION_FAIL,
        CURRENTSTATE_INVALID,
        TRANSITION_ERROR = 0xFF
    };

    extern State    current_state;
    extern uint8_t  connect();
    extern uint8_t  validate();
    extern uint8_t  calibrate();
    extern uint8_t  gotohome();
    extern uint8_t  gobacktohome();
    extern uint8_t  engage();
    extern uint8_t  disengage();
    extern uint8_t  disconnect();
    extern uint8_t  bail();
    extern uint8_t  stress();
    extern uint8_t  poweroff();

    extern    void  updateIncomingData();
    extern    void  updateOutgoingData();
    extern    bool  is_outbound_data_ready;

    #define CTRL_LOOP_PERIOD_US 1000 // period in microseconds [us]
    extern uint8_t  my_control_law();
    #define DOSOMETHING_PERIOD_US 250 // period in microseconds [us]
    extern uint8_t  do_something();

    //Messaging object declaration
    #if defined(AVATAR_SIDE) && !defined(OPERATOR_SIDE)
    extern P2pSocket<optrPayload,avtrPayload> p2pComm;
    #elif defined(OPERATOR_SIDE) && !defined(AVATAR_SIDE)
    extern P2pSocket<avtrPayload,optrPayload> p2pComm;
    #endif
#endif