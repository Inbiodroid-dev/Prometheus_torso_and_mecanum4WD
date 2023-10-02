/*
 Inbiodroid MCU UDP socket teleoperation state machine:
 This program receives UDP datagrams, containing either SM (state machine) commands
 or (position) control input data, and acts accordingly based upon current state and
 of course, the nature of the module to be controlled.

 created 23 Sep 2022
 by Inbiodroid technical team.

 This code is property of Inbiodroid.
 */

#include <tuple>
#include "StateMachine.h"
#include "SPI.h"

#define A_THOUSAND 1000

// Messaging object instantiation
#if defined(AVATAR_SIDE) && !defined(OPERATOR_SIDE)
P2pSocket<optrPayload,avtrPayload> p2pComm(endpointConfiguration);
#elif defined(OPERATOR_SIDE) && !defined(AVATAR_SIDE)
P2pSocket<avtrPayload,optrPayload> p2pComm(endpointConfiguration);
#endif

/**
 * @brief This global variable is meant to save the current state.
 * 
 */
State current_state = State::DISCONNECTED;
bool is_outbound_data_ready = false;

void setup()
{
  // Open serial communications and wait for port to open.
  #ifdef DEBUG_LOG
    Serial.begin(115200);
    // while (!Serial) {
    //   ; // wait for serial port to connect. Needed for native USB port only.
    // }
  #endif
  SPI.begin();
  p2pComm.begin(); // Ethernet start; the cofiguration come from the McuConfig object.
}

/**
 * @brief MCU loop. Implements the state machine input.
 * 
 */
void loop()
{
  static UdpError   error = UdpError::UNKNOWN;
  static Command  command = Command::UNKNOWN;

  #ifdef USE_COOPERATIVE_MULTITASKING
  #ifdef ARDUINO_TEENSY41
  static elapsedMicros ctrlLoopElapsedMicroseconds;
  #elif ARDUINO_ESP32_DEV
  static uint32_t last_time = 0;
  static uint32_t now = 0;
  #endif
  #endif

  #if defined(ENABLE_STRESST) && defined(ARDUINO_TEENSY41)
  static elapsedMicros stressLoopElapsedMs;
  #endif

  // static uint32_t consecutive_timing_fails = 0;

  /* 1. Check if a new datagram is available. */
  std::tie(error, command) = p2pComm.receiveDatagram();

  switch (error)
  {
  case UdpError::MESSAGE_OK:
  /* 2. A new UDP message has been received & validated. Execute command then. */
    switch (command)
    {
    /* 3. For every command, evaluate whether you can excute it based on the current state. */
    case Command::ENQ:
      switch (current_state)
      {
      case State::DISCONNECTED:
        static uint64_t cntr = 0;
        // fill out the payload space with printable chars
        for(unsigned int i = 0; i < sizeof(p2pComm._txDatagram.payload); i++)
        {
          p2pComm.getTxPayloadBuffer()[i] = byte(' '+((cntr+i)%95));
        }
        p2pComm._txDatagram.returnCode = (uint8_t)++cntr; // increment and assign the var
        p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
        break;

      default:
	  	#ifdef DEBUG_LOG
        Serial.printf("WARN: Command 0x%02X invalid for current state 0x%02X.\n", command, current_state);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::CURRENTSTATE_INVALID;
        break;
      }
      break;


    case Command::SM_STRESST:
      switch (current_state)
      {
      case State::DISCONNECTED:
        /* Run stress test only when DISCONNECTED. */
        Debugln("DISCONNECTED->STRESSTING");
        p2pComm._txDatagram.returnCode = stress();
        if(p2pComm._txDatagram.returnCode == 0)
        {
          current_state = State::STRESSTING;
          #if defined(ENABLE_STRESST) && defined(ARDUINO_TEENSY41)
            stressLoopElapsedMs = 0;
          #endif
          // p2pComm._txDatagram.timestamp = 0;
          // p2pComm._lastReceivedTimestamp = 0;
          Debugln("Command::SM_STRESST success.");
        }
        else
        {
          Debug("Command::SM_STRESST fail: 0x");
          DebugPln(p2pComm._txDatagram.returnCode, HEX);
        }
        break;
      
      default:
        #ifdef DEBUG_LOG
        Serial.printf("WARN: Command 0x%02X invalid for current state 0x%02X.\n", command, current_state);
        #endif
		p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
        break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
      break;

    //Connect the Teensy to the router
    case Command::SM_CONNECT:
      switch (current_state)
      {
      case State::DISCONNECTED:
        Debugln("DISCONNECTED->CONNECTED");
        p2pComm._txDatagram.returnCode = connect();
        if(p2pComm._txDatagram.returnCode == 0)
        {
          current_state = State::CONNECTED;
          Debugln("Command::SM_CONNECT success.");
        }
        else
          Debugln("Command::SM_COMMAND fail.");
        break;

      case State::CONNECTED:
        Debugln("Already CONNECTED.");
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_DONEALREADY;
        break;

      default:
        #ifdef DEBUG_LOG
        Serial.printf("WARN: Command 0x%02X invalid for current state 0x%02X.\n", command, current_state);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
        break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
      break;

    //Connect the Teensy to the motors
    case Command::SM_VALIDATE:
      switch (current_state)
      {
      case State::CONNECTED:
        Debugln("CONNECTED->VALIDATED");
        p2pComm._txDatagram.returnCode = validate();
        if(p2pComm._txDatagram.returnCode == 0)
        {
          current_state = State::VALIDATED;
          Debugln("Command::SM_VALIDATE success.");
        }
        else
          Debugln("Command::SM_VALIDATE fail.");
        break;
      
      case State::VALIDATED:
        Debugln("Already VALIDATED");
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_DONEALREADY;
        break;
      
      default:
        #ifdef DEBUG_LOG
        Serial.printf("WARN: Command 0x%02X invalid for current state 0x%02X.\n", command, current_state);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
        break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
      break;

    //Set the current position as zero
    case Command::SM_CALIBRATE:
      switch (current_state)
      {
      case State::VALIDATED:
        Debugln("VALIDATED->CALIBRATED");
        p2pComm._txDatagram.returnCode = calibrate();
        if(p2pComm._txDatagram.returnCode == 0)
        {
          current_state = State::CALIBRATED;
          Debugln("Command::SM_CALIBRATE success.");
        }
        else
          Debugln("Command::SM_CALIBRATE fail.");
        break;

      case State::CALIBRATED:
        Debugln("Already CALIBRATED");
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_DONEALREADY;
        break;

      default:
        #ifdef DEBUG_LOG
        Serial.printf("WARN: Command 0x%02X invalid for current state 0x%02X.\n", command, current_state);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
        break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
      break;

    // Go home, which is velocity zero
    case Command::SM_GOTOHOME:
      switch (current_state)
      {
      case State::CALIBRATED:
        Debugln("CALIBRATED->HOME");
        p2pComm._txDatagram.returnCode = gotohome();
        if(p2pComm._txDatagram.returnCode == 0)
        {
          current_state = State::HOME;
          Debugln("Command::SM_HOME success.");
        }
        else
          Debugln("Command::SM_HOME fail.");
        break;

      case State::DISENGAGED:
        Debugln("DISENGAGED->HOME");
        p2pComm._txDatagram.returnCode = gobacktohome();
        if(p2pComm._txDatagram.returnCode == 0)
        {
          current_state = State::HOME;
          Debugln("Command::SM_HOME success.");
        }
        else
          Debugln("Command::SM_HOME fail.");
        break;

      case State::HOME:
        Debugln("Already at HOME");
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_DONEALREADY;
        break;

      default:
        #ifdef DEBUG_LOG
        Serial.printf("WARN: Command 0x%02X invalid for current state 0x%02X.\n", command, current_state);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
        break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
      break;

    // Begin teleoperation
    case Command::SM_ENGAGE:
      switch (current_state)
      {
      case State::HOME:
        Debugln("HOME->ENGAGED");
        p2pComm._txDatagram.returnCode = engage();
        if(p2pComm._txDatagram.returnCode == 0)
        {
          current_state = State::ENGAGED;
          #ifdef USE_COOPERATIVE_MULTITASKING
          #ifdef ARDUINO_TEENSY41
            ctrlLoopElapsedMicroseconds = 0;
          #elif ARDUINO_ESP32_DEV
            last_time = micros();
          #endif
          #endif
          // p2pComm._txDatagram.timestamp = 0;
          // p2pComm._lastReceivedTimestamp = 0;
          Debugln("Command::SM_ENGAGE success.");
        }
        else
          Debugln("Command::SM_ENGAGE fail.");
        break;

      case State::ENGAGED:
        Debugln("Already ENGAGED");
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_DONEALREADY;
        break;

      default:
        #ifdef DEBUG_LOG
        Serial.printf("WARN: Command 0x%02X invalid for current state 0x%02X.\n", command, current_state);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
        break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
      break;

    //Finish teleoperation
    case Command::SM_DISENGAGE:
      switch (current_state)
      {
      case State::ENGAGED:
        Debugln("ENGAGED->DISENGAGED");
        p2pComm._txDatagram.returnCode = disengage();
        if(p2pComm._txDatagram.returnCode == 0)
        {
          current_state = State::DISENGAGED;
          Debugln("Command::SM_DISENGAGE success.");
        }
        else
          Debugln("Command::SM_DISENGAGE fail.");
        break;

      case State::DISENGAGED:
        Debugln("Already DISENGAGED");
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_DONEALREADY;
        break;

      default:
        #ifdef DEBUG_LOG
        Serial.printf("WARN: Command 0x%02X invalid for current state 0x%02X.\n", command, current_state);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
        break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
      break;

    //Turn off the motors
    case Command::SM_DISCONNECT:
      switch (current_state)
      {
      case State::HOME:
        Debugln("HOME->DISCONNECTED");
        p2pComm._txDatagram.returnCode = disconnect();
        if(p2pComm._txDatagram.returnCode == 0)
        {
          current_state = State::DISCONNECTED;
          Debugln("Command::SM_DISCONNECT success.");
        }
        else
          Debugln("Command::SM_DISCONNECT fail.");
        break;

      case State::DISCONNECTED:
        Debugln("Already DISCONNECTED");
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_DONEALREADY;
        break;

      default:
        #ifdef DEBUG_LOG
        Serial.printf("WARN: Command 0x%02X invalid for current state 0x%02X.\n", command, current_state);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
        break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
      break;


    case Command::SM_POWEROFF:
      switch (current_state)
      {
      case State::DISCONNECTED:
        Debugln("DISCONNECTED->POWEREDOFF");
        p2pComm._txDatagram.returnCode = poweroff();
        if(p2pComm._txDatagram.returnCode == 0)
        {
          current_state = State::POWEREDOFF;
          Debugln("Command::SM_POWEROFF success.");
        }
        else
          Debugln("Command::SM_POWEROFF fail.");
        break;
      
      default:
        #ifdef DEBUG_LOG
        Serial.printf("WARN: Command 0x%02X invalid for current state 0x%02X.\n", command, current_state);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
        break;
      } 
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
      break;


    case Command::TO_RECEIVEDATA:
    /* 3.1 If the command is to consume new data, do something here? */
      switch (current_state)
      {
      case State::ENGAGED:
      case State::STRESSTING:
        if (p2pComm.validateTimestamp(p2pComm._rxDatagram.timestamp) == UdpError::TIMESTAMP_OK)
        {
          // Debugln("Process incoming teleoperation data.");
          // consecutive_timing_fails = 0;
          updateIncomingData();
        }
        #ifndef DISABLE_TIMESTAMP 
        // else if (p2pComm.m_gapClosingCount > A_THOUSAND) // implossible!! twos complement doesn't work this way.
        // //if (p2pComm.validateTimestamp(p2pComm._rxDatagram.timestamp) == UdpError::TIMESTAMP_FAIL)
        // {
        //   // p2pComm._txDatagram.returnCode = (uint8_t)UdpError::TIMESTAMP_FAIL;
        //   // consecutive_timing_fails++;
        //   // resync
        //   p2pComm._lastReceivedTimestamp = p2pComm._rxDatagram.timestamp;
        //   Debugln("LOG: p2pComm._lastReceivedTimestamp REWINDED to p2pComm._rxDatagram.timestamp");  // <- this means our peer is way behind, possibly due to a reset, so we go back and match its timestamp.
        // }
        else if (p2pComm.m_gapWideningCount > A_THOUSAND)
        {
          //resync
          p2pComm._lastReceivedTimestamp = p2pComm._rxDatagram.timestamp;
          Debugln("LOG: p2pComm._lastReceivedTimestamp SET to p2pComm._rxDatagram.timestamp"); // <- this means we're not being able to close the gap, so we simply close it to the received timestamp.
        }
        #endif
        break;

      default:
        #ifdef DEBUG_LOG
        //Serial.printf("Not engaged. Incoming teleoperation data discarded. State 0x%02X invalid to receive command 0x%02X.\n", current_state, command);
        #endif
        // p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::CURRENTSTATE_INVALID; // don't send shit!
        // p2pComm.sendDatagram(MessageType::EXCEPTION);
        break;
      }
      break;


    case Command::SM_ABORT:
      Debugln("XXXXXXXX->DISCONNECT");
      p2pComm._txDatagram.returnCode = bail();
      if(p2pComm._txDatagram.returnCode == 0)
      {
        current_state = State::DISCONNECTED;
        Debugln("Command::SM_ABORT success.");
      }
      else
        Debugln("Command::SM_ABORT fail.");
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
      break;


    default:
    /* 3.2 ToDo: handle unknown command. */
      Debugln("You're not yet handling this command. Tell the coder he or she or xe/xhe or \"it\" to finish effin' the job.");
      p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_ERROR;
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
      break;
    }
  break;


  case UdpError::NO_PACKET:
  /* 2.1 No new UDP; do something here? Yes! Maybe. Maybe, check here if we have output data to be sent... */
    break;

  default:
  /* 2.3 ToDo: Handle UDP error. In the meantime, just print the specific error.  */
    p2pComm._rxDatagram.command = (uint8_t)command;
    p2pComm._rxDatagram.returnCode = (uint8_t)error;
    #ifdef DEBUG_LOG
      Serial.printf("<UdpError::0x%02X>\n",error);
    #endif
    // p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK); // buggy! Sending an ACK to the peer when receiving 0x40, with 0x40 in the reply command, introduces timestamp synchronization problems.
    break;
  }



  /* 4. Now, based on current state, do something additional? */
  switch (current_state)
  {
  case State::STRESSTING:
    #if defined(ENABLE_STRESST) && defined(ARDUINO_TEENSY41)
    if (stressLoopElapsedMs >= DOSOMETHING_PERIOD_US)
    {
      /* call stress looped function */
      stressLoopElapsedMs -= DOSOMETHING_PERIOD_US;
      p2pComm._txDatagram.returnCode = do_something();
    }
    #endif
    if (is_outbound_data_ready)  // flag set by the interrupt
    {
      updateOutgoingData();
      p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_OK;
      p2pComm.sendDatagram(MessageType::TELEOPERATION_DATA);
    }
    break;

  /* 4.1 Send new output data if it's ready to go. */
  case State::ENGAGED:
    // #ifdef USE_INTERRUPTS
    #ifdef USE_COOPERATIVE_MULTITASKING
    #ifdef ARDUINO_TEENSY41
    if (ctrlLoopElapsedMicroseconds >= CTRL_LOOP_PERIOD_US)
    {
      /* call ctrl loop function */
      ctrlLoopElapsedMicroseconds -= CTRL_LOOP_PERIOD_US;
      p2pComm._txDatagram.returnCode = my_control_law();
    }
    #elif ARDUINO_ESP32_DEV
    now = micros();
    if (now - last_time >= CTRL_LOOP_PERIOD_US)
    {
      last_time = now;
      p2pComm._txDatagram.returnCode = my_control_law();
    }
    #endif
    #endif
    if (is_outbound_data_ready)  // flag set by the interrupt
    {
      updateOutgoingData();
      p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_OK;
      p2pComm.sendDatagram(MessageType::TELEOPERATION_DATA);
    }
    break;
  
  default:
    break;
  }

  #if 0 // delays for raw debugging purposed; enable at your own peril!
  delay(1000); // 1Hz
  #elif 0
  delayMicroseconds(250); // 4KHz
  #endif
}

/*
// Processing UDP example to send and receive data buffer;
// press any key to send the byte message
import hypermedia.net.*;

UDP udp;

String ip = "16.221.3.122";                                // the remote IP address (avatarsideMCUTestConfig)
int port = 0xC07A;                                         // the destination port (avatarsideMCUTestConfig)
int buffer_size = 8*4;                                     // Max buffer size on the NativeEthernet.h es is by default (Teensy4.1/PlatformIO)
byte[] phis = new byte[buffer_size];                       // 7 32-bit floats for arm teleoperation

void setup() {
  udp = new UDP(this, 8817);                               // create a new datagram connection on port (RX)
  udp.log(true);                                           // the connection activity
  udp.listen(true);                                        // and wait for incoming message
}

void draw(){
}

void keyPressed() {
  for(int i = 0; i < buffer_size; i++){
    phis[i] = byte(i+'A');
  }
  udp.send(phis, ip, port );                               // send the message
}

void receive(byte[] data) {                                // <-- default handler
//void receive(byte[] data, String ip, int port ) {        // <-- extended handler
  for(int i=0; i < data.length; i++)
    print(char(data[i]));
  println();
}
*/