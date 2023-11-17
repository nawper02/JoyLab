#include <Dynamixel2Arduino.h>

// Definitions
#define DXL_BUS Serial1  // For Dynamixel
#define USB Serial       // For JoyLab

const uint8_t SERVO_ID_1 = 1;
const uint8_t SERVO_ID_2 = 2;
const int INIT_POSITION_1 = 2589;          // Starting position (middle)
const int INIT_POSITION_2 = 2667;          // Starting position (middle)
const int INIT_CURRENT_LIMIT = 500;        // Initial current limit
const int DXL_BAUD_RATE = 1000000;         // Baud rate for communicating with servos
const int JL_BAUD_RATE = 1000000;          // Baud rate for communicating with JoyLab

Dynamixel2Arduino dxl(DXL_BUS);            // Create an instance of the Dynamixel2Arduino class
using namespace ControlTableItem;          // For using constant definitions

void setup() {

  dxl.setPortProtocolVersion(2.0);

  // Initialize JoyLab serial communication
  USB.begin(JL_BAUD_RATE);

  // Initialize the servo serial communication protocol
  dxl.begin(DXL_BAUD_RATE);

  initializeServo(SERVO_ID_1, INIT_POSITION_1, INIT_CURRENT_LIMIT);
  initializeServo(SERVO_ID_2, INIT_POSITION_2, INIT_CURRENT_LIMIT);

  USB.println("Setup complete. Servomotors are initialized.");
}

void loop() {
  // Step 1: Read servomotor positions
  int currentPosition1 = readServoPosition(SERVO_ID_1);
  int currentPosition2 = readServoPosition(SERVO_ID_2);

  // Step 2: Send positions via Serial if no errors
  if ((currentPosition1 != 0) && (currentPosition2 != 0)){
    USB.print(currentPosition1);
    USB.print(",");
    USB.println(currentPosition2);
  }

  // Step 3: Receive servo goal positions from Serial if available
  if (USB.available()) {
    String input = USB.readStringUntil('\n');
    parseAndExecuteInput(input);
  }

}

// Function to read servomotor position
int readServoPosition(uint8_t servoId) {
  int position = dxl.getPresentPosition(servoId);
  /*if (position == 0) {  // Check if the function returned 0 (False)
    uint8_t errorCode = dxl.getLastLibErrCode();  // Get the last error code
    USB.print("error_code,");
    USB.println(errorCode);  // Print the error code to the Serial monitor
  }*/
  return position;
}

void parseAndExecuteInput(String input) {
  int separatorIndex = input.indexOf(',');
  String topic = input.substring(0, separatorIndex);
  String values = input.substring(separatorIndex + 1);


  if (topic == "set_current_limit") {
    uint32_t currentLimit = values.toInt();
    switchTorqueAndWrite(CURRENT_LIMIT, currentLimit); // goal current or current limit?
  }
  else if (topic == "set_servo_mode"){
    uint8_t servoMode = values.toInt();
    dxl.torqueOff(SERVO_ID_1);
    dxl.torqueOff(SERVO_ID_2);
    dxl.setOperatingMode(SERVO_ID_1, servoMode);
    dxl.setOperatingMode(SERVO_ID_2, servoMode);
    dxl.torqueOn(SERVO_ID_1);
    dxl.torqueOn(SERVO_ID_2);
  }
  else if (topic == "toggle_torque"){
    uint8_t state = values.toInt();
    dxl.writeControlTableItem(TORQUE_ENABLE, SERVO_ID_1, state);
    dxl.writeControlTableItem(TORQUE_ENABLE, SERVO_ID_2, state);
  }
  else if (topic == "goal_positions") {
    int posSeparatorIndex = values.indexOf(',');
    int goalPosition1 = values.substring(0, posSeparatorIndex).toInt();
    int goalPosition2 = values.substring(posSeparatorIndex + 1).toInt();
    dxl.setGoalPosition(SERVO_ID_1, goalPosition1);
    dxl.setGoalPosition(SERVO_ID_2, goalPosition2);
  }
  else if (topic == "goal_currents") {
    int posSeparatorIndex = values.indexOf(',');
    int goalCurrent1 = values.substring(0, posSeparatorIndex).toInt();
    int goalCurrent2 = values.substring(posSeparatorIndex + 1).toInt();
    dxl.writeControlTableItem(GOAL_CURRENT, SERVO_ID_1, goalCurrent1);
    dxl.writeControlTableItem(GOAL_CURRENT, SERVO_ID_2, goalCurrent2);
  }
}

// Function to set control table parameters
void switchTorqueAndWrite(uint8_t ADDR, uint32_t value) { // changes are only being used if you switch control modes

  bool torque_was_enabled = (dxl.getTorqueEnableStat(SERVO_ID_1) || dxl.getTorqueEnableStat(SERVO_ID_2));

  if (torque_was_enabled) {
    dxl.torqueOff(SERVO_ID_1);
    dxl.torqueOff(SERVO_ID_2);
  }

  dxl.writeControlTableItem(ADDR, SERVO_ID_1, value);
  dxl.writeControlTableItem(ADDR, SERVO_ID_2, value);

  if (torque_was_enabled) {
    dxl.torqueOn(SERVO_ID_1);
    dxl.torqueOn(SERVO_ID_2);
  }

  dxl.ping(SERVO_ID_1); // try something here to get the changes to save
  dxl.ping(SERVO_ID_2);

}

// Function to turn the torque on and go to the home position
void initializeServo(int servoId, int initPosition, int initCurrentLimit) {
  dxl.torqueOff(servoId);
  dxl.setOperatingMode(servoId, OP_CURRENT_BASED_POSITION);
  dxl.writeControlTableItem(CURRENT_LIMIT, servoId, initCurrentLimit);
  dxl.torqueOn(servoId);
  dxl.setGoalPosition(servoId, initPosition);
}

/*
                 @@@@@@@@@@@@@%                        @@@@@@@.
            ,@@@@@@@@@@@@      @@@@.                         .,@@@,
          @@@@@@@@@@@@@@@          @@,                           .@@@.
        @@@@@@@@@@@@@@@@@            @@,                            @@@.
       @@@@@@@@@@@@@@@@@@             @@*         @@@@@@@@@@/         @@,
      @@@@@@@@@@@@@@@@@@@              @@,      @@@@@@@@*. .@@@        @@.
      @@@@@@@@@@@@@@@@@@@              (@#.    @@@@@@@@@*.   @@*.      @@,
      @@@@@@@@@@@@@@@@@@@              *@%.@@, @@@@@@@@@*.   @@*.      @@*
      @@@@@@@@@@@@@@@@@@@              @@*.@@, @@@@@@@@@*.  @@@,       @@*
      @@@@@@@@@@@@@@@@@@@             @@#,@@*. @@*@@@@@@@@@@@*.        @@*
      @@@@@@@@@@@@@@@@@@@            @@/,@@*,  @@*   .@@/,.            @@*
      @@,.@@@@@@@@@@@@@@@          @@@*.@@*.   @@*    @@*.             @@*
      @@,  .@@@@@@@@@@@@@       @@@*,(@@*,/@@@/*,..@@@@@@@@&           @@*
      @@,      ,@@@@@@@@@@@@@@@**.#@@@*,@@@*.@@@@/    @@@@@@@@@@@.     @@*
      @@,            ..@@,..  @@@@**. @@*,@@@         @@@@@@@@@@@@@@.  @@*
      @@,              @@,    @@,   @@/,@@@           @@@@@@@@@@@@@@@@,@@*
      @@,           @@@@@@@,  @@,  @@*.@@             @@@@@@@@@@@@@@@@@@@*
      @@,        @@@   @@@@@@@@@, @@*.@@              @@@@@@@@@@@@@@@@@@@*
      @@,       @@     @@@@@@@@@, @@-/@#              @@@@@@@@@@@@@@@@@@@*
      @@,      &@      @@@@@@@@@, .*.@@               @@@@@@@@@@@@@@@@@@@*
      @@,       @@     @@@@@@@@/.     @@              @@@@@@@@@@@@@@@@@@@,
       @@.       @@@   @@@@@@@*.      @@              @@@@@@@@@@@@@@@@@@*.
       .@@.        .,@@@@@(*,          @@.            @@@@@@@@@@@@@@@@@/,
         @@@.                           .@@           @@@@@@@@@@@@@@@@*.
           @@@,                           .@@@        @@@@@@@@@@@@@@*.
             .*@@@@@                         .@@@@@.  @@@@@@@@@@/*.
                  .,,+/(,                         .,*+/(/***,..
*/
