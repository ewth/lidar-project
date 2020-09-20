/**
 * Swol v1.0
 * ---------
 * Swol drives the motor.
 * It listens for step commands, and broadcasts movement results.
 */

#define CLIENT                      2         // Client ID (permanent)
#define CLIENT_NAME                 "Swol"    // Client name, only really used for display
#define DEBUG                       0     // Debug mode on/off
#define STATE_TIMEOUT               3000      // Timeout (in MS) before a state *should* timeout
#define STEPS_REV                   4096      // Number of steps per revolution
#define STEPPER_DELAY               10        // Delay between steps
// #define STEP_PIN_1                  16        // Stepper motor pin 1
// #define STEP_PIN_2                  5        // Stepper motor pin 2
// #define STEP_PIN_3                  19        // Stepper motor pin 3
// #define STEP_PIN_4                  22        // Stepper motor pin 4
#define STEP_PIN_1          15
#define STEP_PIN_2          2
#define STEP_PIN_3          0
#define STEP_PIN_4          4
#define EEPROM_STEP_ADDR            87        // Address to read step count from

// -------------------------------
// DO NOT edit below here
// -------------------------------

#define STATE_ADDITIONAL_SETUP      1
#define STATE_STARTUP               2
#define STATE_VERIFY_BRAIN          3
#define STATE_AWAIT_BC_STEP_REQ     4
#define STATE_AWAIT_STEP_CMD        5
#define STATE_PERFORM_STEP_CMD      6

#define STATE_PROCESS_TIMEDOUT      55
#define STATE_SYSTEM_RESTART        77
#define STATE_WIFI_DISCONNECTED     88
#define STATE_SYSTEM_FAILURE        99

#include "LidarComms.h"
#include "LidarState.h"
#include <EEPROM.h>

volatile int currentStep = 0;
volatile int currentSequenceStep = 0;

bool busyIndicator = false;

bool brainVerified = false;
int previousStep = -1;
int targetStep = 0;
bool systemFailure = false;
bool stepperReverse = false;
bool broadcastStepRequestReceived = false;
bool systemRestartCommandReceived = false;
bool stepCommandReceived = false;
bool stepCommandPerformed = false;
int stepCommandedTo = -1;
long lastStepTime = 0;

int stepSequence[8] = {
  B01000,
  B01100,
  B00100,
  B00110,
  B00010,
  B00011,
  B00001,
  B01001
};

LidarComms lidarComms = LidarComms(CLIENT, (CLIENT == 1), DEBUG);
LidarState lidarState = LidarState(STATE_TIMEOUT);

void setup()
{
  Serial.begin(115200);
  Serial.printf("Starting %s...\nTarget AP: %s\n", CLIENT_NAME, lidarComms.getWifiSsid());

  WiFi.onEvent(WiFiEvent);
  lidarState.setStateChangeHandler(handleStateChange);
  lidarComms.setMessageHandler(handleMessage);

  currentStep = 0;

  Serial.println("Boot complete, performing additional setup...");
  
  lidarState.transitionTo(STATE_ADDITIONAL_SETUP);
  
}

void loop()
{
  lidarComms.checkUdpPacket();
  doStateActions();
  delay(1);
}

/**
 * Handle when we move in/out of a state
 */
void handleStateChange(int stateTo, int stateFrom)
{
  if (DEBUG == true)
    Serial.printf("\n\nTransition from state %d to state %d\n\n", stateFrom, stateTo);

  // State entry
  switch (stateTo) {
    case STATE_ADDITIONAL_SETUP:
      pinMode(STEP_PIN_1, OUTPUT);
      pinMode(STEP_PIN_2, OUTPUT);
      pinMode(STEP_PIN_3, OUTPUT);
      pinMode(STEP_PIN_4, OUTPUT);
      checkEeprom();
    break;

    case STATE_STARTUP:
      systemRestartCommandReceived = false;
      // lidarState.setLedState(false, false, true);
      lidarComms.connectWifi();
    break;

    case STATE_AWAIT_BC_STEP_REQ:
      // lidarState.setLedState(true, false, true);
      broadcastStepRequestReceived = false;
    break;

    case STATE_AWAIT_STEP_CMD:
      stepCommandedTo = -1;
      stepCommandReceived = false;
    break;

    case STATE_PERFORM_STEP_CMD:
      stepCommandPerformed = false;
    break;

    case STATE_SYSTEM_RESTART:
      // lidarState.setLedState(true, true, true);
      Serial.println("***");
      Serial.println("-> SYSTEM RESTART command received.");
      Serial.println("***");
    break;

    case STATE_WIFI_DISCONNECTED:
      lidarComms.disconnectWifi();
      delay(1000);
      lidarState.transitionTo(STATE_STARTUP);
    break;

    case STATE_SYSTEM_FAILURE:
      lidarComms.messageBroadcastSystemFailure();
      // @todo: flash led
      // lidarState.setLedState(true, false, false);
    break;
      
  }

  // State exit
  switch (stateFrom) {
    case STATE_AWAIT_BC_STEP_REQ:
      lidarComms.messageBroadcastStep(currentStep);
      // lidarState.setLedState(false, true, false);
    break;
  }
}

/**
 * Perform actions in the current state
 */
void doStateActions()
{
  switch (lidarState.getCurrentState()) {

    case STATE_ADDITIONAL_SETUP:
      lidarState.transitionTo(STATE_STARTUP);
    break;

    // ------------------------------------

    case STATE_STARTUP:
      if (lidarComms.isConnected()) {
        lidarState.transitionTo(STATE_AWAIT_BC_STEP_REQ);
        return;
      }
      if (lidarState.isTimedOut())
        lidarState.transitionTo(STATE_WIFI_DISCONNECTED);
    break;
    
    // ------------------------------------

    case STATE_AWAIT_BC_STEP_REQ:
      if (broadcastStepRequestReceived) {
        lidarState.transitionTo(STATE_AWAIT_STEP_CMD);
        return;
      }
      commonStateChecks();
    break;

    // ------------------------------------

    case STATE_AWAIT_STEP_CMD:
      if (stepCommandReceived) {
        lidarState.transitionTo(STATE_PERFORM_STEP_CMD);
        return;
      }
      commonStateChecks();
    break;

    // ------------------------------------

    case STATE_PERFORM_STEP_CMD:
      // This is the only (normal) state with no timeout recovery
      performStepCommand();
      if (stepCommandPerformed) {
        lidarState.transitionTo(STATE_AWAIT_STEP_CMD);
      }
    break;

    // ------------------------------------

    case STATE_SYSTEM_RESTART:
      delay(2000);
      lidarState.transitionTo(STATE_STARTUP);
    break;
    
    // ------------------------------------

    case STATE_PROCESS_TIMEDOUT:
      lidarState.transitionTo(STATE_STARTUP);
    break;
  }
}

/**
 * Common state checks merged into a single method
 */ 
bool commonStateChecks()
{
    if (systemFailure) {
      lidarState.transitionTo(STATE_SYSTEM_FAILURE);
      return false;
    }
    if (systemRestartCommandReceived) {
      lidarState.transitionTo(STATE_SYSTEM_RESTART);
      return false;
    }
    if (!lidarComms.isConnected()) {
      lidarState.transitionTo(STATE_WIFI_DISCONNECTED);
      return false;
    }
    if (lidarState.isTimedOut()) {
      lidarState.transitionTo(STATE_PROCESS_TIMEDOUT);
      return false;
    }

    return true;
}

/**
 * Check data stored in EEPROM
 */
void checkEeprom()
{
  EEPROM.begin(2);
  // Big endian
  byte readStepMsb = EEPROM.read(0);
  byte readStepLsb = EEPROM.read(1);
  
  if ((readStepMsb == 255 && readStepLsb == 255) || (readStepMsb == 0 && readStepLsb == 0)) {
    if (DEBUG == true)
      Serial.println("Read bad EEPROM value, defaulting 0");
    storeEeprom();
  } else {
    int eepromStep = (readStepMsb << 8) | readStepLsb;
    if (DEBUG == true)
      Serial.printf("Read step value of %d\n", eepromStep);
    currentStep = eepromStep;
  }
}

/**
 * Write data to EEPROM
 */
void storeEeprom()
{
  byte readStepMsb = (byte) ((currentStep >> 8) & 0xFF);
  byte readStepLsb = (byte) (currentStep & 0xFF);
  EEPROM.write(0, readStepMsb);
  EEPROM.write(1, readStepLsb);
  EEPROM.commit();
}

/**
 *  Handle incoming messages. This will be called from LidarComms class.
 */
void handleMessage(int msgFrom, int msgTo, int msgDescriptor, int msgMetaData, int msgValue)
{

  // if (DEBUG == true)
  //   Serial.printf("Client message received:\n\tFrom: %d\n\tTo: %d\n\tDescriptor: %d\n\tMetaData: %d\n\tValue: %d\n\n", msgFrom, msgTo, msgDescriptor, msgMetaData, msgValue);

  if (msgTo != 0 && msgTo != lidarComms.getClientId())
    return;
    
  switch (msgDescriptor) {
    // Client information
    case MSG_CLIENT_INFO:
      lidarComms.addClientInfo(msgValue, msgMetaData);
    break;
    
    // Step to command
    case MSG_STEP_TO_CMD:
      if (DEBUG == true)
        Serial.printf("We're being told to move to step %d\n", msgValue);
      stepCommandedTo = msgValue;
      stepCommandReceived = true;
    break;

    // Request to broadcast current step
    case MSG_REQ_BC_STEP:
      broadcastStepRequestReceived = true;
      lidarComms.messageBroadcastStep(currentStep);
    break;

    // Request for current step
    case MSG_REQ_STEP:
      lidarComms.messageResponseStep(msgFrom, currentStep);
    break;

    case MSG_SYSTEM_RESTART_CMD:
      systemRestartCommandReceived = true;
    break;

  }

}

/**
 * 
 */ 
void performStepCommand()
{
  stepTo(stepCommandedTo);
  lidarComms.messageBroadcastStep(currentStep);
  storeEeprom();

  stepCommandPerformed = true;
}

/**
 * Step to a specific step
 */
bool stepTo(int targetStep)
{
  if (DEBUG == true)
    Serial.printf("We are attempting to move from step %d to step %d\n", currentStep, targetStep);
   
  // Factor in rollover
  if (targetStep > STEPS_REV)
    targetStep = STEPS_REV - targetStep;

  // Calculate how many steps we need to take
  if (targetStep == currentStep)
    return true;

  if (DEBUG == true)
    Serial.printf("We are attempting to move to step %d\n", targetStep);

  // NOTE: For now, we are working under the assumption we will need to go forward and then reverse so we don't tangle wires. This may change!!!

  // Move backwards to the target
  if (targetStep < currentStep) {
    stepperReverse = true;
  }
  
  // Move forwards to the target
  if (targetStep > currentStep) {
    stepperReverse = false;
  }

  while (currentStep != targetStep) {
    takeStep();
  }
  return currentStep == targetStep;
}

/**
 * Take a single step
 */
void takeStep()
{
  long currentTime = millis();
  long timeUntilReady = (lastStepTime + STEPPER_DELAY) - currentTime;
  if (timeUntilReady > 0) {
    delay(timeUntilReady);
  }

  lastStepTime = millis();
  writeStepper(currentSequenceStep);
  
  if (stepperReverse)
    currentStep--;
  else
    currentStep++;
  
  if (currentStep > STEPS_REV)
    currentStep = 0;

    
  currentSequenceStep++;
  
  if (currentSequenceStep > 7)
    currentSequenceStep = 0;

}

/**
 * Write the stepper position to the stepper motor
 */
void writeStepper(int sequenceStep) {

  int sequenceIndex;
  if (stepperReverse)
    sequenceIndex = 7 - currentSequenceStep;
  else
    sequenceIndex = currentSequenceStep;

  int sequence = stepSequence[sequenceIndex];
  
  if (DEBUG == true)
    Serial.printf("Sequence step: %d\n", sequenceIndex);
    
  digitalWrite(STEP_PIN_1, bitRead(sequence,0));
  digitalWrite(STEP_PIN_2, bitRead(sequence,1)); 
  digitalWrite(STEP_PIN_3, bitRead(sequence,2));
  digitalWrite(STEP_PIN_4, bitRead(sequence,3));
}

/**
 * Event handler for WiFi events
 */ 
void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  lidarComms.wifiEvent(event, info);
}
