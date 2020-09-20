/**
 * Smol v1.0
 * ---------
 * Smol sits at the top of the motor's rotor, communicating with the sensors.
 * It connects to Bigbrain's Wifi AP, and then talks to Bigbrain and Swol.
 * It should broadcast sensor data.
 */

#define CLIENT                      3                 // Client ID (permanent)
#define CLIENT_NAME                 "Smol"            // Client name, only really used for display
#define DEBUG                       false             // Debug mode on/off
#define STATE_TIMEOUT               3000              // Timeout (in MS) before a state *should* timeout
#define TOF_INTERVAL                1                 // Delay between ToF readings
#define SAMPLE_SIZE                 50                // Number of samples to get the median from

// -------------------------------
// DO NOT edit below here
// -------------------------------

#define STATE_SENSOR_SETUP          1
#define STATE_STARTUP               2
#define STATE_VERIFY_BRAIN          3
#define STATE_AWAIT_BC_STEP_REQ     4
#define STATE_AWAIT_TOF_POLL_CMD    5
#define STATE_AWAIT_TOF_REQ         6
#define STATE_RESP_TOF_REQ          7

#define STATE_SYSTEM_RESTART        77
#define STATE_WIFI_DISCONNECTED     88
#define STATE_SYSTEM_FAILURE        99

#include "LidarComms.h"
#include "LidarState.h"

LidarComms lidarComms = LidarComms(CLIENT, (CLIENT == 1), DEBUG);
LidarState lidarState = LidarState(STATE_TIMEOUT);

#include "DFRobot_VL53L0X.h"

DFRobotVL53L0X tofSensor;

int currentStep = 0;
int stepReadingAt = 0;
float distanceMeasurement = 0;

bool broadcastStepRequestReceived = false;
bool tofPollCommandReceived = false;
bool tofRequestReceived = false;
bool tofRequestResponded = false;
int tofRequestedBy = 0;
bool systemRestartCommandReceived = false;

int readingsAtCurrentStep = 0;
int readingIndex = 0;
float tofSamples[SAMPLE_SIZE];

bool systemFailure = false;
bool busyIndicator = false;

void setup() {
  
  Serial.begin(115200);
  Serial.printf("Starting %s...\nTarget AP: %s\n", CLIENT_NAME, lidarComms.getWifiSsid());
  
  WiFi.onEvent(WiFiEvent);
  
  lidarState.setStateChangeHandler(handleStateChange);
  lidarComms.setMessageHandler(handleMessage);
 
  Serial.println("Boot complete, setting up sensors...");

  lidarState.transitionTo(STATE_SENSOR_SETUP);

}

void loop() {

  lidarComms.checkUdpPacket();
  doStateActions();
  delay(1);
}

/**
 * Handle when we move in/out of a state
 */
void handleStateChange(int stateTo, int stateFrom)
{
  if (DEBUG === true)
    Serial.printf("\n\nTransition from state %d to state %d\n\n", stateFrom, stateTo);

  // State entry
  switch(stateTo) {

    case STATE_SENSOR_SETUP:
      // Set up TOF sensor
      Serial.println("Put your hand near the ToF sensor so it can get a reading.");
      Wire.begin();
      tofSensor.begin(0x50);
      tofSensor.setMode(Continuous, High);
      tofSensor.start();
    break;
  
    case STATE_STARTUP:
      systemRestartCommandReceived = false;
      lidarState.setLedState(false, false, true);
      lidarComms.connectWifi();
    break;

    case STATE_AWAIT_BC_STEP_REQ:
      lidarState.setLedState(true, false, true);
      broadcastStepRequestReceived = false;
    break;

    case STATE_AWAIT_TOF_POLL_CMD:
      lidarState.setLedState(true, true, false);
      tofPollCommandReceived = false;
    break;

    case STATE_AWAIT_TOF_REQ:
      tofRequestReceived = false;
    break;

    case STATE_RESP_TOF_REQ:
      tofRequestResponded = false;
    break;

    case STATE_SYSTEM_RESTART:
      lidarState.setLedState(true, true, true);
    break;

    case STATE_WIFI_DISCONNECTED:
      lidarComms.disconnectWifi();
      delay(1000);
      lidarState.transitionTo(STATE_STARTUP);
    break;

    case STATE_SYSTEM_FAILURE:
      lidarComms.messageBroadcastSystemFailure();
      // @todo: flash red LED
      lidarState.setLedState(true, false, false);
    break;
  }

  // State exits
  switch (stateFrom)
  {
    case STATE_AWAIT_TOF_POLL_CMD:
      lidarState.setLedState(false, true, false);
    break;
    
    default:
    break;
  }
}

/**
 * Perform actions in the current state
 */
void doStateActions()
{
  switch (lidarState.getCurrentState()) {
    
    case STATE_SENSOR_SETUP:
      if (tofSensor.getDistance() < 16000) {
        Serial.println("Sensor setup complete, beginning startup...");
        lidarState.transitionTo(STATE_STARTUP);
        return;
      }
      // We transition to startup anyway, to try and notify the network of the failure
      if (lidarState.isTimedOut()) {
        systemFailure = true;
        lidarState.transitionTo(STATE_STARTUP);
        return;
      }
      delay(100);
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

    case STATE_VERIFY_BRAIN:
      // Nothing really to do here, seems redundant
      lidarState.transitionTo(STATE_AWAIT_BC_STEP_REQ);
    break;

    // ------------------------------------

    case STATE_AWAIT_BC_STEP_REQ:
      if (broadcastStepRequestReceived) {
        lidarState.transitionTo(STATE_AWAIT_TOF_POLL_CMD);
        return;
      }
      commonStateChecks();

    break;

    // ------------------------------------

    case STATE_AWAIT_TOF_POLL_CMD:
      if (tofPollCommandReceived) {
        lidarState.transitionTo(STATE_AWAIT_TOF_REQ);
        return;
      }
      commonStateChecks();
    break;

    // ------------------------------------
    
    case STATE_AWAIT_TOF_REQ:
      pollTof();
      if (tofRequestReceived)
        lidarState.transitionTo(STATE_RESP_TOF_REQ);

      commonStateChecks();
    break;
    
    // ------------------------------------

    case STATE_RESP_TOF_REQ:
      if (tofRequestResponded) {
        lidarState.transitionTo(STATE_AWAIT_TOF_REQ);
        return;
      }

      respondTofRequest();

      // Bail out here if common checks don't pass, so we don't _also_ trip a system failure
      if (!commonStateChecks()) {
        return;
      }

    break;

    // ------------------------------------

    case STATE_SYSTEM_RESTART:
      Serial.println("***");
      Serial.println("-> SYSTEM RESTART command received.");
      Serial.println("***");
      delay(2000);
      lidarState.transitionTo(STATE_STARTUP);
    break;

    // ------------------------------------

    case STATE_SYSTEM_FAILURE:
      // @todo: flash LED
      lidarState.setLedState(true, false, false);
    break;

    // ------------------------------------


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

    return true;
}


/**
 *  Handle incoming messages. This will be called from LidarComms class.
 */
void handleMessage(int msgFrom, int msgTo, int msgDescriptor, int msgMetaData, int msgValue)
{
  if (DEBUG === true)
    Serial.printf("Client message received:\n\tFrom: %d\n\tTo: %d\n\tDescriptor: %d\n\tMetaData: %d\n\tValue: %d\n\n", msgFrom, msgTo, msgDescriptor, msgMetaData, msgValue);

  if (msgTo != 0 && msgTo != CLIENT)
    return;

  switch (msgDescriptor) {
    // Request to send back TOF data
    case MSG_REQ_TOF:
      tofRequestedBy = msgFrom;
      tofRequestReceived = true;
    break;

    case MSG_REQ_BC_STEP:
      broadcastStepRequestReceived = true;
      lidarComms.messageBroadcastStep(currentStep);
    break;

    case MSG_TOF_POLL_CMD:
      Serial.println("\n\n\nTOF POLL COMMAND RECEIVED!!!");
      tofPollCommandReceived = true;
      lidarComms.messageConfirmTofPollCommand(msgFrom);
    break;

    case MSG_BC_STEP:
      currentStep = msgValue;
      lidarComms.messageBroadcastStep(currentStep);
    break;

    case MSG_SYSTEM_RESTART_CMD:
      systemRestartCommandReceived = true;
    break;
  }
}

/**
 * When a connection event occurs
 */ 
void handleConnection(int clientId)
{
  Serial.println("**");
  Serial.printf("Connection from %d\n", clientId);
  Serial.println("**");
}

/**
 * Get the current TOF reading and send it to the requester
 */
void respondTofRequest()
{
  // It goes WAY too slow if we do multiple samples for smoothing :(
  
  lidarComms.messageResponseTof(tofRequestedBy, currentStep, (int)tofSensor.getDistance());
  tofRequestResponded = true;
  return;

  int iterations = 0;
  while (readingsAtCurrentStep < SAMPLE_SIZE) {
    pollTof();
    delay(TOF_INTERVAL);
    iterations++;
    if (iterations > 1000) {
      systemFailure = true;
      return;
    }
  }
  
  int sLength = sizeof(tofSamples)/ sizeof(tofSamples[0]);
  qsort(tofSamples, sLength, sizeof(tofSamples[0]), qSortAlgo);

  float tofValue = tofSamples[sLength/2];

  lidarComms.messageResponseTof(tofRequestedBy, currentStep, tofValue);
  tofRequestResponded = true;
}


/**
 * Constantly poll ToF data
 */
void pollTof()
{

  if (stepReadingAt != currentStep) {
    stepReadingAt = currentStep;
    readingsAtCurrentStep = 0;
    readingIndex = 0;
  }

  tofSamples[readingIndex] = tofSensor.getDistance();
  readingsAtCurrentStep++;
  readingIndex++;
  if (readingIndex >= SAMPLE_SIZE) {
    readingIndex = 0;
  }
}

/**
 * Algorithm thanks to: https://arduino.stackexchange.com/questions/38177/how-to-sort-elements-of-array-in-arduino-code
 */
int qSortAlgo(const void *cmp1, const void *cmp2)
{
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  return a > b ? -1 : (a < b ? 1 : 0);
}

/**
 * Event handler for WiFi events
 */ 
void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  lidarComms.wifiEvent(event, info);
}
