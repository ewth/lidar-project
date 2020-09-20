/**
 ** Bigbrain v1.0
 * -------------
 * Bigbrain sets up a Wifi AP to allow comms.
 * - Comms are across UDP.
 * - We don't care about connections.
 * - Messages are 16 bytes each, consisting of:
 *    From        sender client ID
 *    To          receipient client ID, 0 = all
 *    Descriptor  integer specifying operation
 *    MetaData    optional integer specifying additional operational params
 *    Value       actual data value
 */

#define DEBUG                       false             // Debug mode on/off
#define CLIENT                      1                 // Client number (permanent)
#define CLIENT_NAME                 "Bigbrain"        // Client name, only really used for display
#define STATE_TIMEOUT               3000              // Timeout (in MS) before a state *should* timeout
#define STEP_FAIL_LIMIT             5                 // No. of attempts to request Swol to step before giving up

// -------------------------------
// DO NOT edit below here
// -------------------------------

#include "LidarComms.h"
#include "LidarState.h"

#define STATE_STARTUP               1
#define STATE_AWAIT_CLIENTS         2
#define STATE_REQUEST_STEP_BC       3
#define STATE_CONFIRM_STEPS         4
#define STATE_SEND_TOF_POLL_CMD     5
#define STATE_REQUEST_TAKE_STEP     6
#define STATE_PROCESS_INVALID_STEP  8
#define STATE_MOVEMENT_ERROR        9
#define STATE_REQUEST_TOF           10
#define STATE_SEND_DATA_PC          11
#define STATE_NEXT_STEP             12
#define STATE_STEP_MATCH_FAILED     13
#define STATE_STEP_BC_NOT_RECV      14
#define STATE_RECOVERABLE_FAILURE   15
#define STATE_SYSTEM_FAILURE        99

LidarComms lidarComms = LidarComms(CLIENT, (CLIENT == 1), DEBUG);
LidarState lidarState = LidarState(STATE_TIMEOUT);

IPAddress bigbrainIp;

int swolCurrentStep = -1;
int smolCurrentStep = -1;
int currentStep = -1;
int stepBeforeRequest;

long stateTimer = 0;
int state = 0;
int prevState = 0;

int stepFailCounter = 0;

long stateTimeout = 10000;

bool stepBroadcastReceived = false;
bool clientSwolConnected = false;
bool clientSmolConnected = false;
bool tofPollCommandConfirmed = false;
bool systemFailed = false;
bool systemFailureBroadcastReceived = false;
int systemFailureClient = 0;
int failReason = 0;
bool tofTaken1 = false;
bool tofTaken2 = false;
float tofMeasure1 = -1;
float tofMeasure2 = -1;
int tofStep1 = -1;
int tofStep2 = -1;

void setup()
{
  // Run BOOT state
  Serial.begin(115200);
  Serial.printf("Starting %s...\n", CLIENT_NAME);
  WiFi.onEvent(WiFiEvent);
  lidarState.setStateChangeHandler(handleStateChange);
  lidarComms.setMessageHandler(handleMessage);
  lidarComms.setConnectionHandler(handleConnection);
  Serial.println("Boot complete, beginning startup...");
  lidarState.transitionTo(STATE_STARTUP);


}

void loop()
{
  if (systemFailed) {
    Serial.println("**************************");
    Serial.println("**************************");
    Serial.println("SYSTEM FAILED");
    Serial.printf("Reason: %d\n", failReason);
    Serial.println("**************************");
    Serial.println("**************************");
    delay(10000);
  }
  lidarComms.checkUdpPacket();
  doStateActions();
  
  // Should be minimal artificial delay eventually
  delay(1);
}

/**
 * Message displayed in the event a recoverable failure is encountered
 */
void recoverFailureMessage(int stateFrom = -1)
{
  Serial.println("**************************");
  Serial.println("**************************");
  Serial.println("RECOVERABLE FAILURE");
  if (stateFrom != -1) {
    Serial.printf("State: %d\n", stateFrom);
  }
  Serial.println("This means something didn't happen (probably within a certain timeframe).");
  Serial.println("It's not fatal, but we'll head back to startup regardless.");
  Serial.println("**************************");
  Serial.println("**************************");
}


/**
 * Startup state entry instructions refactored into method
 */
void stateStartupEntry()
{
    Serial.println("Beginning startup...");

    clientSwolConnected = false;
    clientSmolConnected = false;

    lidarState.setLedState(false, false, true);

    WiFi.softAPdisconnect();
    delay(5000);

    // Set up WiFi Access Point
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    bigbrainIp = WiFi.softAPIP();
    lidarComms.setLocalIp(bigbrainIp);
    Serial.printf("AP: %s\nIP: ", WIFI_SSID);
    
    Serial.println(bigbrainIp);

    lidarComms.startUdp();
    
    lidarState.transitionTo(STATE_AWAIT_CLIENTS);
}

/**
 * Handle when we move in/out of a state
 */
void handleStateChange(int stateTo, int stateFrom)
{
  if (DEBUG === true)
    Serial.printf("\n\nTransition from state %d to state %d\n\n", stateFrom, stateTo);
    
  // State entry
  switch (stateTo) {
    
    case STATE_STARTUP:
      stateStartupEntry();
      lidarState.transitionTo(STATE_AWAIT_CLIENTS);
      return;
    break;

    case STATE_AWAIT_CLIENTS:
      clientSwolConnected = false;
      clientSmolConnected = false;
      lidarState.setLedState(false, true, true);
    break;
    
    case STATE_REQUEST_STEP_BC:
      // On entry: request broadcast step from Swol and Smol
      lidarState.setLedState(true, false, true);
      swolCurrentStep = smolCurrentStep = -1;
      lidarComms.messageRequestBroadcastStep(0);
    break;

    case STATE_CONFIRM_STEPS:
      // On entry: set values
      lidarState.setLedState(true, true, false);
      stepFailCounter = 0;
      currentStep = swolCurrentStep;
    break;

    case STATE_SEND_TOF_POLL_CMD:
      tofPollCommandConfirmed = false;
      lidarState.setLedState(true, true, true);
      lidarComms.messageTofPollCommand(SMOL_CLIENT);
    break;

    case STATE_REQUEST_TAKE_STEP:
      stepBroadcastReceived = false;
      stepBeforeRequest = currentStep;
      // On entry: request Swol take a step
      moveToStep(currentStep + 1);
    break;

    case STATE_PROCESS_INVALID_STEP:
      stepFailCounter++;
    break;

    case STATE_REQUEST_TOF:
      tofTaken1 = tofTaken2 = false;
      tofMeasure1 = tofMeasure2 = -1;
      tofStep1 = tofStep2 = -1;
      lidarComms.messageRequestTof(SMOL_CLIENT, currentStep);
    break;

    case STATE_NEXT_STEP:
      // currentStep++;
    break;

    case STATE_RECOVERABLE_FAILURE:
      recoverFailureMessage(stateFrom);
      lidarComms.messageBroadcastSystemRestartCommand();
      return;
    break;

    case STATE_SYSTEM_FAILURE:
      // @todo: Flash a pattern that indicates the client
      // @todo: Flash red
      lidarState.setLedState(true, false, false);
    break;

    default:
    break;
  }

  // State exit
  switch (stateFrom) {
    case STATE_SEND_TOF_POLL_CMD:
      // set LEDs
      lidarState.setLedState(false, true, false);
    break;

    case STATE_REQUEST_TAKE_STEP:
      currentStep = swolCurrentStep;
    break;
  }
}

/**
 * Perform actions in the current state
 */
void doStateActions()
{

  switch (lidarState.getCurrentState()) {
    
    // ------------------------------------

    case STATE_AWAIT_CLIENTS:
      if (clientSmolConnected && clientSwolConnected) {
        lidarState.transitionTo(STATE_REQUEST_STEP_BC);
      }
    break;

    // ------------------------------------
    
    case STATE_REQUEST_STEP_BC:
      // On entry, send a position broadcast request
      if (swolCurrentStep != -1 && smolCurrentStep != -1) {
        lidarState.transitionTo(STATE_CONFIRM_STEPS);
        return;
      }
      commonStateChecks();
    break;

    // ------------------------------------

    case STATE_CONFIRM_STEPS:
      if (swolCurrentStep != -1 && smolCurrentStep == swolCurrentStep) {
        lidarState.transitionTo(STATE_SEND_TOF_POLL_CMD);
        return;
      }
      Serial.printf("%d %d\n", smolCurrentStep, swolCurrentStep);
      // if ((swolCurrentStep != -1 && smolCurrentStep != -1 && swolCurrentStep != smolCurrentStep))
      //   lidarState.transitionTo(STATE_RECOVERABLE_FAILURE);

      commonStateChecks();
    break;

    // ------------------------------------

    case STATE_SEND_TOF_POLL_CMD:
      if (tofPollCommandConfirmed)
        lidarState.transitionTo(STATE_REQUEST_TAKE_STEP);
    break;

    // ------------------------------------
    
    case STATE_REQUEST_TAKE_STEP:
      // Only track Swol's step here for now, we may need to look at Smol in the future
      if (stepBroadcastReceived && swolCurrentStep != stepBeforeRequest) {
        lidarState.transitionTo(STATE_REQUEST_TOF);
        return;
      }
      if (lidarState.isTimedOut() && (!stepBroadcastReceived || swolCurrentStep == stepBeforeRequest))
        lidarState.transitionTo(STATE_PROCESS_INVALID_STEP);

      commonStateChecks();
    break;

    // ------------------------------------

    case STATE_REQUEST_TOF:

      if (!tofTaken1 && tofMeasure1 != -1 && tofStep1 != -1)


      // if (!tofTaken2 && tofMeasure2 != -1 && tofStep2 != -1) {
      //   tofTaken2 = true;
      // }
      // ToF2 doesn't exist for now
      tofTaken2 = true;

      if (tofTaken1) {
        lidarState.transitionTo(STATE_NEXT_STEP);
        return;
      }

      commonStateChecks();
    break;

    // ------------------------------------

    case STATE_NEXT_STEP:
      lidarState.transitionTo(STATE_REQUEST_TAKE_STEP);
    break;

    // ------------------------------------

    case STATE_PROCESS_INVALID_STEP:
      if (stepFailCounter < STEP_FAIL_LIMIT)
        lidarState.transitionTo(STATE_REQUEST_TAKE_STEP);
        
      if (stepFailCounter >= STEP_FAIL_LIMIT)
        lidarState.transitionTo(STATE_RECOVERABLE_FAILURE);
        
    break;

    // ------------------------------------

    case STATE_RECOVERABLE_FAILURE:
      lidarState.transitionTo(STATE_STARTUP);
    break;

    // ------------------------------------
    
    case STATE_SYSTEM_FAILURE:
      // Not much to do here
      delay(5000);
    break;

  }

}

void commonStateChecks()
{
  if (systemFailureBroadcastReceived)
    lidarState.transitionTo(STATE_SYSTEM_FAILURE);

  if (lidarState.isTimedOut())
    lidarState.transitionTo(STATE_RECOVERABLE_FAILURE);
}

/**
 * Broadcast ID message on network. Intended to be periodic for bookkeeping.
 */
void broadcastId()
{
    if (DEBUG === true)
      Serial.println("Broadcasting ID");
      
    lidarComms.messageBroadcastId();
}

/**
 * Handle connections from LidarComms
 */
void handleConnection(int clientId)
{
  switch (clientId) {
    case SWOL_CLIENT:
      if (DEBUG === true)
        Serial.println("Swol connected!");
      clientSwolConnected = true;
    break;
    case SMOL_CLIENT:
      Serial.println("Smol connected!");
      clientSmolConnected = true;
    break;
  }
}

/**
 *  Handle incoming messages
 */
void handleMessage(int msgFrom, int msgTo, int msgDescriptor, int msgMetaData, int msgValue)
{

  if (DEBUG === true)
    Serial.printf("(Client) Message received:\n\tFrom: %d\n\tTo: %d\n\tDescriptor: %d\n\tMetaData: %d\n\tValue: %d\n\n", msgFrom, msgTo, msgDescriptor, msgMetaData, msgValue);

  // Should we listen to this message?
  if (msgTo != 0 && msgTo != CLIENT)
    return;
  
  // Decide what to do
  switch (msgDescriptor) {
    // Response to broadcast step request
    case MSG_BC_STEP:
    case MSG_RESP_BC_STEP:
      stepBroadcastReceived = true;
      if (msgFrom == SWOL_CLIENT)
        swolCurrentStep = msgValue;

      if (msgFrom == SMOL_CLIENT)
        smolCurrentStep = msgValue;

    break;

    case MSG_TOF_POLL_CMD_CONFIRM:
      tofPollCommandConfirmed = true;
    break;

    case MSG_RESP_TOF:
      tofMeasure1 = msgValue;
      tofStep1 = msgMetaData;
        tofTaken1 = true;
    break;

    // System failure somewhere in the network
    case MSG_SYSTEM_FAILURE:
      systemFailureBroadcastReceived = true;
      systemFailureClient = msgValue;
    break;
  }
}



void moveToStep(int step)
{
  if (DEBUG === true)
    Serial.printf("Telling Swol to move to step %d\n", step);
    
  // lidarComms.sendMessage(SWOL_CLIENT, MSG_STEP_COMMAND, 0, step);
  lidarComms.messageStepToCommand(SWOL_CLIENT, step);
}

/**
 * Ask Swol nicely to move a certain number of steps in either direction
 */
void moveSteps(int steps, bool forward = true)
{
  if (DEBUG === true)
    Serial.printf("Telling Swol to move %d steps\n", steps);
  // This will become lidarComms.messageStepNumCommand()
  // lidarComms.sendMessage(SWOL_CLIENT, MSG_STEP_NUM_COMMAND, (int)forward, steps);
}


/**
 * Event handler for WiFi events
 */ 
void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  lidarComms.wifiEvent(event, info);
}
