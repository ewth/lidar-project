/**
 * Bigbrain v1.1
 * -------------
 * Bigbrain sets up a Wifi AP to allow comms, and feeds data back to the PC via serial.
 */

#define CLIENT                      1                 // Client number (permanent)
#define CLIENT_NAME                 "Bigbrain"        // Client name, only really used for display

// -------------------------------
// DO NOT edit below here
// -------------------------------

#include "LidarComms.h"
#include "LidarState.h"

#define STATE_STARTUP               1
#define STATE_AWAIT_CLIENT          2
#define STATE_SEND_POLL_CMD         3
#define STATE_RECV_POLL_DATA        4
#define STATE_NEW_BC_ID             5
#define STATE_RECOVERABLE_FAILURE   77
#define STATE_SYSTEM_FAILURE        99

LidarComms lidarComms = LidarComms(CLIENT, (CLIENT == 1), false);
LidarState lidarState = LidarState();

bool clientConnected;
bool pollCommandConfirmed;
int clientDistance;
int clientStep;
bool systemFailureLedState;
bool systemFailed;
bool broadcastIdReceived;
hw_timer_t* systemFailureTimer = NULL;

/**
 * Timer interrupt to flash red LED in case of system failure
 */ 
void IRAM_ATTR systemFailureIsr()
{
  systemFailureLedState = !systemFailureLedState;
  lidarState.setLedState(systemFailureLedState, false, false);
}

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

    clientConnected = false;
    
    lidarState.setLedState(false, false, true);

    WiFi.softAPdisconnect();
    delay(5000);

    // Set up WiFi Access Point
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("AP: %s\nIP: ", WIFI_SSID);
    
    Serial.println(WiFi.softAPIP());

    lidarComms.startUdp();
}

/**
 * Handle when we move in/out of a state
 */
void handleStateChange(int stateTo, int stateFrom)
{
    
  // State entry
  switch (stateTo) {
    
    case STATE_STARTUP:
      stateStartupEntry();
    break;

    case STATE_AWAIT_CLIENT:
      lidarComms.messageBroadcastSystemRestartCommand();
      lidarState.setLedState(false, true, true);
      broadcastIdReceived = false;
      clientConnected = false;
    break;

    case STATE_SEND_POLL_CMD:
      pollCommandConfirmed = false;
      lidarState.setLedState(true, false, true);
      lidarComms.messageBroadcastPollCommand();
    break;

    case STATE_RECV_POLL_DATA:
      broadcastIdReceived = false;
      clientStep = -1;
      clientDistance = -1;
      lidarState.setLedState(false, true, false);
    break;

    case STATE_RECOVERABLE_FAILURE:
      lidarComms.messageBroadcastSystemRestartCommand();
    break;

    case STATE_SYSTEM_FAILURE:
      lidarComms.messageBroadcastSystemFailure();
      systemFailureTimer = timerBegin(0, 40, true);
      timerAttachInterrupt(systemFailureTimer, &systemFailureIsr, true);
      timerAlarmWrite(systemFailureTimer, 1000000, true);
      timerAlarmEnable(systemFailureTimer);
    break;

  }

  // State exit
  switch (stateFrom) {

  }
}

/**
 * Perform actions in the current state
 */
void doStateActions()
{

  switch (lidarState.getCurrentState()) {
    case STATE_STARTUP:
        lidarState.transitionTo(STATE_AWAIT_CLIENT);
    break;

    case STATE_AWAIT_CLIENT:
      if (clientConnected)
        lidarState.transitionTo(STATE_SEND_POLL_CMD);

    break;

    case STATE_SEND_POLL_CMD:
      if (pollCommandConfirmed)
        lidarState.transitionTo(STATE_RECV_POLL_DATA);

      if (systemFailed)
        lidarState.transitionTo(STATE_SYSTEM_FAILURE);

      if (lidarState.isTimedOut())
        lidarState.transitionTo(STATE_RECOVERABLE_FAILURE);

    break;
    
    case STATE_RECV_POLL_DATA:
      if (broadcastIdReceived)
        lidarState.transitionTo(STATE_NEW_BC_ID);
    break;

    case STATE_NEW_BC_ID:
      lidarState.transitionTo(STATE_STARTUP);
    break;

    case STATE_RECOVERABLE_FAILURE:
      lidarState.transitionTo(STATE_STARTUP);
    break;
    
    case STATE_SYSTEM_FAILURE:
      // Not much to do here
      delay(5000);
    break;
  }
}


/**
 * Broadcast ID message on network. Intended to be periodic for bookkeeping.
 */
void broadcastId()
{
    lidarComms.messageBroadcastId();
}

/**
 * Handle connections from LidarComms
 */
void handleConnection(int clientId)
{
  clientConnected = true;
}

/**
 *  Handle incoming messages
 */
void handleMessage(int msgFrom, int msgTo, int msgDescriptor, int msgMetaData, int msgValue)
{

  // Should we listen to this message?
  if (msgTo != 0 && msgTo != CLIENT)
    return;
  
  // Decide what to do
  switch (msgDescriptor) {
    case MSG_ID:
      broadcastIdReceived = true;
    break;

    case MSG_POLL_CONFIRM:
      pollCommandConfirmed = true;
    break;

    case MSG_POLL_RESULT:
      Serial.printf("[POLL:%d,%d]", msgMetaData, msgValue);
    break;
  }
}

/**
 * Event handler for WiFi events
 */ 
void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  lidarComms.wifiEvent(event, info);
}
