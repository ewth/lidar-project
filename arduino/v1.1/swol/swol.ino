/**
 * Swol v1.1
 * ---------
 * Swol drives the motor and polls the TOF sensor.
 */

#define CLIENT                      2         // Client ID (permanent)
#define CLIENT_NAME                 "Swol"    // Client name, only really used for display
#define SERVO_PIN                   14        // Pin for Servo signal

// -------------------------------
// DO NOT edit below here
// -------------------------------

#define STATE_STARTUP               1
#define STATE_AWAIT_POLL_CMD        2
#define STATE_POLL                  3
#define STATE_SYSTEM_RESTART        55
#define STATE_WIFI_DISCONNECTED     88
#define STATE_SYSTEM_FAILURE        99

#include "LidarComms.h"
#include "LidarState.h"
#include "DFRobot_VL53L0X.h"
#include <ESP32Servo.h>

LidarComms lidarComms = LidarComms(CLIENT, (CLIENT == 1), false);
LidarState lidarState = LidarState();
Servo servo;
DFRobotVL53L0X tofSensor;
hw_timer_t* systemFailureTimer = NULL;
int servoPosition;
bool servoReverse;
bool pollCommandReceived;
bool systemFailure;
bool systemFailureLedState;
bool systemRestartCommandReceived;

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
  Serial.begin(115200);
  Serial.printf("Starting %s...\nTarget AP: %s\n", CLIENT_NAME, lidarComms.getWifiSsid());

  WiFi.onEvent(WiFiEvent);
  lidarState.setStateChangeHandler(handleStateChange);
  lidarComms.setMessageHandler(handleMessage);

  Serial.println("Boot complete, entering startup...");
  
  lidarState.transitionTo(STATE_STARTUP);
  
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
  // State entry
  switch (stateTo) {
    case STATE_STARTUP:
      systemRestartCommandReceived = false;
      lidarState.setLedState(false, false, true);
      lidarComms.connectWifi();
      Wire.begin();
      tofSensor.begin(0x50);
      tofSensor.setMode(Continuous, High);
      tofSensor.start();
      ESP32PWM::allocateTimer(0);
      ESP32PWM::allocateTimer(1);
      ESP32PWM::allocateTimer(2);
      ESP32PWM::allocateTimer(3);
      servo.setPeriodHertz(50);
      servo.attach(SERVO_PIN, 800, 2200); // MG995
    break;

    case STATE_AWAIT_POLL_CMD:
      pollCommandReceived = false;
      lidarComms.messageBroadcastId();
      lidarState.setLedState(false, true, true);
    break;

    case STATE_POLL:
    break;

    case STATE_SYSTEM_RESTART:
      lidarState.setLedState(true, true, true);
    break;

    case STATE_WIFI_DISCONNECTED:
      lidarState.setLedState(true, false, false);
      lidarComms.disconnectWifi();
      delay(5000);
      lidarState.transitionTo(STATE_STARTUP);
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
      case STATE_AWAIT_POLL_CMD:
        lidarState.setLedState(false, true, false);
        lidarComms.messageBroadcastPollConfirm();
      break;
  }
}

/**
 * Perform actions in the current state
 */
void doStateActions()
{
  switch (lidarState.getCurrentState()) {
    case STATE_STARTUP:
      // @TODO: Put the hardware check back in!
      if (lidarComms.isConnected() && tofSensor.getDistance() < 16000) {
        lidarState.transitionTo(STATE_AWAIT_POLL_CMD);
        return;
      }
      
      if (lidarState.isTimedOut()) {
        if (tofSensor.getDistance() > 16000) {
          lidarState.transitionTo(STATE_SYSTEM_FAILURE);
          return;
        }
        lidarState.transitionTo(STATE_WIFI_DISCONNECTED);
      }
    break;
    case STATE_AWAIT_POLL_CMD:
      if (pollCommandReceived) {
        lidarState.transitionTo(STATE_POLL);
        return;
      }

      commonStateChecks();
    break;

    case STATE_POLL:
      doPolling();
      commonStateChecks();
    break;
  }
}


/**
 * Common checks performed in different states
 */ 
void commonStateChecks()
{
  if (systemRestartCommandReceived)
    lidarState.transitionTo(STATE_SYSTEM_RESTART);

  if (systemFailure)
    lidarState.transitionTo(STATE_SYSTEM_FAILURE);

  if (!lidarComms.isConnected())
    lidarState.transitionTo(STATE_WIFI_DISCONNECTED);
}


/**
 * Step the servo and take polling data
 */ 
void doPolling()
{
  if (servoPosition > 180) {
    servoPosition = 180;
    servoReverse = true;
  }

  if (servoPosition < 0) {
    servoPosition = 0;
    servoReverse = false;
  }

  servo.write(servoPosition);

  delay(10);

  
  int distance = (int)tofSensor.getDistance();

  lidarComms.messageBroadcastPollResult(servoPosition, distance);

    if (servoReverse) {
    servoPosition--;
  } else {
    servoPosition++;
  }

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
    case MSG_POLL_CMD:
      pollCommandReceived = true;
    break;

    case MSG_SYSTEM_RESTART_CMD:
      systemRestartCommandReceived = true;
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
