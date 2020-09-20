#include "LidarState.h"

LidarState::LidarState(long stateTimeout)
{
    this->stateTimeout = stateTimeout;
    if (LED_SETUP) {
        pinMode(LED_RED_PIN, OUTPUT);
        pinMode(LED_GREEN_PIN, OUTPUT);
        pinMode(LED_BLUE_PIN, OUTPUT);
    }
}

void LidarState::setStateChangeHandler(handleStateChange stateChangeHandler)
{
    this->stateChangeHandler = stateChangeHandler;
}


bool LidarState::transitionTo(int destinationState)
{
    if (DEBUG) {
        Serial.printf("(Lib) Request to transition to state %d\n", destinationState);
    }

    if (currentState == destinationState) {
        return false;
    }
    

    int prevPrevState = prevState;

    prevState = currentState;
    currentState = destinationState;
    timedOut = false;
    stateChangeTime = millis();

    if (stateChangeHandler) {
        stateChangeHandler(currentState, prevState);
    }

    if (DEBUG) {
        Serial.printf("(Lib) Now in state %d\n", currentState);
    }

    return true;
}

bool LidarState::isTimedOut()
{
    if (timedOut) {
        return true;
    }

    long currentTime = millis();

    if (currentTime >= (stateChangeTime + stateTimeout)) {
        timedOut = true;
    }

    return timedOut;
}

long LidarState::getStateChangeTime()
{
    return stateChangeTime;
}

int LidarState::getCurrentState()
{
    return currentState;
}

int LidarState::getPrevState()
{
    return prevState;
}

void LidarState::setLedState(bool red, bool green, bool blue)
{
    digitalWrite(LED_RED_PIN, red ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, green ? HIGH : LOW);
    digitalWrite(LED_BLUE_PIN, blue ? HIGH : LOW);
}

void LidarState::setLedOff()
{
    setLedState(false, false, false);
}