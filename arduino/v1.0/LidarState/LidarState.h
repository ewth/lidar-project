#ifndef LIDARSTATE_H
#define LIDARSTATE_H

#include <Arduino.h>

#define DEBUG           true

#define LED_SETUP       true
#define LED_RED_PIN     0
#define LED_GREEN_PIN   2
#define LED_BLUE_PIN    4

class LidarState {

    typedef void (*handleStateChange)(int stateTo, int stateFrom);
    
    private:
        int prevState;
        int currentState;
        long stateTimeout;
        long stateChangeTime;
        bool timedOut;
        handleStateChange stateChangeHandler;

    public:
        LidarState(long stateTimeout = 10000);
        void setStateChangeHandler(handleStateChange stateChangeHandler);
        bool transitionTo(int destinationState);
        bool isTimedOut();
        long getStateChangeTime();
        int getCurrentState();
        int getPrevState();
        
        void setLedState(bool red, bool green, bool blue);
        void setLedOff();

};

#endif

