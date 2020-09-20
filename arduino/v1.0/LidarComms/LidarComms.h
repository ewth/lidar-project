#ifndef LIDARCOMMS_H
#define LIDARCOMMS_H

#define WIFI_SSID                 "LIDAR319"        // AP SSID
#define WIFI_PASSWORD             "ItsBigBrainTime" // AP Password
#define AUTO_DESTINATION          true              // Auto attach to gateway
#define DEGREE_STEP               0.1               // Approx. how many degrees per step (for directing Swol from Smol)
#define BEARING_INTERVAL          30000             // How long to repeat a bearing calibration if Smol reconnects
#define BEARING_ZERO              90.0              // Bearing to start at
#define BEARING_TOLERANCE         0.1               // Tolerance when checking bearing
#define BEARING_MAX_ATTEMPTS      10                // Max adjustments to attempt in zero bearing
#define MESSAGE_SIZE              20                // Size of messages to be expected
#define PORT                      21337             // UDP Port
#define RECONNECT_INTERVAL        3000              // Interval (ms) between WiFi reconnection attempts
#define BUFFER_SIZE               255               // Size of buffer to read at a time from UDP stack
#define BROADCAST_ID              false             // Whether to broadcast ID on network
#define BROADCAST_INTERVAL        10000             // Interval between broadcasting ID
#define BIGBRAIN_CLIENT           1
#define SWOL_CLIENT               2
#define SMOL_CLIENT               3

#define MSG_ID                          1           // Identification message
#define MSG_CLIENT_INFO                 5
#define MSG_STEP_CMD                    10
#define MSG_STEP_NEXT_CMD               11
#define MSG_STEP_TO_CMD                 12
#define MSG_REQ_BC_STEP                 15
#define MSG_RESP_BC_STEP                16
#define MSG_REQ_STEP                    17
#define MSG_RESP_STEP                   18
#define MSG_BC_STEP                     19
#define MSG_REQ_TOF                     20
#define MSG_RESP_TOF                    21
#define MSG_TOF_POLL_CMD                22
#define MSG_TOF_POLL_CMD_CONFIRM        23
#define MSG_RESP_TOF_1                  25
#define MSG_RESP_TOF_2                  26
#define MSG_SYSTEM_RESTART_CMD          77
#define MSG_SYSTEM_FAILURE              99

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>


class LidarComms {
    typedef void (*handleMessageCallback)(int from, int to, int descriptor, int metaData, int value);
    typedef void (*handleClientConnection)(int clientId);
    
    private:
        int clientId;
        bool brain;
        bool debugMode;
        bool connected;

        int clientIps[10];

        char *clientName;

        long lastConnectionTime;
        long lastMessageTime;

        IPAddress localIp;
        IPAddress destination; // Defaults to client 1 (Bigbrain)

        int decompileMessage(char *message, int msgStart);

        WiFiUDP udp;

        handleMessageCallback messageHandler;
        handleClientConnection connectionHandler;
        handleClientConnection disconnectionHandler;

        bool sendMessageBroadcast(int descriptor, int metaData, int value);
        bool sendMessage(int to, int descriptor, int metaData, int value);
        bool sendMessageToIp(IPAddress ipTo, int to, int descriptor, int metaData, int value);

        void wifiConnectedEvent();
        void wifiDisconnectedEvent();

    public:
        LidarComms(int clientId, bool isBrain, bool debugMode);

        void checkUdpPacket();

        void handleMessage(IPAddress remoteIp, char *message, int length);
        bool sayHello();
        bool sayHelloBack(int to);

        void addClientInfo(int clientId, int ipSegment);

        void wifiEvent(WiFiEvent_t event, WiFiEventInfo_t info);

        void broadcastId();
        IPAddress getClientIp(int clientId);
        bool connectWifi();
        bool disconnectWifi();
        void startUdp();

        void setLocalIp(IPAddress localIp);

        void setMessageHandler(handleMessageCallback messageHandler);
        void setConnectionHandler(handleClientConnection connectionHandler);
        void setDisconnectionHandler(handleClientConnection disconnectionHandler);

        char *getWifiSsid();
        bool isConnected();
        int getClientId();
        bool isClientConnected(int clientId);
        long getLastMessageTime();

        bool messageId(int to);
        bool messageBroadcastId();
        bool messageClientInfo(int to);
        // bool messageStepCommand(int to);
        // bool messageStepNextCommand(int to);
        bool messageStepToCommand(int to, int step);
        bool messageBroadcastStep(int currentStep);
        bool messageRequestBroadcastStep(int to = 0);
        bool messageResponseBroadcastStep(int currentStep);
        bool messageRequestStep(int to);
        bool messageResponseStep(int to, int currentStep);
        bool messageRequestTof(int to, int step);
        bool messageResponseTof(int to, int step, float tof);
        bool messageTofPollCommand(int to);
        bool messageConfirmTofPollCommand(int to);
        // bool messageResponseTof1(int to);
        // bool messageResponseTof2(int to);
        // bool messageRequestBearing(int to);
        // bool messageResponseBearing(int to);
        bool messageBroadcastSystemRestartCommand(int reason = 0);
        bool messageBroadcastSystemFailure(int reason = 0);

};

#endif