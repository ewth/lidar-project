#ifndef LIDARCOMMS_H
#define LIDARCOMMS_H

#define WIFI_SSID                 "LIDAR319"        // AP SSID
#define WIFI_PASSWORD             "ItsBigBrainTime" // AP Password
#define AUTO_DESTINATION          true              // Auto attach to gateway
#define MESSAGE_SIZE              20                // Size of messages to be expected
#define PORT                      21337             // UDP Port
#define BUFFER_SIZE               255               // Size of buffer to read at a time from UDP stack
#define BROADCAST_ID              false             // Whether to broadcast ID on network
#define BROADCAST_INTERVAL        10000             // Interval between broadcasting ID
#define RECONNECT_INTERVAL         1000              // Interval between reconnection attempts

#define MSG_ID                    1           
#define MSG_CLIENT_INFO           5
#define MSG_POLL_CMD              10
#define MSG_POLL_CONFIRM          20
#define MSG_POLL_RESULT           30
#define MSG_STOP_CMD              40
#define MSG_SYSTEM_RESTART_CMD    77
#define MSG_SYSTEM_FAILURE        99

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
        bool messageBroadcastPollCommand();
        bool messageBroadcastPollConfirm();
        bool messageBroadcastPollResult(int position, int distance);
        bool messageBroadcastStopCommand();
        bool messageBroadcastSystemRestartCommand(int reason = 0);
        bool messageBroadcastSystemFailure(int reason = 0);

};

#endif