#include "LidarComms.h"

struct Message {
    int From;
    int To;
    int Descriptor;
    int MetaData;
    int Value;
};

LidarComms::LidarComms(int clientId, bool isBrain = false, bool debugMode = false)
{
    this->clientId = clientId;
    this->brain = isBrain; 
    this->debugMode = debugMode;
}

/**
 * Check if a packet exists on the UDP stack and if so, handle it
 */
void LidarComms::checkUdpPacket()
{
    // My assumption here is that parsePacket() essentially parses a single message, i.e. one message handled per call
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) {
        return;
    }

    IPAddress remoteIp = udp.remoteIP();
    if (debugMode) {
        Serial.printf("Packet received: %d bytes from ", packetSize);
        Serial.println(remoteIp);
    }
    char packetBuffer[BUFFER_SIZE];
    int readLen = udp.read(packetBuffer, BUFFER_SIZE);
    if (!localIp) {
        if (brain) 
            localIp = WiFi.softAPIP();
    }
    handleMessage(remoteIp, packetBuffer, readLen);
    
}

/**
 * Send messages to all clients (broadcast)
 */
bool LidarComms::sendMessageBroadcast(int descriptor, int metaData, int value)
{
    sendMessage(0, descriptor, metaData, value);
}

/**
 * Send message to a specific client
 */
bool LidarComms::sendMessage(int to, int descriptor, int metaData, int value)
{
    if (to != 0) {
        IPAddress dest = getClientIp(to);
        if (debugMode) {
            Serial.printf("Client ID %d resolved to IP ", to);
            Serial.println(dest);
        }
        if (dest) {
            return sendMessageToIp(dest, to, descriptor, metaData, value);
        }
        
    }
    // Otherwise, broadcast
    return sendMessageToIp(IPAddress {255,255,255,255}, to, descriptor, metaData, value);

}

/**
 * Send a message to a specific IP address 
 */ 
bool LidarComms::sendMessageToIp(IPAddress ipTo, int to, int descriptor, int metaData, int value)
{
    Message message = { .From = clientId , .To = to, .Descriptor = descriptor , .MetaData = metaData , .Value = value };
    if (debugMode) {
        Serial.printf("Sending %d message to (%d) => ", descriptor, to);
        Serial.println(ipTo);
    }
    udp.beginPacket(ipTo, PORT);
    udp.write((byte*)&message, sizeof message);
    udp.endPacket();
    return true;
}

/**
 *  Handle incoming messages
 */
void LidarComms::handleMessage(IPAddress remoteIp, char *message, int length) {
    if (length < MESSAGE_SIZE) {
        if (debugMode)
            Serial.printf("Message discarded for being undersized: %d\n", length);
        return;
    }

    lastMessageTime = millis();

    // TODO: Tidy this up
    int msgFrom = decompileMessage(message,0);
    int msgTo = decompileMessage(message,4);
    int msgDescriptor = decompileMessage(message,8);
    int msgMetaData = decompileMessage(message,12);
    int msgValue = decompileMessage(message,16);

    if (debugMode)
        Serial.printf("(Lib) Message received:\n\tFrom: %d\n\tTo: %d\n\tDescriptor: %d\n\tMetaData: %d\n\tValue: %d\n\n", msgFrom, msgTo, msgDescriptor, msgMetaData, msgValue);

    if (msgTo != clientId && msgTo != 0)
        return;

    switch (msgDescriptor) {
        // Identification message
        case MSG_ID:
            addClientInfo(msgFrom, remoteIp[3]);
            if (msgMetaData == 1) {
                // Saying hello, so say hello back
                sayHelloBack(msgFrom);
            }
        break;
        // Client information message
        case MSG_CLIENT_INFO:
            addClientInfo(msgValue, msgMetaData);
        break;
    }

    // Hand over to client
    if (messageHandler) {
        messageHandler(msgFrom, msgTo, msgDescriptor, msgMetaData, msgValue);
    }
}

/**
 * Broadcast ID on network (say "Hello")
 */ 
bool LidarComms::sayHello()
{
    if (!connected)
        return false;
    
    Serial.printf("-> Hello, I am client %d\n", clientId);
    return sendMessageBroadcast(MSG_ID, 1, clientId);
}

/**
 * Respond to a hello request
 */ 
bool LidarComms::sayHelloBack(int to)
{
    if (!brain && !connected)
        return false;
    
    Serial.printf("-> Hello back %d! I am client %d\n", to, clientId);

    if (connectionHandler) {
        if (debugMode) {
            Serial.println("Firing connection handler");
        }
        connectionHandler(to);
    }

    return sendMessage(to, MSG_ID, 2, clientId);
}

/**
 * Add info about a remote client
 */ 
void LidarComms::addClientInfo(int clientId, int ipSegment)
{
    // Don't track ourselves
    if (clientId == this->clientId)
        return;
    
    if (debugMode)
        Serial.printf("Received info on client %d, IP segment %d\n", clientId, ipSegment);

    clientIps[clientId] = ipSegment;
}



/**
 * This should be called by parent when event raised.
 */ 
void LidarComms::wifiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    if (debugMode)
        Serial.printf("WiFi event %d raised\n", event);

    switch(event) {
        case SYSTEM_EVENT_STA_GOT_IP:
            wifiConnectedEvent();
        break;

        case SYSTEM_EVENT_STA_DISCONNECTED:
            wifiDisconnectedEvent();
        break;

        default: break;
    }
    
}

void LidarComms::wifiConnectedEvent()
{
    if (brain) {
        Serial.println("Client connected.");
        return;
    }

    connected = true;

    if (AUTO_DESTINATION)
        destination = WiFi.gatewayIP();

    localIp = WiFi.localIP();
    udp.begin(localIp,PORT);
    if (debugMode) {
        Serial.print("WiFi connected! IP address: ");
        Serial.println(localIp);  
        Serial.print("Attaching to destination: ");
        Serial.println(destination);
    }
    
    sayHello();
}

/**
 * This should be called by parent when event raised.
 */ 
void LidarComms::wifiDisconnectedEvent()
{
    if (brain) {
        Serial.println("Client lost connection.");
        return;
    }

    connected = false;
    Serial.println("WiFi lost connection.");
}

/**
 * Construct an IP address from our local IP and the last segment of the client if stored
 */
IPAddress LidarComms::getClientIp(int clientId)
{
    if (debugMode) {
        Serial.printf("Getting client IP for %d with base ", clientId);
        Serial.println(localIp);
    }
    if (localIp && clientIps[clientId] != NULL && clientIps[clientId] != 0) {
        IPAddress clientIp = {localIp[0], localIp[1], localIp[2], clientIps[clientId]};
        return clientIp;
    }
    return (0,0,0,0);
}

/**
 * Establish a WiFi connection
 */ 
bool LidarComms::connectWifi()
{
    if (connected || brain) {
        return true;
    }


    long currentTime = millis();
    if ((currentTime - lastConnectionTime) < RECONNECT_INTERVAL) {
        return false;
    }
    
    lastConnectionTime = currentTime;
    
    disconnectWifi();

    Serial.print("Connecting to Wifi on ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    return true;
}

/**
 * Disconnect from Wifi.
 * @TODO: Should we raise an event here?
 */
bool LidarComms::disconnectWifi()
{
    WiFi.disconnect();
    return true;
}

/**
 * Listen for UDP messages
 */ 
void LidarComms::startUdp()
{
    udp.begin(PORT);
    Serial.printf("Ready for UDP messages on port %d.\n\n", PORT);
}

/**
 * Set IP address. Should only be used by Brain
 */ 
void LidarComms::setLocalIp(IPAddress localIp)
{
    this->localIp = localIp;
}

/**
 * Set callback to handle messages
 */ 
void LidarComms::setMessageHandler(handleMessageCallback messageHandler)
{
    this->messageHandler = messageHandler;
}

/**
 * Set callback to handle connections
 */
void LidarComms::setConnectionHandler(handleClientConnection connectionHandler)
{
    this->connectionHandler = connectionHandler;
} 

/**
 * Set callback to handle disconnections
 */
void LidarComms::setDisconnectionHandler(handleClientConnection disconnectionHandler)
{
    this->disconnectionHandler = connectionHandler;
} 

/**
 * 
 */
char *LidarComms::getWifiSsid()
{
    return WIFI_SSID;
}

/**
 * 
 */
bool LidarComms::isConnected()
{
    return connected;
}

/**
 * 
 */
int LidarComms::getClientId()
{
    return clientId;
}

long LidarComms::getLastMessageTime()
{
    return lastMessageTime;
}

/**
 * Send ID to specific client
 */
bool LidarComms::messageId(int to)
{
    return sendMessage(to, MSG_ID, 0, clientId);
}

/**
 * Broadcast ID to network
 */
bool LidarComms::messageBroadcastId()
{
    return sendMessageBroadcast(MSG_ID, 0, clientId);
}

/**
 * Message info about all known clients
 */
bool LidarComms::messageClientInfo(int to)
{
    for (int i = 0; i < sizeof(clientIps)/sizeof(clientIps[0]); i++) {
        int ipSegment = clientIps[i];
        if (ipSegment != 0)
            sendMessage(to, MSG_CLIENT_INFO, ipSegment, i);
    }
    return true;
}

/**
 * Send a client a command to start polling
 */ 
bool LidarComms::messageBroadcastPollCommand()
{
    return sendMessage(0, MSG_POLL_CMD, 0, 0);
}

/**
 * Confirm a command to start polling
 */ 
bool LidarComms::messageBroadcastPollConfirm()
{
    return sendMessage(0, MSG_POLL_CONFIRM, 0, 0);
}

/**
 * Broadcast a polling result
 */ 
bool LidarComms::messageBroadcastPollResult(int position, int distance)
{
    return sendMessage(0, MSG_POLL_RESULT, position, distance);
}

/**
 * Broadcast a stop command
 */ 
bool LidarComms::messageBroadcastStopCommand()
{
    return sendMessage(0, MSG_STOP_CMD, 0, 0);
}

/**
 * Broadcast system restart command (brain only)
 */
bool LidarComms::messageBroadcastSystemRestartCommand(int reason)
{
    if (!brain) {
        return false;
    }
    return sendMessageBroadcast(MSG_SYSTEM_RESTART_CMD, 0, reason);
}

/**
 * Broadcast unrecoverable system failure
 */ 
bool LidarComms::messageBroadcastSystemFailure(int reason)
{
    return sendMessageBroadcast(MSG_SYSTEM_FAILURE, 0, reason);
}

/**
 * Decompile message bytes
 */
int LidarComms::decompileMessage(char *message, int msgStart) {
    byte b[4];
    for (int i = msgStart; i < (msgStart + 4); i++) {
        b[(i - msgStart)] = message[i];
    }
    
    return (b[3] << 24) | (b[2] << 16) | (b[1] << 8) | b[0];
}

