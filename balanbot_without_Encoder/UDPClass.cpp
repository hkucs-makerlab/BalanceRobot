
/* 
 * File:   UDPClass.cpp
 * Author: user
 * 
 * Created on January 31, 2017, 10:48 AM
 */
#include <cstdio>
#include "UDPClass.hpp"

#define MAXMESSAGESIZE 64

#define UDP_COMMAND_PORT 4242 //UDP Port for receiving command packets
#define UDP_CONTROL_PORT 4240 //UDP Port for receiving control packets
#define UDP_RESPOND_PORT 4243 //UDP Port for returning data to user



UDPClass::UDPClass() :
udplistenerThread(nullptr),
isRunning(true),
tx_command_socketfd(-1),
tx_control_socketfd(-1),
isBoundToClient(0), lastRXAddress{0}, commandBindAddress{0}
{

}

UDPClass::UDPClass(const UDPClass& orig) {
}

void UDPClass::initUDP(void (*p_cmdFuncPtr)(char*), void (*p_ctrlFuncPtr)(char*)) {
    commandFunctionPtr = p_cmdFuncPtr;
    controlFunctionPtr = p_ctrlFuncPtr;
    //pthread_create(udplistenerThread, NULL, &udplistenerThreadHandler, NULL);
}

void UDPClass::setCommandBindAddress() {
    //Set the bind address to the last address received from
    strcpy(commandBindAddress, lastRXAddress);

    //Init the TX command socket with the new bind address
    initUDPCmdSend(commandBindAddress, UDP_RESPOND_PORT);

    isBoundToClient = 1;
}

void UDPClass::initUDPCtrlSend(char * sendtoIP, unsigned short sendtoPort) {
    tx_control_socketfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP); // Create socket for sending
    memset(&tx_ctrl_addrin, 0, sizeof ( tx_ctrl_addrin)); // Zero out structure
    tx_ctrl_addrin.sin_family = AF_INET; // Internet address family
    tx_ctrl_addrin.sin_addr.s_addr = inet_addr(sendtoIP); // Destination IP address
    tx_ctrl_addrin.sin_port = htons(sendtoPort); // Destination port
}

void UDPClass::initUDPCmdSend(char * sendtoIP, unsigned short sendtoPort) {
    tx_command_socketfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP); // Create socket for sending
    memset(&tx_cmd_addrin, 0, sizeof ( tx_cmd_addrin)); // Zero out structure
    tx_cmd_addrin.sin_family = AF_INET; // Internet address family
    tx_cmd_addrin.sin_addr.s_addr = inet_addr(sendtoIP); // Destination IP address
    tx_cmd_addrin.sin_port = htons(sendtoPort); // Destination port
}

void UDPClass::UDPBindSend(char * data, int len) {
    if (tx_command_socketfd >= 0 && isBoundToClient) {
        sendto(tx_command_socketfd, data, len, 0, (struct sockaddr *) &tx_cmd_addrin, sizeof ( tx_cmd_addrin));
    }
}

void UDPClass::UDPCtrlSend(char * data) {
    if (tx_control_socketfd >= 0) {
        sendto(tx_control_socketfd, data, strlen(data), 0, (struct sockaddr *) &tx_ctrl_addrin, sizeof ( tx_ctrl_addrin));
    }
}

void UDPClass::initListener(unsigned short udpListenPort, int * p_socket, struct sockaddr_in * p_addr) {
    *p_socket = socket(AF_INET, SOCK_DGRAM, 0);
    bzero(p_addr, sizeof ( *p_addr));
    (*p_addr).sin_family = AF_INET;
    (*p_addr).sin_addr.s_addr = htonl(INADDR_ANY);
    (*p_addr).sin_port = htons(udpListenPort);
    bind(*p_socket, (struct sockaddr *) p_addr, sizeof ( *p_addr));
}

int UDPClass::checkUDPReady(char * udpBuffer, int * p_socket) {
    int bytesAv = 0;

    /* If there is data to be read on the socket bring it in and capture the source IP */
    if (ioctl(*p_socket, FIONREAD, &bytesAv) > 0 || bytesAv > 0) {
        int udpMsgLen = 0;
        struct sockaddr_in rx_from_addr;
        socklen_t len = sizeof ( rx_from_addr);
        //Receive UDP data
        udpMsgLen = recvfrom(*p_socket, udpBuffer, MAXMESSAGESIZE, 0, (struct sockaddr *) &rx_from_addr, &len);
        udpBuffer[ udpMsgLen ] = 0; //Null terminate UDP RX string

        //Get address from this received packet
        char thisRXaddress[16] = {0};
        sprintf(thisRXaddress, "%s", inet_ntoa(rx_from_addr.sin_addr));

        //If this RX address does not match the last RX address && is a control packet...
        if (p_socket == &rx_control_socketfd && memcmp(lastRXAddress, thisRXaddress, sizeof (lastRXAddress)) != 0) {
            UDPCloseCtrlTX(); //...close the control TX socket
            initUDPCtrlSend(thisRXaddress, UDP_RESPOND_PORT); //and re-open with the address we need to respond to
        }

        //Store the last RX address
        strcpy(lastRXAddress, thisRXaddress);

        return 1;
    }

    return 0;
}

void UDPClass::UDPCloseTX() {
    close(tx_command_socketfd);
    tx_command_socketfd = -1;
}

void UDPClass::UDPCloseCtrlTX() {
    close(tx_control_socketfd);
    tx_control_socketfd = -1;
}

UDPClass::~UDPClass() {
}

void* udplistenerThreadHandler(void * arg) {
    
    UDPClass *udpclass = (UDPClass *) arg;

    prctl(PR_SET_NAME, "updlistener", 0, 0, 0);
    udpclass->initListener(UDP_COMMAND_PORT, &udpclass->rx_command_socketfd, &udpclass->rx_cmd_addrin);
    udpclass->initListener(UDP_CONTROL_PORT, &udpclass->rx_control_socketfd, &udpclass->rx_ctrl_addrin);

    char incomingUDP[ MAXMESSAGESIZE + 1 ];

    while (udpclass->isRunning) {
        usleep(20000); //Give this thread a break between iterations to keep CPU usage down

        /* Check for UDP data on Control port */
        while (udpclass->checkUDPReady(incomingUDP, & udpclass->rx_control_socketfd)) {
            printf("%s","check udp");
            (*udpclass->controlFunctionPtr)(incomingUDP);
            bzero(incomingUDP, sizeof ( incomingUDP));
        }

        /* Check for UDP data on Command port */
        while (udpclass->checkUDPReady(incomingUDP, & udpclass->rx_command_socketfd)) {
            if (udpclass->isBoundToClient && memcmp(udpclass->lastRXAddress, udpclass->commandBindAddress, sizeof ( udpclass->lastRXAddress)) == 0) {
                (*udpclass->commandFunctionPtr)(incomingUDP);
            }
            bzero(incomingUDP, sizeof (incomingUDP));
        }
    }

    return NULL;
}
