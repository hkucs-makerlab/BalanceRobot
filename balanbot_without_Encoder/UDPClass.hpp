/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   UDPClass.hpp
 * Author: user
 *
 * Created on January 31, 2017, 10:48 AM
 */

#ifndef UDPCLASS_HPP
#define UDPCLASS_HPP
#ifdef __cplusplus
extern "C" {
#endif
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/prctl.h>
#include <signal.h>
#include <pthread.h>

#ifdef __cplusplus
}
#endif 


extern "C" void* udplistenerThreadHandler(void * arg);
class UDPClass {
    friend void* udplistenerThreadHandler(void * arg);
public:
    UDPClass();
    UDPClass(const UDPClass& orig);
    void initUDP(void (*p_cmdFuncPtr)(char*), void (*p_ctrlFuncPtr)(char*));
    void UDPCloseTX();
    void UDPCloseCtrlTX();
    void initUDPCmdSend(char * sendtoIP, unsigned short sendtoPort);
    void setCommandBindAddress();
    void initUDPCtrlSend(char * sendtoIP, unsigned short sendtoPort);
    void UDPBindSend(char * data, int len);
    void UDPCtrlSend(char * data);
    void initListener(unsigned short udpListenPort, int * p_socket, struct sockaddr_in * p_addr);
    int checkUDPReady(char * udpBuffer, int * p_socket);

    void stop() {
        isRunning = false;
        if (udplistenerThread)
            pthread_join(*udplistenerThread, NULL);
    }

    virtual ~UDPClass();
private:
    static UDPClass *me;
    char lastRXAddress[16]; //The last address a UDP message was received from
    char commandBindAddress[16]; //This is the address we are bound to receive commands from
    int isBoundToClient;
    int tx_command_socketfd; //Socket descriptor used to send data to bound client
    int tx_control_socketfd; //Socket descriptor used to response to control packets
    int rx_command_socketfd; //Socket descriptor used to receive commands from user
    int rx_control_socketfd; //Socket descriptor used to receive control data from user

    struct sockaddr_in tx_cmd_addrin;
    struct sockaddr_in tx_ctrl_addrin;
    struct sockaddr_in rx_cmd_addrin;
    struct sockaddr_in rx_ctrl_addrin;

    void (*commandFunctionPtr)(char *);
    void (*controlFunctionPtr)(char *);
    pthread_t *udplistenerThread;
    bool isRunning;
};

#endif /* UDPCLASS_HPP */

