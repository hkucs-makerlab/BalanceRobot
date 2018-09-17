/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Identity.cpp
 * Author: user
 * 
 * Created on January 30, 2017, 5:49 PM
 */
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>    
#include <fcntl.h>
#ifdef __cplusplus
}
#endif 
#include "Identity.hpp"

Identity::Identity() {
}

Identity::Identity(const Identity& orig) {
}

int Identity::initName() {
    char responseBuff[64] = {0};
    int pipe = open("/etc/EddieBalance.conf", O_RDONLY);
    if (pipe == -1) {
        //DEBUG: printf("Eddie::initName: Open file for reading failed.\r\n");
        return 0;
    }
    if (read(pipe, thisEddieName, sizeof (thisEddieName))) {
        //DEBUG: printf( "Eddie::initName: set name to: %s\r\n", thisEddieName );
        return 1;
    }
    return 0;
}

void Identity::setName(char * p_name) {
    int pipe = open("/etc/EddieBalance.conf", O_WRONLY | O_CREAT);
    if (pipe == -1) {
        //DEBUG: printf("Eddie::setName: Open file for writing failed.\r\n");
        return;
    }
    if (write(pipe, p_name, sizeof (thisEddieName))) {
        strcpy(thisEddieName, p_name);
    }
}

void Identity::initIdentity() {
    if (!initName()) //If initName fails no name has been saved. Generate a unique ID to append to default name.
    {
        int thisEddieID;
        char responseBuff[256] = {0};
        FILE * pipe = popen("ls /dev/disk/by-uuid\n", "r");
        if (pipe == NULL) {
            printf("Eddie::initIdentity: Invoking command failed.\r\n");
        }
        while (fgets(responseBuff + strlen(responseBuff), sizeof (responseBuff) - strlen(responseBuff), pipe) != NULL);
        pclose(pipe);

        thisEddieID = checksum(responseBuff, strlen(responseBuff));
        sprintf(thisEddieName, "EddieBalance[%04x]", thisEddieID);
    }
}

unsigned short Identity::checksum(const char * key, int len) {
    unsigned short crc = 0xFFFF;
    int i, j;

    for (i = 0; i < len; ++i) {
        char data = key[i];
        crc = crc ^ (data << 8);
        for (j = 0; j < 8; ++j) {
            if ((crc & 0x8000) != 0) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

Identity::~Identity() {
}

