/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Identity.hpp
 * Author: user
 *
 * Created on January 30, 2017, 5:49 PM
 */

#ifndef IDENTITY_HPP
#define IDENTITY_HPP
#include <cstdio>

class Identity {
public:
    Identity();
    Identity(const Identity& orig);
    int initName();
    void setName(char * p_name);
    void initIdentity();
    unsigned short checksum(const char * key, int len);

    virtual ~Identity();
     char thisEddieName[64];
private:
   
};

#endif /* IDENTITY_HPP */

