/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ServoClass.hpp
 * Author: user
 *
 * Created on March 22, 2017, 9:44 AM
 */

#ifndef SERVOCLASS_HPP
#define SERVOCLASS_HPP
#include "SoftPWM.hpp"

class ServoClass : public SoftPWM {
public:
    ServoClass();
    ServoClass(const ServoClass& orig);
    ServoClass(int gpio);
    void setAngle(int angle);
    
    virtual ~ServoClass();
private:

};

#endif /* SERVOCLASS_HPP */

