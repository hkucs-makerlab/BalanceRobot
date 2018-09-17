/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   LEDClass.hpp
 * Author: user
 *
 * Created on March 22, 2017, 10:51 AM
 */

#ifndef LEDCLASS_HPP
#define LEDCLASS_HPP
#include "SoftPWM.hpp"

class LEDClass : public SoftPWM {
public:
    LEDClass();
    LEDClass(const LEDClass& orig);
    LEDClass(int gpio);
    virtual ~LEDClass();
private:

};

#endif /* LEDCLASS_HPP */

