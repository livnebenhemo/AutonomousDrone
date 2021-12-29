#define ORB_SLAM2_GPIOADD_H
#include <iostream>
#include <wiringPi.h>
#include <unistd.h>


class gpio{
private:
    int button_pin = 32;
    int sense_pin = 40;
    int led_pin  = 22;
public:
    const int& getButtonPin();
    const int& getSensePin();
    const int& getLedPin();
    void setButtonPin(const int& toSet);
    void setSensePin(const int& toSet);
    void setLedPin(const int& toSet);
    void cleanUpForCharge();
    void toCharge();
    void SetUps();
};