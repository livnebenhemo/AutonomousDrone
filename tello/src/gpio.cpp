#include "include/gpio.h"

//#define button_pin (32)
//#define sense_pin (40)
//#define led_pin (22)

int main()
{
}

void gpio::setSensePin(const int &toSet) {
    sense_pin = toSet;
}

void gpio::setLedPin(const int &toSet) {
    led_pin = toSet;
}

void gpio::setButtonPin(const int &toSet) {
    button_pin = toSet;
}

const int &gpio::getButtonPin() {
    return button_pin;
}

const int &gpio::getSensePin() {
    return sense_pin;
}

const int &gpio::getLedPin() {
    return led_pin;
}

void gpio::toCharge() {
    this->SetUps();

    pinMode(button_pin, OUTPUT);
    digitalWrite(button_pin, HIGH);
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, HIGH);
    pinMode(sense_pin, INPUT);

    int counter = 0;
    int counter_neg = 0;
    int charge_time = 2400;
    bool flag = true;

    while (flag) {
        try {
            std::cout << counter << std::endl;
            if (digitalRead(sense_pin)) {
                sleep(0.01);
                counter++;
            } else {
                counter_neg++;
                if (counter_neg == 100) {
                    counter = 0;
                    counter_neg = 0;
                }
            }
            if (counter == 100) {
                digitalWrite(button_pin, LOW);
                digitalWrite(led_pin, LOW);
                std::cout << "Turned Off" << std::endl;
                sleep(10);
                digitalWrite(button_pin, HIGH);
                std::cout << "Button is released" << std::endl;
                sleep(20 + charge_time);
                digitalWrite(button_pin, LOW);
                std::cout << "Turned on" << std::endl;
                sleep(3);
                digitalWrite(button_pin, HIGH);
                digitalWrite(led_pin, HIGH);
                std::cout << "Button is released" << std::endl;
                sleep(5);
            }
        }
        catch (std::exception const) {
            std::cout << " Stop " << std::endl;
            break;
        }
        cleanUpForCharge();
        return;
    }
}

void gpio::cleanUpForCharge() {
    pinMode(button_pin,INPUT);
    pinMode(led_pin,INPUT);
    pinMode(sense_pin,INPUT);
}

void gpio::SetUps() {
    wiringPiSetup();            //wiring set up
    wiringPiSetupGpio();        //board set up
}
