#ifndef POWER_CONSUMPTION_H
#define POWER_CONSUMPTION_H

#include <string>
#include "OperationalPlan.h"

#define DEBUG 1

class PowerConsumption
{ 
  
  public: 
    PowerConsumption();
    ~PowerConsumption();
    PowerConsumption(const char *, double, double);
    PowerConsumption(const char *, double, double, double, OperationalPlan *);
    
    void setCurrentDraw(double);
    double getCurrentDraw();
    
    void printPartNumber();
    
    bool isOn();
    void setPowerState(bool);
    
    double getPowerConsumption();
    double getEnergyConsumption();
    
    void update(int, int);

  private:
    bool powerState;           //ON = true or OFF = false
    string partNumber;         //Manufacture Part Number [P/N]
    double currentDraw;        //Units [Amps = A]
    double voltageRating;      //Units [Volts = V]
    double powerRating;        //Units [Watts = W]
    double dutyCycle;          //Units [% per 90 minute orbit]
    double energyConsumed;     //Units [AmpHours = Ah]
    double powerConsumed;      //Units [WattHours = Wh] 
    OperationalPlan opPlan;    //Power operation plan per half orbit
};

#endif
