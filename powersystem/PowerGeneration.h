#ifndef POWER_GENERATION_H
#define POWER_GENERATION_H

#include <string>

#define DEBUG 1

class PowerGeneration
{ 
  
  public: 
    PowerGeneration();
    ~PowerGeneration();
    PowerGeneration(const char *, double, double);
    
    //~ double setCurrentOutput();
    //~ double getCurrentOutput();
    
    //~ double setVoltageOutput();
    //~ double getVoltageOutput();
    
    //~ void update(int, int);

  private:
    string partNumber;         //Manufacture Part Number [P/N]
    double currentOutput;      //Units [Amps = A]
    double voltageOuput;       //Units [Volts = V]
    double powerOutput;        //Units [Watts = W]
};

#endif
