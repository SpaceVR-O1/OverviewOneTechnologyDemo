#include <iostream>
#include <ctime>                 //System Time
#include "OperationalPlan.h"
#include "PowerConsumption.h"
#include "PowerGeneration.h"

using namespace std;

#define DEBUG 1
#define ON 1
#define OFF 0
#define POWER 0
#define ENERGY 1

#define NUMBER_OF_POWER_DRAW_SYSTEMS 6
#define NUMBER_OF_POWER_GENERATION_SYSTEMS 6
#define NUMBER_OF_POWER_STORAGE_SYSTEMS 4


//http://www1.cmc.edu/pages/faculty/alee/g++/g++.html
//Compiled using g++ -o OverviewOne OverviewOne.cpp PowerConsumption.cpp OperationalPlan.cpp -I.

int main(int argc, char *argv[])
{
  unsigned int x = 0xFFAA;
  unsigned int output = (x*2) & 0x000000FF;
  cout << "Ouput = " << output << endl;
    
  OperationalPlan AlwaysOn = OperationalPlan(true);
  
  OperationalPlan ThermalKnifePlan = OperationalPlan(false);
  ThermalKnifePlan.setPowerPlan(1, true);
  
  bool plan101010[6] = {true, false, true, false, true, false};
  bool plan010001[6] = {false, true, false, false, false, true};
  bool plan011001[6] = {false, true, true, false, false, true};
  OperationalPlan CommsPlan = OperationalPlan(plan101010);
  OperationalPlan CameraPlan = OperationalPlan(plan010001);
  OperationalPlan AttitudePlan = OperationalPlan(plan011001);
  
  double totalConsumption[2] = {0.0, 0.0};   //{Power (Wh), Energy (Ah)}
  
  PowerConsumption PowerConsumptionSystems[NUMBER_OF_POWER_DRAW_SYSTEMS] =
    {PowerConsumption("TK1", 0.960, 12.0, 100.0, &AlwaysOn), 
     PowerConsumption("See3CamCU130:0", 0.508, 5.0, 66.0, &CameraPlan),
     PowerConsumption("See3CamCU130:1", 0.508, 5.0, 66.0, &CameraPlan),
     PowerConsumption("SWIFT-XTS", 0.188, 7.4, 33.0, &CommsPlan),
     PowerConsumption("MAI-400", 1.690, 5.0, 100.0, &AttitudePlan),
     PowerConsumption("ThermalKnife", 2.00, 8.0, 100.0, &ThermalKnifePlan)};
     
  PowerGeneration PowerGenerationSystem[NUMBER_OF_POWER_GENERATION_SYSTEMS][2] = 
    { 
      {PowerGeneration("SP-L-S3U-0016-CS:0A", 443.0, 16.45), PowerGeneration("SP-L-S3U-0016-CS:0B", 443.0, 16.45)}, 
	  {PowerGeneration("SP-L-S3U-0016-CS:1A", 443.0, 16.45), PowerGeneration("SP-L-S3U-0016-CS:1B", 443.0, 16.45)},
	  {PowerGeneration("SP-L-S3U-0016-CS:2A", 443.0, 16.45), PowerGeneration("SP-L-S3U-0016-CS:2B", 443.0, 16.45)},
	  {PowerGeneration("SP-L-S3U-0016-CS:3A", 443.0, 16.45), PowerGeneration("SP-L-S3U-0016-CS:3B", 443.0, 16.45)},
	  {PowerGeneration("SP-L-S3U-0016-CS:4A", 443.0, 16.45), PowerGeneration("SP-L-S3U-0016-CS:4B", 443.0, 16.45)},
	  {PowerGeneration("SP-L-S1U-0002-CS:0A", 443.0, 16.45), PowerGeneration("SP-L-S1U-0002-CS:0B", 443.0, 16.45)}
	};
    
  for(int orbitNumber = 0; orbitNumber < FINAL_ORBIT; orbitNumber++){

    
    cout << "SUNSET of orbit #" << orbitNumber << endl;
    for(int i = 0; i < NUMBER_OF_POWER_DRAW_SYSTEMS; i++){
	  PowerConsumptionSystems[i].update(HALF_ORBIT, (orbitNumber*2));
	  totalConsumption[POWER] += PowerConsumptionSystems[i].getPowerConsumption();
	  totalConsumption[ENERGY]+= PowerConsumptionSystems[i].getEnergyConsumption(); 
	}
    cout << "Total power consumption is: " << totalConsumption[POWER] << " Wh" << endl;
	cout << "Total energy consumption is: " << totalConsumption[ENERGY] << " Ah" << endl;
	
    cout << "SUNRISE of orbit #" << (orbitNumber + 1) << endl;
    for(int i = 0; i < NUMBER_OF_POWER_DRAW_SYSTEMS; i++){
	  PowerConsumptionSystems[i].update(HALF_ORBIT, (orbitNumber*2+1)); 
	  totalConsumption[POWER] += PowerConsumptionSystems[i].getPowerConsumption();
	  totalConsumption[ENERGY]+= PowerConsumptionSystems[i].getEnergyConsumption();
	}
	cout << "Total power consumption is: " << totalConsumption[POWER] << " Wh" << endl;
	cout << "Total energy consumption is: " << totalConsumption[ENERGY] << " Ah" << endl;
  }//END OrbitNumber FOR LOOP    
    
  cout << endl << endl << endl;
  
  return 0;
}//END MAIN LOOP
