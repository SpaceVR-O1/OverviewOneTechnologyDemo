#include <iostream>
#include "PowerConsumption.h"
#include "OperationalPlan.h"

using namespace std;

/** Default Constructor. 
 */ 
PowerConsumption::PowerConsumption(){
  cout << "I'm sorry Dave, I'm afraid I can't do that. Invalid PowerConsumption created!" << endl;
}

/** Default Destructor with part number label
 */ 
PowerConsumption::~PowerConsumption(){
  cout << "PowerConsumption " << partNumber << " object was deleted." << endl;
}

/** The prefered Constructor that sets all member variables 
 * @param name
 * @param current The current draw in units of Amps
 * @param voltage
 * @param onPercentage The power duty cycle 
 * @param plan Half orbit operation plan for 
 */ 
PowerConsumption::PowerConsumption(const char * name, double current, double voltage, double onPercentage, OperationalPlan * plan){
  powerState = true;
  partNumber = name;
  currentDraw = current;
  voltageRating = voltage;
  powerRating = current * voltage;
  dutyCycle = onPercentage;
  energyConsumed = 0.0;
  powerConsumed =  0.0;
  opPlan = *plan;
}

/**Set new current draw of a Power Consumption object 
 * @param newCurrentDraw The new current draw in Amps
 * @retun NOTHING
 * @author Blaze Sanders 
 */
void PowerConsumption::setCurrentDraw(double newCurrentDraw){
  currentDraw = newCurrentDraw;
}

/**Get current current draw of a Power Consumption object 
 * @retun Current draw in Amps
 * @author Blaze Sanders 
 */
double PowerConsumption::getCurrentDraw(){
  return currentDraw;
}

/**Turn a Power Consumption object ON or OFF
 * @param newState The new power state of a object
 * @retun NOTHING
 * @author Blaze Sanders 
 */
void PowerConsumption::setPowerState(bool newState){
  if(newState == true){
    powerState = true;
  }
  else{
    powerState = false;
  }	
}

/**Print object part number
 * @retun cout print of object part number
 * @author Blaze Sanders 
 */
void PowerConsumption::printPartNumber(){
  cout << partNumber << " ";	
}

/**Determines the power state of a Power Consumption object 
 * @retun True if object is turned ON, False otherwise
 * @author Blaze Sanders 
 */
bool PowerConsumption::isOn(){
  return powerState;	
}

/**Get the lifetime power consumption of an object
 * @retun Total power consumed in units of WattHours
 * @author Blaze Sanders 
 */
double PowerConsumption::getPowerConsumption(){
  return powerConsumed;
}

/**Get the lifetime energy consumption of an object
 * @retun Total power consumed in units of AmpHours
 * @author Blaze Sanders 
 */
double PowerConsumption::getEnergyConsumption(){
  return energyConsumed;
}

/**Update the lifetime power and energy consumption of an object 
 * @param stepSize The intergration step size for calculation in units of minutes
 * @param dutyCycle The on percent out of 100% per 90 min orbit
 * @param halfOrbitNumber Number of half orbits that have passed (Three half orbits = 1.5 orbits)
 * @retun NOTHING
 * @author Blaze Sanders 
 */
void PowerConsumption::update(int stepSize, int halfOrbitNumber){
  
  if(opPlan.getPowerPlan(halfOrbitNumber)){ 
    if(DEBUG) cout << partNumber << " was on during half orbit number: " << halfOrbitNumber << endl;
    switch (stepSize){
      case 45:
        energyConsumed += (currentDraw * 0.75);
        powerConsumed   += (powerRating * 0.75);
        break;  
      case 90:
       energyConsumed += (currentDraw * 1.5);
       powerConsumed   += (powerRating * 1.5);
       break;
      default: 
        cout << "Invalid step size. Please use 45 or 90 mintues." << endl; 
    }//END SWITCH STATEMENT 
    
    energyConsumed *= (dutyCycle/100);
    powerConsumed *= (dutyCycle/100);
  }//END isON() IF STATMENT 
}

