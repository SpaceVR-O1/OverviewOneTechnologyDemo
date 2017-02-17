#include <iostream>
#include "OperationalPlan.h"

using namespace std;

/** Default Constructor which sets powerPlan array to all TRUE / ON
 */
OperationalPlan::OperationalPlan(){
  OperationalPlan(true);
}

OperationalPlan::OperationalPlan(bool powerState){
  for(int i = 0; i < FINAL_ORBIT*2; i++){
    powerPlan[i] = powerState;  
  }//END FOR LOOP
}


/** Default Destructor with part number label
 */
OperationalPlan::~OperationalPlan(){
  cout << "OperationalPlan object was deleted." << endl;
}

/** Constructor to fill the powerPlan array with repeating input parameter pattern 
 * @param pattern The operation plan to repeat until final orbit
 */
OperationalPlan::OperationalPlan(bool pattern[]){
  int arrayLength = sizeof(pattern) / sizeof(*pattern);
  for(int i = 0; i < FINAL_ORBIT*2;  i += arrayLength){
	if( (i + arrayLength) <= FINAL_ORBIT*2 ){
      for(int j = 0; j < arrayLength; j++){ 
        powerPlan[i+j] = pattern[j]; 
	  }//END FOR LOOP
    }//END IF
    else{
	  for(int k = 0; k < (FINAL_ORBIT*2) - i ; k++){ 
        powerPlan[i+k] = pattern[k]; 
	  }//END FOR LOOP	
	}//ENDELSE 
  }//END OUTER FOR LOOP
}

void OperationalPlan::setPowerPlan(int halfOrbitNumber, bool powerState){
  powerPlan[halfOrbitNumber] = powerState;	
}

bool OperationalPlan::getPowerPlan(int halfOrbitNumber){
  return powerPlan[halfOrbitNumber];
}

