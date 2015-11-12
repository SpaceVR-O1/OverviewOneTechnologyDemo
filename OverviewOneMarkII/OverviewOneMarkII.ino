#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

/* This driver assumes you are using the Arduino Pro Mini 3.3V wired as follows:
 * https://upverter.com/SolX2010/882c528eb42e3c1a/Balloon-Flight-1/ 
 *  
 * This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
 * which provides a common 'type' for sensor data and some helper functions.
 *  
 * To use this driver you will also need to download the Adafruit_Sensor
 * library and include it in your libraries folder.
 * You should also assign a unique ID to this sensor for use with
 * the Adafruit Sensor API so that you can identify this particular
 * sensor in any data logs, etc.  To assign a unique ID, simply
 * provide an appropriate value in the constructor below (12345
 * is used by default in this example).
 * 
 * Connections
 * ===========
 * Connect SCL to analog 5
 * Connect SDA to analog 4
 * Connect VDD to 3.3V DC
 * Connect GROUND to common ground (GND)
 *  
 * History
 * =======
 * 2015/10/22 Initial code base
 */


/***************************************************************************
 * USEFUL CONSTANTS
 ***************************************************************************/
enum
{

  DEBUG = 1,                     //Change to 0 to turn OFF debug statments
  PRO_MINI = 2,
  NANO = 3,
  UNO = 4,
  NO_BUTTON = 0,
  SHORT_BUTTON_PRESS = 1,
  LONG_BUTTON_PRESS = 4,
  BUTTON_DEPRESS_MS_RESOLUTION = 10, //Software button debounce of 10 ms    
  HYSTERSIS_DELAY = 500

}; //END ENUM 


/***************************************************************************
 * HARDWARE PIN CONFIGURATION CONSTANTS
 * Arduino Pro Mini 5V/16 MHz ATmega328, Esplora 5V/16 MHz ATmega32u4, 
 * Nano 5V/16 MHz ATmega328, & Uno 5V/16 MHz ATmega16U2 pin configuration
 * @see https://www.arduino.cc/en/Main/Products
 ***************************************************************************/
enum
{
  // The Arduino Nano, Uno, and Pro Mini use the following:
  // Outputs: D2 and D9
  // Inputs: D5, D6, D7, D8,
  
  NANO_CUTDOWN_PIN = 2,
  NANO_BUTTON_1_PIN = 5,
  NANO_BUTTON_2_PIN = 6,
  NANO_BUTTON_3_PIN = 7,
  NANO_BUTTON_4_PIN = 8,
  NANO_HEATSINK_HEATER_PIN = 9,
  
  UNO_CUTDOWN_PIN = 2,
  UNO_BUTTON_1_PIN = 5,
  UNO_BUTTON_2_PIN = 6,
  UNO_BUTTON_3_PIN = 7,
  UNO_BUTTON_4_PIN = 8,
  UNO_HEATSINK_HEATER_PIN = 9,
  
  PRO_MINI_CUTDOWN_PIN = 2,          
  PRO_MINI_BUTTON_1_PIN = 5,        
  PRO_MINI_BUTTON_2_PIN = 6,        
  PRO_MINI_BUTTON_3_PIN = 7,        
  PRO_MINI_BUTTON_4_PIN = 8,        
  PRO_MINI_HEATSINK_HEATER_PIN = 9, 
  PRO_MINI_YELLOW_LED = 13,         //D13  

}; //END ENUM 


/***************************************************************************
 * AVERAGE CUTDOWN TIME CONSTANTS
 ***************************************************************************/
enum
{

  BALLOON_HY_1600_100_000_FEET = 5379,     // seconds
  BALLOON_HY_1600_75_000_FEET  = 4034,     // seconds
  BALLOON_HY_1600_X_50_000_FEET  = 2689,   // seconds
  SEA_LEVEL_PRESSURE = 1023,               // hPa 
  TARGET_MIN_TEMPERATURE =  2              // degrees C   

}; //END ENUM 


/***************************************************************************
 * GLOBAL VARIABLES
 ***************************************************************************/
int EEPROM_AddressPointer;         // Points to next memory location to write to

struct MyObject {                  // EEPROM.put() test structure 
  float field1;
  byte field2;
  char name[10];
};



/***************************************************************************
 * PUBLIC FUNCTIONS
 ***************************************************************************/

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

/**************************************************************************
 * Displays some basic information on this sensor from the unified
 * sensor API sensor_t type (see Adafruit_Sensor for more information)
 **************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); 
  Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); 
  Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); 
  Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); 
  Serial.print(sensor.max_value); 
  Serial.println(" hPa");
  Serial.print  ("Min Value:    "); 
  Serial.print(sensor.min_value);    //300 hPa = 9 km 0 C = not between 10 and 27 km
  Serial.println(" hPa");
  Serial.print  ("Resolution:   "); 
  Serial.print(sensor.resolution); 
  Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(5000);
}//END displaySensorDetails() FUNCTION


/*!
 * @brief Store temperature (degrees Celsius) and Pressure (hPa) into a 1 KB EEPROM.
 * 
 * @details Thhis functions can write upto 128 8-Byte pieces of data to the 1KB EEPROM every 
 * 100 seconds when using the Esplora, Nano, Pro Mini, and Uno Arduino products.
 * 
 * @see https://www.arduino.cc/en/Reference/EEPROMPut
 *
 * @param currentTemperature temperature reading from the BMP085 sensor in degreees C.
 * @param currentPressure pressure reading from the BMP085 sensor in hPa.
 * 
 * @return True if EEPROM has not overflowed - False otherwise
 */
bool LogData(float currentTemperature, float currentPressure)
{ 
  if (EEPROM_AddressPointer == EEPROM.length()) { // Memory pointer has overflowed
    if(DEBUG) Serial.println("ERROR! EEPROM full, please enter another quarter :)");
    float error = -1;
    EEPROM.put(0, error);         // Write -1 to EEMPROM address 0 to note overflow

    return false;
  }  

  // Each float variable is 4 bytes long
  EEPROM.put(EEPROM_AddressPointer, currentTemperature);
  EEPROM_AddressPointer += sizeof(float);  
  EEPROM.put(EEPROM_AddressPointer, currentPressure/100.00); //Convert Pa to hPa by dividing 
  EEPROM_AddressPointer += sizeof(float); 
  Serial.println("EEPROM data logging was successful.");
  return true;
  
}//END LogData() FUNCTION


/*!
 * @brief Deterime which thermal system (heating or cooling) to engage.
 * 
 * @details This functions controls a N-channel Power MOSFET driver circuit to supply
 * current to a resistive aluminium heating element, which when off acts like a heat sink.
 * 
 * @see https://upverter.com/SolX2010/882c528eb42e3c1a/Balloon-Flight-1/ 
 * 
 * @param currentTemperature temperature reading from the BMP085 sensor in degreees C.
 * 
 * @return NOTHING
 */
void adjustThermalControlSystem(int currentTemperature)
{
  if (DEBUG) Serial.println("THERMAL CONTROL SYSTEM ADJUSTING.");
  
  if(currentTemperature < TARGET_MIN_TEMPERATURE){
    if (DEBUG) Serial.println("THERMAL SYSTEM ON.");
    digitalWrite(PRO_MINI_HEATSINK_HEATER_PIN, HIGH);   // Turn ON MOSFET Q1 and heater
    delay(HYSTERSIS_DELAY);     //Hysteresis delay to stop thermal system from toggle high and low  
  }
  else{
    if (DEBUG) Serial.println("THERMAL SYSTEM OFF.");
    digitalWrite(PRO_MINI_HEATSINK_HEATER_PIN, LOW);   // Turn OFF MOSFET Q1 and allow heat sink to cool electronics
    delay(HYSTERSIS_DELAY);     //Hysteresis delay to stop thermal system from toggle high and low     
  }//END ELSEIDF
  
}//END adjustThermalControlSystem() FUNCTION


/*!
 * @brief Prints data in the 1KB EEPROM collected during flight.
 *
 * @details This functions reads and prints the 128 8-byte data pairs (temp & pressure)  
 * stored by the logData function.
 * 
 * @see https://www.arduino.cc/en/Tutorial/EEPROMGet
 * 
 * @param NONE
 *
 * @return NOTHING
 */
void EEPROMGET(){
  float f = 0.00f;   //Variable to store data read from EEPROM.
  int eeAddress = 0; //EEPROM address to start reading from
  
  for (int i = 0; i <= 127; i++){
    EEPROM.get(eeAddress, f);
    Serial.print("\nEEPROM ADDRESS #");
    Serial.println(i);
    Serial.print("EEPROM DATA = ");
    Serial.println(f, 3);    //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
    eeAddress += sizeof(float); //Move address to the next byte after float 'f'.
  }
}



/*!
 * @brief Example test code from Github for the BMP085 sensor.
 * 
 * @details This precision sensor from Bosch is the best low-cost sensing solution for measuring 
 * barometric pressure and temperature. Because pressure changes with altitude you can also use it 
 * as an altimeter!
 * 
 * @see https://github.com/adafruit/Adafruit_BMP085_Unified/blob/master/examples/sensorapi/sensorapi.pde
 * 
 * @param NONE
 *
 * @return NOTHING
 */
void unitTestBMP085(void)
{

  // Test Adafruit_BMP085_Unified Class
  Serial.println("Testing EAdafruit_BMP085_Unified Class.");

  /* Get a new sensor event */
  sensors_event_t event;
  bmp.getEvent(&event);

  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    Serial.print("Pressure:    ");
    Serial.print(event.pressure);
    Serial.println(" hPa");

    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */

    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    /* Then convert the atmospheric pressure, and SLP to altitude         *
     * Update this next line with the current SLP for better results      *
     * SLP = 1022 hPa at Denver International Airport                     */
    float seaLevelPressure = 1023.2; //SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print("Altitude:    "); 
    Serial.print(bmp.pressureToAltitude(seaLevelPressure, event.pressure)); 
    Serial.println(" m");
    Serial.println("");
  }
  else
  {
    Serial.println("Sensor error: DANGER, Will Robinson");
  }
  delay(1000);  

  // Test EEPROM.put() function
  Serial.print("Testing EEPROM.put() function");
  float f = 123.456f;  //Variable to store in EEPROM.
  int eeAddress = 0;   //Location we want the data to be put.

  //One simple call, with the address first and the object second.
  EEPROM.put(eeAddress, f);

  Serial.println("Written float data type!");

  /** Put is designed for use with custom structures also. **/

  //Data to store.
  MyObject customVar = {
    3.14f,
    65,
    "Working!"
  };

  eeAddress += sizeof(float); //Move address to the next byte after float 'f'.
  

  //EEPROM.put(eeAddress, customVar);
  Serial.print("Custom data type written! \nView the example sketch eeprom_get to see how you can retrieve the values! \n\n");
  
  /*
  //MAIN LOOP test data logging too many data points into EEPROM 150 > 127
  for (int i = 0; i < 150; i++){
    bmp.getTemperature(&currentTemperature);
    bmp.getPressure(&currentPressure);
    ok &= LogData(currentTemperature, currentPressure); 
    Serial.print("Loop #");
    Serial.println(i);
    delay(250);
  }//END FOR LOOP
  */ 
   EEPROMGET();
}//END unitTest() FUNCTION


/*!
 * @brief Configure the general purpose the input and output pins. 
 * 
 * @details This function configures the hardware depending on Arduino product used.
 * 
 * @see https://www.arduino.cc/en/Main/Products 
 *
 * @param arduinoBoardName - Development board selected for your project
 *
 * @return NOTHING
 */
void selectHardwareConfiguration(int arduinoBoardName)
{
  switch(arduinoBoardName){
    case PRO_MINI: //BUTTON_2_PIN:
      if (DEBUG) Serial.println("Arduino Pro Mini selected.");
      pinMode(PRO_MINI_CUTDOWN_PIN, OUTPUT);
      pinMode(PRO_MINI_BUTTON_1_PIN, INPUT_PULLUP);
      pinMode(PRO_MINI_BUTTON_2_PIN, INPUT_PULLUP);
      pinMode(PRO_MINI_BUTTON_3_PIN, INPUT_PULLUP);
      pinMode(PRO_MINI_BUTTON_4_PIN, INPUT_PULLUP);
      pinMode(PRO_MINI_YELLOW_LED, OUTPUT);
      pinMode(PRO_MINI_HEATSINK_HEATER_PIN, OUTPUT);
      break;
    case NANO: //BUTTON_3_PIN:
      if (DEBUG) Serial.println("Arduino Nano selected.");
      pinMode(NANO_CUTDOWN_PIN, OUTPUT);
      pinMode(NANO_BUTTON_1_PIN, INPUT_PULLUP);
      pinMode(NANO_BUTTON_2_PIN, INPUT_PULLUP);
      pinMode(NANO_BUTTON_3_PIN, INPUT_PULLUP);
      pinMode(NANO_BUTTON_4_PIN, INPUT_PULLUP);
      pinMode(NANO_HEATSINK_HEATER_PIN, OUTPUT);
      break;
    case UNO: //BUTTON_4_PIN:
      if (DEBUG) Serial.println("Arduino Uno selected.");
      pinMode(UNO_CUTDOWN_PIN, OUTPUT);
      pinMode(UNO_BUTTON_1_PIN, INPUT_PULLUP);
      pinMode(UNO_BUTTON_2_PIN, INPUT_PULLUP);
      pinMode(UNO_BUTTON_3_PIN, INPUT_PULLUP);
      pinMode(UNO_BUTTON_4_PIN, INPUT_PULLUP);
      pinMode(UNO_HEATSINK_HEATER_PIN, OUTPUT);
      break;
    default:
      Serial.println("ERROR! Invalid Arduino development board selected. Get a new job :)");
      break;
   }//END SWITCH  
   
}//END selectHardwareConfiguration() FUNCTION


/**************************************************************************
 * Arduino setup function (automatically called at startup)
 **************************************************************************/
void setup(void) 
{
  Serial.begin(9600);
  
  delay(100);
  while (!Serial);
  
  selectHardwareConfiguration(PRO_MINI); 
  
  EEPROM_AddressPointer = 0;  // Initialize global variable
  
  /* Initialise the sensor */
  if(!bmp.begin()){
    //* There was a problem detecting the BMP085 ... check your connections */
    if(DEBUG) Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  /* Display some basic information on this sensor */

  displaySensorDetails();
  
}//END setup() FUNCTION


/**************************************************************************
 * Arduino MAIN (automatically called after setup function)
 **************************************************************************/
void loop(void) 
{ 
  int i = 1;                             // While loop increment 
  bool nextButtonPressed = false;

  /* Get a new sensor event */
  sensors_event_t event;
  bmp.getEvent(&event);  
  float currentTemperature, currentPressure, currentAltitude;
  bool ok = true;                       // System status flag
  bool systemOn = true;
  
  
  if(DEBUG){
    Serial.println("Overview One BMP085 Sensor Connection Test STARTING. \n"); 
    unitTestBMP085();
    Serial.println("Overview One BMP085 Sensor Connection Test PASSED. \n\n"); 
    delay(1000);
  }

  
  //START THERMAL CONTROL SYSTEM AND LOGGING DATA EVERY 100 SECONDS
  if (DEBUG) Serial.println("START THERMAL CONTROL SYSTEM AND LOG DATA EVERY 100 SECONDS.");
  while(systemOn){ 
    // Loop every 100 seconds until button #1 is pressed again
    // ADD STUFF TO DO WHILE IN FLIGHT TO THIS WHILE LOOP
    bmp.getTemperature(&currentTemperature);
    bmp.getPressure(&currentPressure);     
    adjustThermalControlSystem(currentTemperature);
    ok &= LogData(currentTemperature, currentPressure);  
      
    // Pause 100 seconds for data logging, but update thermal control system every 1 second
    for (int sec = 0; sec < 100; sec++){
      delay(1000); 
      bmp.getTemperature(&currentTemperature);
      adjustThermalControlSystem(currentTemperature);
    }//END FOR LOOP
  }//END OUTER WHILE LOOP
     
}//END MAIN LOOP

