#include <Wire.h>
#include <Esplora.h>             //Include the Esplora library
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <StopWatch.h>

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
 * Connect GROUND to common ground
 *  
 * History
 * =======
 * 2015/10/22 Initial code base
 */


/***************************************************************************
 USEFUL CONSTANTS
 ***************************************************************************/
 enum
 {
   DEBUG = 1,                     //Change to 0 to turn OFF debug statments
   X_AXIS_INDEX = 0,
   Y_AXIS_INDEX = 1,
   Z_AXIS_INDEX = 2,  
   SHORT_BUTTON_PRESS = 1,
   LONG_BUTTON_PRESS = 2,
   BUTTON_DEPRESS_MS_RESOLUTION = 10 //Software button debounce of 10 ms                           
}; //END ENUM 


/***************************************************************************
 HARDWARE PIN CONFIGURATION CONSTANTS
 Arduino Pro Mini 3.3V / 16 MHz ATmega328 and Esplora pin configuration
 ***************************************************************************/
 enum
 {
   PRO_MINI_CUTDOWN_PIN = 2,         //D2 
   PRO_MINI_BUTTON_1_PIN = 5,        //D5
   PRO_MINI_BUTTON_2_PIN = 6,        //D6
   PRO_MINI_BUTTON_3_PIN = 7,        //D7
   PRO_MINI_BUTTON_4_PIN = 8,        //D8
   PRO_MINI_HEATSINK_HEATER_PIN = 9, //D9
   ESPLORA_CUTDOWN_PIN = 3,          //D3
   ESPLORA_HEATSINK_HEATER_PIN = 4   //D4       
  
}; //END ENUM 

/***************************************************************************
 AVERAGE CUTDOWN TIME CONSTANTS
 ***************************************************************************/
 enum
 {
   BALLOON_HY_1600_100_000_FEET = 5379,
   BALLOON_HY_1600_75_000_FEET  = 4034,  
   BALLOON_HY_1600_X_50_000_FEET  = 2689, 
   SEA_LEVEL_PRESSURE = 1022,               // hPa 
   TARGET_MIN_TEMPERATURE =  0              // degrees C                       
}; //END ENUM 

/***************************************************************************
 GLOBAL VARIABLES
 ***************************************************************************/
int EEPROM_AddressPointer;         // Points to next memory location to write to

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

/**************************************************************************
 Displays some basic information on this sensor from the unified
 sensor API sensor_t type (see Adafruit_Sensor for more information)
 **************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}//END displaySensorDetails() FUNCTION

StopWatch SW_CutDown(StopWatch::SECONDS);  // Initialize to full seconds
StopWatch SWarray[5];                      // Defaults to milliseconds

/*!
 * @brief START running timer.
 */
void startCutDownTimer(void)
{
  SW_CutDown.start();
  Esplora.writeRGB(0, 255, 0); //Turn RGB LED ON and make green
  Esplora.tone(50, 2000);  
}

/*!
 * @brief STOP timer until restartCutDownTimer() or startCutDownTimer() function is called.
 */
void stopCutDownTimer(void)
{
  SW_CutDown.stop();
  Esplora.writeRGB(0, 0, 0); //Turn RGB LED OFF  
}

/*!
 * @brief Reset timer to zero seconds.
 */
void restartCutDownTimer(void)
{
  SW_CutDown.reset();
  Esplora.writeRGB(0, 255, 0); //Turn RGB LED ON and make green 
}

/*!
 * @brief Get current length timer has run for in seconds
 * @return The current value (Overflows after 49.7 days)
 */
unsigned long getCutDownTimerValue(void)
{
  return SW_CutDown.elapsed();
}

/*!
 * @brief Get acceleration (in G's) of payload in the in X Y and Z direction in the form of array. 
 * 
 * @details The functions requests data from the YEI 3-Space IMU (Positive = up / Negative = down)
 * 
 * @return and array of 1. The acceleration along the Z -axis G's 0 to 3. 
 *                      2. The acceleration along the Y -axis
 *                      3. The acceleration along the X -axis
 */
float getAccelerometerData(void)
{
  float axisValues[3];
  
  axisValues[X_AXIS_INDEX] = Esplora.readAccelerometer(X_AXIS);
  axisValues[Y_AXIS_INDEX] = Esplora.readAccelerometer(Y_AXIS);
  axisValues[Z_AXIS_INDEX] = Esplora.readAccelerometer(Z_AXIS);
  
  return axisValues[3];
 
}

/*!
 * @brief Give command to cut down balloon 
 * 
 * @details This functions continues to try an cut down the balloon 
 * every 10 seconds until accelerometer states balloon is falling. 
 */
void cutDownBalloon(void)
{
  digitalWrite(ESPLORA_CUTDOWN_PIN, HIGH);   // Turn on MOSFET Q2
  
  bool isPayloadFalling = false;
  
  while(!isPayloadFalling){
    //CHECK ACCELEROMETER Z AXIS HERE???
    float data[3] ={0, 0, 0};
    data[3] = getAccelerometerData();
    if(data[Z_AXIS_INDEX] > 0){
      isPayloadFalling = false;
      delay(10000);
      digitalWrite(ESPLORA_CUTDOWN_PIN, HIGH);   // Turn on MOSFET Q1   
    }
    else{
     isPayloadFalling = true; 
    }
     
  }//END WHILE LOOP
}//END cutDownBalloon() FUNCTION

/*!
 * @brief Store temperture (degrees Celsius), Pressure (kPa), and altitude (m) in 1 KB EEPROM.
 * 
 * @details The functions writes upto 62 16-Byte pieces of data to the 1KB EEPROM every 100 seconds 
 * 
 * @return Nothing
 */
bool LogData(float currentTemperature, float currentPressure, float currentAltitude)
{ 
  if (EEPROM_AddressPointer == EEPROM.length()) { // Memory pointer has overflowed
    if(DEBUG) Serial.println("ERROR! EEPROM full, please enter another quater.");
    EEPROM.write(0, -1);         // Write -1 to EEMPROM address 0 to note overflow
    return false;
  }  

  // Details on EEPROM.put() funtion https://www.arduino.cc/en/Reference/EEPROMPut
  EEPROM.put(EEPROM_AddressPointer, currentTemperature);
  EEPROM_AddressPointer++;
  EEPROM.put(EEPROM_AddressPointer, currentPressure);
  EEPROM_AddressPointer++;
  EEPROM.put(EEPROM_AddressPointer, currentAltitude);
  EEPROM_AddressPointer++;

  Esplora.writeRGB(0, 0, 255); //Turn RGB LED ON and make blue
  return true;
}


/*!
 * @brief Deterime which thermal system to turn and engage.
 * 
 * @param Input temperature reading from sensor.
 * 
 * @return Nothing
 */
void adjustThermalControlSystem(int currentTemperature)
{
  if(currentTemperature < TARGET_MIN_TEMPERATURE){
    digitalWrite(ESPLORA_HEATSINK_HEATER_PIN, HIGH);   // Turn ON MOSFET Q1 and heater  
  }
  else{
    digitalWrite(ESPLORA_HEATSINK_HEATER_PIN, LOW);   // Turn OFF MOSFET Q1 and allow heat sink to cool electronics    
  }
}

/*!
 * @brief Turn off the thermal control system
 */
void stopThermalControlSystem()
{
  digitalWrite(ESPLORA_HEATSINK_HEATER_PIN, LOW);   // Turn OFF MOSFET Q1 and allow heat sink to cool electronics
}

/*!
 * @brief Caputure button pushes on an external hardware button.
 *
 * @details This functions caputures when Normaly Open (NO) button is 
 * depressed. It assumes there is NO hardware debouncing circuitry. 
 * 
 * @param pin - Microcontroller pin button is connected to (pulled high).
 * 
 * @returns 2 for long button hold, 1 for short buton hold, and loops for no button press
 */  
int getProMiniButtonState(int pin)
{
  bool buttonPressCaptured = false;
  unsigned long ButtonDepressTimeMS = 0;
       
  while(!buttonPressCaptured){   
    if(digitalRead(pin) == HIGH){             // Is button pressed? 
      SWarray[0].start();                     // Start button debounce and push length timer
     
      while(digitalRead(pin) == HIGH){        // Is button still pressed?
        delay(BUTTON_DEPRESS_MS_RESOLUTION);  // Software button debounce ~10 ms
        ButtonDepressTimeMS = SWarray[0].elapsed();  // Timer to deterime short vs long button press
      }//END INNER WHILE LOOP     
       
      if(ButtonDepressTimeMS > 2000){         
        buttonPressCaptured = true;          // Long button press
        return LONG_BUTTON_PRESS;   
      } 
      else if(BUTTON_DEPRESS_MS_RESOLUTION < ButtonDepressTimeMS && ButtonDepressTimeMS <= 2000 ){
        buttonPressCaptured = true;         // Short button press
        return SHORT_BUTTON_PRESS;   
      }
      else{
        buttonPressCaptured = false;
        delay(2000);
        Serial.println("ERROR! Invalid button press length.");
        SWarray[0].reset();                // Reset timer to determine button press type again.
      }//END INNER IF       
     
    }//END OUTER IF

    switch(pin){
      case SWITCH_UP: //BUTTON_1_PIN:
        Serial.println("Please push button 1 to begin cut down timer.");
        break;
      case SWITCH_RIGHT: //BUTTON_2_PIN:
        Serial.println("Please push button 2 to begin data logging.");
        break;
      case SWITCH_DOWN: //BUTTON_3_PIN:
        Serial.println("Please push button 3 to start thermal control system.");
        break;
      case SWITCH_LEFT: //BUTTON_4_PIN:
        Serial.println("Please push button 4 to stop thermal control system.");
        break;
      default:
        Serial.println("ERROR! Invalid hardware pin passed to getButtonState() function.");
      break;
    }// END SWITCH 
    delay(1000);
  }//END OUTER WHILE LOOP  
 
}//END getButtonState() FUNCTION



struct MyObject {
  float field1;
  byte field2;
  char name[10];
};

/*!
 * @brief Example test code from Github
 * @see https://github.com/adafruit/Adafruit_BMP085_Unified/blob/master/examples/sensorapi/sensorapi.pde
 */
void unitTest(void)
{
  
  // Test Adafruit_BMP085_Unified Class
  Serial.print("Testing EAdafruit_BMP085_Unified Class.");
  
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
    float seaLevelPressure = 1022; // = SENSORS_PRESSURE_SEALEVELHPA;
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

  EEPROM.put(eeAddress, customVar);
  Serial.print("Written custom data type! \n\nView the example sketch eeprom_get to see how you can retrieve the values!");
  
}//END unitTest() FUNCTION


/**************************************************************************
 Arduino setup function (automatically called at startup)
 **************************************************************************/
void setup(void) 
{
   // Set digital pins as outputs.
   pinMode(ESPLORA_CUTDOWN_PIN, OUTPUT);  
   pinMode(ESPLORA_HEATSINK_HEATER_PIN, OUTPUT);

   // Set the Pro Mini digital pins as inputs.
   //pinMode(BUTTON_1_PIN, INPUT);
   //pinMode(BUTTON_2_PIN, INPUT);
   //pinMode(BUTTON_3_PIN, INPUT);
   //pinMode(BUTTON_4_PIN, INPUT);
  
  Serial.begin(9600);
  if(DEBUG) Serial.println("Overview One BMP085 Sensor Connection Test"); 
  if(DEBUG) Serial.println("");

  // Initialize global variables 
  EEPROM_AddressPointer = 0;

  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    if(DEBUG) Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
}//END setup() FUNCTION


void loop(void) 
{ 
  /* Get a new sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);  
  float currentTemperature, currentPressure, currentAltitude;
  bool ok = false;                       // System status flag
  
  if(DEBUG){
    unitTest();
    Serial.println("Overview One BMP085 Sensor Connection Test PASSED"); 
    delay(1000);
  }

  Serial.println("Push Button 1 to begin timer.");
  if(!Esplora.readButton(SWITCH_1)){
  //if(getProMiniButtonState(BUTTON_1_PIN) == SHORT_BUTTON_PRESS){ //Loops until button 1 is pressed
    startCutDownTimer();   
  }

  Serial.println("Push Button 2 to begin data logging.");
  if(!Esplora.readButton(SWITCH_2)){
  //if(getProMiniButtonState(BUTTON_2_PIN) == SHORT_BUTTON_PRESS){ //Loops until button 2 is pressed
    bmp.getTemperature(&currentTemperature);
    bmp.getPressure(&currentPressure);
    currentAltitude = bmp.pressureToAltitude(SEA_LEVEL_PRESSURE, event.pressure);
    ok &= LogData(currentTemperature, currentPressure, currentAltitude);  
  }

  Serial.println("Push button 3 to start thermal control system.");
  if(!Esplora.readButton(SWITCH_3)){
  //if(getProMiniButtonState(BUTTON_3_PIN) == LONG_BUTTON_PRESS){ //Loops until button 3 is pressed
    bmp.getTemperature(&currentTemperature);
    adjustThermalControlSystem(currentTemperature);   
  }

  while(getCutDownTimerValue() < BALLOON_HY_1600_100_000_FEET){
    // Loop every 100 secondsuntil timer reaches set cut down time 
    // Do OTHER stuff here while in flight
    bmp.getTemperature(&currentTemperature);
    bmp.getPressure(&currentPressure);
    currentAltitude = bmp.pressureToAltitude(SEA_LEVEL_PRESSURE, event.pressure);
    ok &= LogData(currentTemperature, currentPressure, currentAltitude);  
    delay(100000);
  }

  cutDownBalloon(); 

  Serial.println("Push button 4 to stop thermal control system.");
  if(!Esplora.readButton(SWITCH_4)){
  //if(getProMiniButtonState(BUTTON_4_PIN) == SHORT_BUTTON_PRESS){ //Loops until button 4 is pressed
    stopThermalControlSystem();  
  }

  //Status LED at end of flight Red = BAD / Green = GOOD
  if(ok){
    Esplora.writeRGB(0, 255, 0); //Turn RGB LED ON and make green   
  }
  else{
    Esplora.writeRGB(255, 0, 0); //Turn RGB LED ON and make red   
  }
    
}//END MAIN LOOP
