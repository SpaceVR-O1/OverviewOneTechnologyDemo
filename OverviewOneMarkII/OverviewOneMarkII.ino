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
  ESPLORA = 1,
  PRO_MINI = 2,
  NANO = 3,
  UNO = 4,
  X_AXIS_INDEX = 0,
  Y_AXIS_INDEX = 1,
  Z_AXIS_INDEX = 2,  
  SHORT_BUTTON_PRESS = 1,
  LONG_BUTTON_PRESS = 2,
  BUTTON_DEPRESS_MS_RESOLUTION = 10 //Software button debounce of 10 ms    

}; //END ENUM 


/***************************************************************************
 * HARDWARE PIN CONFIGURATION CONSTANTS
 * Arduino Pro Mini 3.3V / 16 MHz ATmega328 and Esplora pin configuration
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
  
  PRO_MINI_CUTDOWN_PIN = 2,         //D2 
  PRO_MINI_BUTTON_1_PIN = 5,        //D5
  PRO_MINI_BUTTON_2_PIN = 6,        //D6
  PRO_MINI_BUTTON_3_PIN = 7,        //D7
  PRO_MINI_BUTTON_4_PIN = 8,        //D8
  PRO_MINI_HEATSINK_HEATER_PIN = 9, //D9


  // The Arduino Esplora uses the following:
  // Outputs: D3 and D4
  // Inputs: Built in buttons 
  //         > Esplora.readButton(SWITCH_1)
  //         > Esplora.readButton(SWITCH_2)
  //         > Esplora.readButton(SWITCH_3)
  //         > Esplora.readButton(SWITCH_4)
  
  ESPLORA_CUTDOWN_PIN = 3,          //D3
  ESPLORA_HEATSINK_HEATER_PIN = 4   //D4       

}; //END ENUM 


/***************************************************************************
 * AVERAGE CUTDOWN TIME CONSTANTS
 ***************************************************************************/
enum
{

  BALLOON_HY_1600_100_000_FEET = 5379,     // seconds
  BALLOON_HY_1600_75_000_FEET  = 4034,     // seconds
  BALLOON_HY_1600_X_50_000_FEET  = 2689,   // seconds
  SEA_LEVEL_PRESSURE = 1022,               // hPa 
  TARGET_MIN_TEMPERATURE =  0              // degrees C   

}; //END ENUM 


/***************************************************************************
 * GLOBAL VARIABLES
 ***************************************************************************/
int EEPROM_AddressPointer;         // Points to next memory location to write to

struct MyObject {
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
  Serial.print(sensor.min_value); 
  Serial.println(" hPa");
  Serial.print  ("Resolution:   "); 
  Serial.print(sensor.resolution); 
  Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(5000);
}//END displaySensorDetails() FUNCTION



StopWatch SW_CutDown(StopWatch::SECONDS);  // Initialize to full seconds
StopWatch SWarray[5];                      // Defaults to milliseconds

/*!
 * @brief START running timer.
 */
void startCutDownTimer(void)
{
  if(DEBUG) Serial.println("Timer has started.");
  SW_CutDown.start();
  
}//END startCutDownTimer() FUNCTION

/*!
 * @brief STOP timer until restartCutDownTimer() or startCutDownTimer() function is called.
 *
 * @return NOTHING
 */
void stopCutDownTimer(void)
{
  if(DEBUG) Serial.println("Timer has stopped.");
  SW_CutDown.stop();
  
}//END stopCutDownTimer() FUNCTION

/*!
 * @brief Reset timer to zero seconds and .
 *
 * @return NOTHING
 */
void restartCutDownTimer(void)
{
  if(DEBUG) Serial.println("Timer has being reset to zero and is still running.");
  SW_CutDown.reset();
  
}//END restartCutDownTimer() FUNCTION

/*!
 * @brief Get current length timer has run for in seconds
 *
 * @return The current cutdown timer value (Overflows after 49.7 days)
 */
unsigned long getCutDownTimerValue(void)
{
  if(DEBUG){
    Serial.print("Current elapsed time is: ");
    Serial.println(SW_CutDown.elapsed());
  }//END IF
  
  return SW_CutDown.elapsed();
  
}//END getCutDownTimerValue() FUNCTION

/*!
 * @brief Get acceleration (in G's) of payload in the in X Y and Z direction in the form of array. 
 * 
 * @details The functions requests data from the Esplora Acceleromete or YEI 3-Space IMU (Positive = up / Negative = down)
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

}//END getAccelerometerData() FUNCTION

/*!
 * @brief Give command to cut down balloon 
 * 
 * @details This functions continues to try an cut down the balloon 
 * every 10 seconds until accelerometer states balloon is falling.
 *
 * @return NOTHING 
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
    }//END ELSEIF

  }//END WHILE LOOP
  
}//END cutDownBalloon() FUNCTION

/*!
 * @brief Store temperture (degrees Celsius), Pressure (kPa), and altitude (m) in 1 KB EEPROM.
 * 
 * @details The functions can write upto 62 16-Byte pieces of data to the 1KB EEPROM every 100 seconds 
 * 
 * @return True if EEPROM has not overflowed - False otherwise
 */
bool LogData(float currentTemperature, float currentPressure, float currentAltitude)
{ 
  if (EEPROM_AddressPointer == EEPROM.length()) { // Memory pointer has overflowed
    if(DEBUG) Serial.println("ERROR! EEPROM full, please enter another quarter :)");
    EEPROM.write(0, -1);         // Write -1 to EEMPROM address 0 to note overflow

    return false;
  }  

  // Details on EEPROM.put() funtion https://www.arduino.cc/en/Reference/
  // Each float variable is 4 bytes long
  EEPROM.put(EEPROM_AddressPointer, currentTemperature);
  EEPROM_AddressPointer = EEPROM_AddressPointer + 4;   
  EEPROM.put(EEPROM_AddressPointer, currentPressure);
  EEPROM_AddressPointer = EEPROM_AddressPointer + 4;  
  EEPROM.put(EEPROM_AddressPointer, currentAltitude);
  EEPROM_AddressPointer = EEPROM_AddressPointer + 4;  

  return true;
  
}//END LogData() FUNCTION


/*!
 * @brief Deterime which thermal system to turn and engage.
 * 
 * @param Input temperature reading from sensor.
 * 
 * @return NOTHING
 */
void adjustThermalControlSystem(int currentTemperature)
{
  if(currentTemperature < TARGET_MIN_TEMPERATURE){
    digitalWrite(ESPLORA_HEATSINK_HEATER_PIN, HIGH);   // Turn ON MOSFET Q1 and heater  
  }
  else{
    digitalWrite(ESPLORA_HEATSINK_HEATER_PIN, LOW);   // Turn OFF MOSFET Q1 and allow heat sink to cool electronics    
  }//END ELSEIDF
  
}//END adjustThermalControlSystem() FUNCTION


/*!
 * @brief Turn off the thermal control system
 * 
 * @return NOTHING
 */
void stopThermalControlSystem()
{
  digitalWrite(ESPLORA_HEATSINK_HEATER_PIN, LOW);   // Turn OFF MOSFET Q1 and allow heat sink to cool electronics
  
}//END stopThermalControlSystem() FUNCTION


/*!
 * @brief Caputure button pushes on an external hardware button.
 *
 * @details This functions caputures when Normaly Open (NO) button is 
 * depressed. It assumes there is NO hardware debouncing circuitry. 
 * 
 * @param pin - Microcontroller pin button is connected to and which is pulled high.
 * @see https://upverter.com/SolX2010/882c528eb42e3c1a/Balloon-Flight-1/ 
 * 
 * @return 2 for long button hold, 1 for short buton hold, and loops when no button is pressed
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
      }//END INNER ELSEIF       

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
  Serial.print("Written +custom data type! \n\nView the example sketch eeprom_get to see how you can retrieve the values!");

}//END unitTest() FUNCTION



void waitForNextEsploraButton(int lastButtonPressed)
{
  bool nextButtonPressed = false;
  
  while(!nextButtonPressed){
    switch(lastButtonPressed){
      case SWITCH_1: //BUTTON_1_PIN:
        if (DEBUG) Serial.println("Button 1 was pressed last. Please press button 2 next.");
        Esplora.writeRGB(0, 255, 0); 
          if(!Esplora.readButton(SWITCH_2) || (!Esplora.readButton(SWITCH_3))||(!Esplora.readButton(SWITCH_4))){
            nextButtonPressed = true;
        }//END IF
        break;
      case SWITCH_2: //BUTTON_2_PIN:
        if (DEBUG) Serial.println("Button 2 was pressed last. Please press button 3 next.");
        Esplora.writeRGB(0, 0, 255); 
          if(!Esplora.readButton(SWITCH_1) || (!Esplora.readButton(SWITCH_3))||(!Esplora.readButton(SWITCH_4))){
            nextButtonPressed = true;
        }//END IF
        break;
      case SWITCH_3: //BUTTON_3_PIN:
        if (DEBUG) Serial.println("Button 3 was pressed last. Please press button 4 next.");
        Esplora.writeRGB(255, 0, 0); 
          if(!Esplora.readButton(SWITCH_1) || (!Esplora.readButton(SWITCH_2))||(!Esplora.readButton(SWITCH_4))){
            nextButtonPressed = true;
        }//END IF
        break;
      case SWITCH_4: //BUTTON_4_PIN:
        if (DEBUG) Serial.println("Button 4 was pressed last. Have a nice day.");
        Esplora.writeRGB(255, 0, 0); 
          if(!Esplora.readButton(SWITCH_1) || (!Esplora.readButton(SWITCH_2))||(!Esplora.readButton(SWITCH_3))){
            nextButtonPressed = true;
        }//END IF
      default:
        Serial.println("ERROR! You have too many buttons on your Esplora Dev Kit :)");
        break;
   }//END SWITCH

   delay(500); // Slow down print statements
   
 }//END WHILE LOOP
        
}//END waitForNextEsploraButton() functions

/*!
 * @brief Configures the input and output pins depending on hardware used.
 * @see https://www.arduino.cc/en/Main/Products 
 *
 * @param arduinoBoardName - Development board select for your project
 *
 $ @return NOTHING
 */
void selectHardwareConfiguration(int arduinoBoardName)
{
  switch(arduinoBoardName){
    case ESPLORA: //BUTTON_1_PIN:
      if (DEBUG) Serial.println("Arduino Esplora selected.");
      pinMode(ESPLORA_CUTDOWN_PIN, OUTPUT);  
      pinMode(ESPLORA_HEATSINK_HEATER_PIN, OUTPUT);
      break;
    case PRO_MINI: //BUTTON_2_PIN:
      if (DEBUG) Serial.println("Arduino Pro Mini selected.");
      pinMode(PRO_MINI_BUTTON_1_PIN, INPUT);
      pinMode(PRO_MINI_BUTTON_2_PIN, INPUT);
      pinMode(PRO_MINI_BUTTON_3_PIN, INPUT);
      pinMode(PRO_MINI_BUTTON_4_PIN, INPUT);
      break;
    case NANO: //BUTTON_3_PIN:
      if (DEBUG) Serial.println("Arduino Nano selected.");
      //TO-DO
      break;
    case UNO: //BUTTON_4_PIN:
      if (DEBUG) Serial.println("Arduino Uno selected.");
      //TO-DO
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
  
  selectHardwareConfiguration(ESPLORA); //selectHardwareConfiguration(PRO_MINI);  //selectHardwareConfiguration(NANO);
  
  EEPROM_AddressPointer = 0;  // Initialize global variable
  
  displaySensorDetails();

  //
  /* Initialise the sensor */
  //  if(!bmp.begin())
  //  {
  //    /* There was a problem detecting the BMP085 ... check your connections */
  //    if(DEBUG) Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
  //    while(1);
  //  }
  //  
  /* Display some basic information on this sensor */

  
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
  //bmp.getEvent(&event);  
  float currentTemperature, currentPressure, currentAltitude;
  bool ok = false;                       // System status flag

  if(DEBUG){
    Serial.println("Overview One BMP085 Sensor Connection Test STARTING."); 
    //unitTest();
    Serial.println("Overview One BMP085 Sensor Connection Test PASSED."); 
    delay(1000);
  }

  Serial.println("Push Button 1 to start cutdown timer.");
  Serial.println("Push Button 2 to start data logging.");
  Serial.println("Push button 3 to start thermal control system.");
  Serial.println("Push button 4 to stop thermal control system and data logging.");

  //START CUT DOWN TIMER
  if(!Esplora.readButton(SWITCH_1)){ //if(getProMiniButtonState(BUTTON_1_PIN) == SHORT_BUTTON_PRESS) //Loops until button 1 is pressed
    startCutDownTimer();   
    Esplora.tone(50, 2000);          // Two second 50 Hz tone
  }//END IF
  waitForNextEsploraButton(SWITCH_1);

  //START DATA LOGGING
  if(!Esplora.readButton(SWITCH_2)){ //if(getProMiniButtonState(BUTTON_3_PIN) == LONG_BUTTON_PRESS){ //Loops until button 3 is pressed
    bmp.getTemperature(&currentTemperature);
    bmp.getPressure(&currentPressure);
    currentAltitude = bmp.pressureToAltitude(SEA_LEVEL_PRESSURE, event.pressure);
    ok &= LogData(currentTemperature, currentPressure, currentAltitude);
  }//END IF
  waitForNextEsploraButton(SWITCH_2); 
  
  //START THERMAL CONTROL SYSTEM AND CONTINUE LOGGING DATA EVERY 100 SECONDS
  if(!Esplora.readButton(SWITCH_3)){ //if(getProMiniButtonState(BUTTON_2_PIN) == SHORT_BUTTON_PRESS){ //Loops until button 2 is pressed

    while(getCutDownTimerValue() < BALLOON_HY_1600_100_000_FEET){
      // Loop every 100 seconds until timer reaches set cut down time 
      // ADD STUFF TO DO WHILE IN FLIGHT TO THIS WHILE LOOP
      
      bmp.getTemperature(&currentTemperature);
      bmp.getPressure(&currentPressure);
      currentAltitude = bmp.pressureToAltitude(SEA_LEVEL_PRESSURE, event.pressure);
      
      adjustThermalControlSystem(currentTemperature);
      ok &= LogData(currentTemperature, currentPressure, currentAltitude);  
    
      // Pause 100 seconds for data logging, but update thermal control system every 1 second
      for (int sec = 0; sec < 100; sec++){
        delay(1000); 
        bmp.getTemperature(&currentTemperature);
        adjustThermalControlSystem(currentTemperature);
      }//END FOR LOOP

    }//END WHILE LOOP
    
  }//END IF
  waitForNextEsploraButton(SWITCH_3);
  
  if(!Esplora.readButton(SWITCH_4)){
    //if(getProMiniButtonState(BUTTON_4_PIN) == SHORT_BUTTON_PRESS){ //Loops until button 4 is pressed
    stopThermalControlSystem();  
  }
  
  //Status LED at end of flight 
  if(ok){
    if (DEBUG) Serial.println("GOOD TO GO");
    Esplora.writeRGB(0, 255, 0); //Turn RGB LED ON and make green   
  }
  else{
    if (DEBUG) Serial.println("HARDWARE ERORR");
    Esplora.writeRGB(255, 0, 0); //Turn RGB LED ON and make red   
  }

}//END MAIN LOOP


