#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <StopWatch.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.
   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground
    
   History
   =======
   2015/10/22 Initial code base
*/

/***************************************************************************
 AVERAGE CUTDOWN TIME CONSTANTS
 ***************************************************************************/
 enum
 {
   BALLOON_HY_1600_100_000_FEET = 5379,
   BALLOON_HY_1600_75_000_FEET  = 4034,  
   BALLOON_HY_1600_X_50_000_FEET  = 2689
}; //END ENUM 


/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

StopWatch SW_CutDown(StopWatch::SECONDS);  // Initialize to full seconds
StopWatch SWarray[5];                      // Defaults to milliseconds

/*!
 * @brief START running timer.
 */
void startCutDownTimer(void)
{
  SW_CutDown.start();
}

/*!
 * @brief STOP timer until restartCutDownTimer() or startCutDownTimer() function is called.
 */
void stopCutDownTimer(void)
{
  SW_CutDown.stop();
}

/*!
 * @brief Reset timer to zero seconds.
 */
void restartCutDownTimer(void)
{
  SW_CutDown.reset();
}

/*!
 * @brief Get current length timer has run for in seconds
 * @return The current value (Overflows after 49.7 days)
 */
unsigned long getTimerValue(void)
{
  return SW_CutDown.elapsed();
}

/*!
 * @brief Give command to cut down balloon 
 * @return True is cutdown hardware phyically cut wire - False otherwise 
 */
bool cutDownBalloon(void)
{
  
}


/*!
 * @brief START storing temperture (degrees Celsius), Pressure (kPa), and altitude (m) in EEPROM
 * @return True, if EEPROM as room for 8191 (16 Byte) data points - False otherwise
 */
bool startLoggingData(void)
{
  
}

/*!
 * @brief STOP storing temperture (degrees Celsius), Pressure (kPa), and altitude (m) in EEPROM
 */
void stopLoggingData(void)
{
  
}

/*!
 * @brief Turn on the required thermal control system.
 * @param Input temperature reading from sensor.
 */
void startHeatingCoolingSystem(int temperature)
{
  if(checkHeatingCoolingSystem(temperature)){
    // Turn on Heater  
    
  }
  else{
    // Turn off heater and allow heat sink to cool system
      
  }
}

/*!
 * @brief Deterime which thermal system to turn and engage.
 * @param Input temperature reading from sensor.
 * @return True if heating system was turned on - False otherwise
 */
bool checkHeatingCoolingSystem(int temperature)
{
  
}

/*!
 * @brief Turn off the thermal control system
 */
void stopHeatingCoolingSystem()
{
  
}


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
} // END DISPLAYSENSORDETAILS() FUNCTION

/*!
 * @brief Example test code from Github
 * @see https://github.com/adafruit/Adafruit_BMP085_Unified/blob/master/examples/sensorapi/sensorapi.pde
 */
void unitTest(void)
{
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
} // END UNITTEST() FUNCTION


/**************************************************************************
 Arduino setup function (automatically called at startup)
 **************************************************************************/
void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Overview One Pressure Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
} // END SETUP() FUNCTION

/***************************************************************************
 GLOBAL VARIABLES
 ***************************************************************************/
float currentTemperature;
bool button1State = false;
bool button2State = false;
bool button3State = false;
bool button4State = false;


void loop(void) 
{
  
  unitTest();
  
  if(button1State){
    startCutDownTimer();  
    
  }
  
  if(button2State){
    startLoggingData();
  }

  while(button3State){
    button3State = true;
    bmp.getTemperature(&currentTemperature);
    startHeatingCoolingSystem(currentTemperature); 

    if(getTimerValue() > BALLOON_HY_1600_100_000_FEET){
      cutDownBalloon();  
    }
    else{
      //DO NOTHING
    }
  
    if(button4State){
      stopHeatingCoolingSystem();
      button3State = false;
    }
  } //END WHILE LOOP
    
} //END MAIN LOOP
