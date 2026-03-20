#include <Wire.h>
#include <SPI.h>
#include <SD.h> // SD card (obvious)
#include <Adafruit_BME280.h> // Multi-sensor (pressure, humidity, smth)
#include <Adafruit_Sensor.h> // general adafruit library
#include "Adafruit_TSL2591.h" // Light intensity
#include <DFRobot_BMI160.h> // Accelerometer
// connect SCL to I2C Clock
// connect SDA to I2C Data
// connect Vin to 3.3-5V DC
// connect GROUND to common ground

#define SEALEVELPRESSURE_HPA (1013.25)

#define buzzerPin 7
#define valvePin 8
#define laserPin 9
#define atomizerPin A1

#define OutsideMultisensorAddress 0x77
#define InnerMultisensorAddress 0x76

#define SD_ChipSelect 10


DFRobot_BMI160 bmi160; // Accelerometer
const int8_t i2c_addr = 0x69; // I2C Address for light sensor

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // Light intensity; pass in a number for the sensor identifier (for your use later)
Adafruit_BME280 bmeInside; // use I2C address 0x76
Adafruit_Sensor *bme_temp1 = bmeInside.getTemperatureSensor();
Adafruit_Sensor *bme_pressure1 = bmeInside.getPressureSensor();
Adafruit_Sensor *bme_humidity1 = bmeInside.getHumiditySensor();

Adafruit_BME280 bmeOutside; // use I2C address 0x77
Adafruit_Sensor *bme_temp2 = bmeOutside.getTemperatureSensor();
Adafruit_Sensor *bme_pressure2 = bmeOutside.getPressureSensor();
Adafruit_Sensor *bme_humidity2 = bmeOutside.getHumiditySensor();

// Flight stages; 0 -> pre-flight; 1 -> Ascent (atomizer enables); 2 -> Descent (valve opens, atomizer disables); 3 -> Landed (Valve closes, buzzer activates)
uint8_t mode = 0;

bool multisensor2Works = true;
bool accelerometerWorks = true;

// For phase change detection using altitude measurement
int highest_point = 0;
int altitude = 0;


// Counters for accelerometer
uint8_t ascentCounter = 0;
uint8_t  descentCounter = 0;
uint8_t staticCounter = 0;

int16_t accelGyro[6];

uint16_t lightReadings[2] = {0,0};
float lux = 0;

File dataFile;


void configureLightSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)
  /* Display the gain and integration time for reference sake */  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Timing:       "));
  Serial.print((tsl.getTiming() + 1) * 100, DEC);
  Serial.println(F(" ms"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}

void readLightIntensity(void)
{

  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  //Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
  //Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
  Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);
  lightReadings[0] = full;
  lightReadings[1] = ir;
  lux = tsl.calculateLux(full, ir);
}

void setup(void)
{
  Serial.begin(9600);

  pinMode(buzzerPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  pinMode(laserPin, OUTPUT);
  pinMode(atomizerPin, OUTPUT);


  // Light setup
  if (tsl.begin()) {
    Serial.println(F("TSL g"));
  }
  else {
    Serial.println(F("TSL not g"));
  }

  configureLightSensor();

  // Multi-sensor setup
  if (!bmeInside.begin(0x76)) {
    Serial.println(F("inner MultiSensor not g"));
  }
  else {
    Serial.println(F("inner BME280 g"));
  }
  bme_temp1->printSensorDetails();
  bme_pressure1->printSensorDetails();
  bme_humidity1->printSensorDetails();

  if (!bmeOutside.begin(0x77)) {
    Serial.println(F("outside MultiSensor not g"));
  }
  else {
    Serial.println(F("Foutside BME280 g"));
  }

  bme_temp2->printSensorDetails();
  bme_pressure2->printSensorDetails();
  bme_humidity2->printSensorDetails();


  // Accelerometer setup
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println(F("bmi160 not g ig"));
  }

  // SD card setup
  if(SD.begin(SD_ChipSelect)) {
    Serial.println(F("SD g"));
  }
  else {
    Serial.println(F("SD fail oopsie :<"));
  }
}

// ****************************
// IMPORTANT! REMEMBER TO TEST WHAT THE OUTPUT LOOKS LIKE WHEN THE SENSORS ARE BROKEN!
// ****************************
void loop(void)
{
  delay(1000);

  // Multi sensor
  sensors_event_t temp_event1, pressure_event1, humidity_event1, temp_event2, pressure_event2, humidity_event2;

  bme_temp1->getEvent(&temp_event1);
  bme_pressure1->getEvent(&pressure_event1);
  bme_humidity1->getEvent(&humidity_event1);

  bme_temp2->getEvent(&temp_event2);
  bme_pressure2->getEvent(&pressure_event2);
  bme_humidity2->getEvent(&humidity_event2);

  auto altitude1 = bmeInside.readAltitude(SEALEVELPRESSURE_HPA);
  auto altitude2 = bmeOutside.readAltitude(SEALEVELPRESSURE_HPA);

  // Accelerometer
  //parameter accelGyro is the pointer to store the data
  int rslt = bmi160.getAccelGyroData(accelGyro);

  // Light sensor
  readLightIntensity();

  // [REQUIRES SENSOR FUNCTIONALITY DETECTION]
  if(mode != 3) {
    // TEST FOR SENSORS WORKING CORRECTLY HERE
    // SET TRUE / FALSE FOR multiSensor2Works and accelerometerWorks
    // INCREMENT ascentCounter if ASCENDING AND RESET OTHERWISE
    // INCREMENT descentCountter IF DESCENDING AND RESET OTHERWISE
    // INCREMENT staticCounter IF OBJECT ISN'T MOVING AND RESET OTHERWISE
  }

  // Pre-flight. Runs checks to see when ascent begins. 
  // [REQUIRES FINAL TWEAKS]
  if(mode == 0){
      Serial.println(F("\nPRE-FLIGHT"));

      // Multi-sensor data
      Serial.print(F("Temp Inside = "));
      Serial.print(temp_event1.temperature);
      Serial.println(" *C");
      Serial.print(F("Hum Inside = "));
      Serial.print(humidity_event1.relative_humidity);
      Serial.println(" %");
      Serial.print(F("Pres Inside = "));
      Serial.print(pressure_event1.pressure);
      Serial.println(" hPa");
      Serial.print("Approx. Alt = ");
      Serial.print(altitude1);
      Serial.println(" m\n\n");

      
      Serial.print(F("Temp Outside = "));
      Serial.print(temp_event2.temperature);
      Serial.println(" *C");
      Serial.print(F("Hum Outside = "));
      Serial.print(humidity_event2.relative_humidity);
      Serial.println(" %");
      Serial.print(F("Pres Outside = "));
      Serial.print(pressure_event2.pressure);
      Serial.println(" hPa");
      Serial.print("Approx. Alt = ");
      Serial.print(altitude2);
      Serial.println(" m");

      // Accelerometer data
      for(int i = 0; i < 6; i++){
        if(i < 3) {
          //the first three are gyro data
          Serial.print(accelGyro[i]*3.14/180.0);Serial.print("\t");
        }
        else {
          //the following three data are accel data
          Serial.print(accelGyro[i]/16384.0);Serial.print("\t");
        }
      }
      Serial.println();

      if(multisensor2Works && altitude2 > 300){
        Serial.println(F("multi -> ascent"));
        digitalWrite(atomizerPin, HIGH);
        mode++;
      }
      else if(!multisensor2Works && accelerometerWorks && ascentCounter > 30) {
        Serial.println(F("bmi -> ascent"));
        digitalWrite(atomizerPin,  HIGH);
        mode++;
      }
      else{
        Serial.println(F("no ascent"));
      }
  }

  // Ascent. Runs checks to see when descent begins.
  else if(mode == 1){
    Serial.println(F("\nASCENT"));

    if (multisensor2Works && altitude2 > highest_point){
      highest_point = altitude2;
    }

    if (multisensor2Works && altitude2 < highest_point - 50){
      Serial.println(F("multi -> descent"));
      mode++;
      digitalWrite(atomizerPin, LOW);
      digitalWrite(valvePin, HIGH);
    }
    else if (!multisensor2Works && accelerometerWorks && descentCounter > 10){
      Serial.println(F("bmi -> descent"));
      mode++;
      digitalWrite(atomizerPin, LOW);
      digitalWrite(valvePin, HIGH);
    }
    else {
      Serial.println(F("no descent"));
    }
  }  

  // Descent. Runs checks for hitting the ground 
  // [REQUIRES BETTER DETECTION]
  else if (mode == 2) {
    Serial.println(F("\nDESCENT"));

    if (accelerometerWorks & staticCounter > 30) {
      Serial.println(F("bmi -> end"));
      mode++;
      digitalWrite(buzzerPin, HIGH);
      digitalWrite(valvePin, LOW);
    }
    else{
      Serial.println(F("no end"));
    }

  }

  dataFile = SD.open("CanSatDATA.csv", FILE_WRITE);;
  if (dataFile) {
    dataFile.print(temp_event1.temperature); dataFile.print(F(", "));
    dataFile.print(humidity_event1.relative_humidity); dataFile.print(F(", "));
    dataFile.print(pressure_event1.pressure); dataFile.print(F(", "));
    dataFile.print(temp_event2.temperature); dataFile.print(F(", "));
    dataFile.print(humidity_event2.relative_humidity); dataFile.print(F(", "));
    dataFile.print(pressure_event2.pressure); dataFile.print(F(", "));
    dataFile.print(lightReadings[0]); dataFile.print(F(", "));
    dataFile.print(lightReadings[1]); dataFile.print(F(", "));
    dataFile.print(lux); dataFile.print(F(", "));
    dataFile.print(accelGyro[0]); dataFile.print(F(", "));
    dataFile.print(accelGyro[1]); dataFile.print(F(", "));
    dataFile.print(accelGyro[2]); dataFile.print(F(", "));
    dataFile.print(accelGyro[3]); dataFile.print(F(", "));
    dataFile.print(accelGyro[4]); dataFile.print(F(", "));
    dataFile.print(accelGyro[5]); dataFile.print(F(", "));
    Serial.println(F("Saved data to SD."));
    dataFile.close();
  }
  else{
    Serial.println(F("SD save failed."));
  }


  // [LORA IMPLEMENTATION NEEDED] (probably sx1276)


}