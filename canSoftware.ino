#include <Wire.h>
#include <SPI.h>
#include <SD.h> // SD card (obvious)
#include <LoRa.h>
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

#define SD_ChipSelect 10


DFRobot_BMI160 bmi160; // Accelerometer
const int8_t i2c_addr = 0x69; // I2C Address for light sensor

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // Light intensity; pass in a number for the sensor identifier (for your use later)
Adafruit_BME280 bmeInside; // use I2C address 0x76
Adafruit_BME280 bmeOutside; // use I2C address 0x77

// Flight stages; 0 -> pre-flight; 1 -> Ascent (atomizer enables); 2 -> Descent (valve opens, atomizer disables); 3 -> Landed (Valve closes, buzzer activates)
uint8_t mode = 0;

bool multisensor2Works = true;
bool accelerometerWorks = true;

// For phase change detection using altitude measurement
int highest_point = 0;

// Counters for accelerometer
uint8_t ascentCounter = 0;
uint8_t descentCounter = 0;
uint8_t staticCounter = 0;

int16_t accelGyro[6];

uint16_t lightReadings[2] = {0,0};
float lux = 0;

File dataFile;

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

  // Prevent multisensor + SD from freezing the code indefinitely sometimes
  Wire.setWireTimeout(2000, true);

  pinMode(buzzerPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  pinMode(laserPin, OUTPUT);
  pinMode(SD_ChipSelect, OUTPUT);
  pinMode(A1, OUTPUT);


  // Light setup
  tsl.begin();
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);

  // Multi-sensors setup
  bmeInside.begin(0x76);
  bmeOutside.begin(0x77);

  // Accelerometer setup
  bmi160.I2cInit(i2c_addr);

  // SD card setup
  if(SD.begin(SD_ChipSelect)) {
    Serial.println(F("SD g"));
  }
  else {
    Serial.println(F("SD fail oopsie :<"));
  }
  Wire.begin();

  // LoRa setup
  LoRa.setPins(5, 17, 4); //SS, RST and DIO0
  LoRa.begin(868E6);
  LoRa.setSyncWord(0x26);          // 0-0xFF sync word to match the receiver
  LoRa.setSpreadingFactor(12);     // (6-12) higher value increases range but decreases data rate
  LoRa.setSignalBandwidth(125E3);  // lower value increases range but decreases data rate
  LoRa.setCodingRate4(8);          // higher value increases range but decreases data rate
  LoRa.enableCrc();                // improves data reliability
}

void loop(void)
{
  delay(1000);
  // Multi sensors
  auto temp1 = bmeInside.readTemperature();
  auto temp2 = bmeOutside.readTemperature();
  
  auto pres1 = bmeInside.readPressure() / 100.0F;
  auto pres2 = bmeOutside.readPressure() / 100.0F;

  auto hum1 = bmeInside.readHumidity();
  auto hum2 = bmeOutside.readHumidity();

  auto altitude1 = bmeInside.readAltitude(SEALEVELPRESSURE_HPA);
  auto altitude2 = bmeOutside.readAltitude(SEALEVELPRESSURE_HPA);

  // Accelerometer
  //parameter accelGyro is the pointer to store the data
  int rslt = bmi160.getAccelGyroData(accelGyro);

  // SENSOR FUNCTIONALITY DETECTION
  if(mode != 3) {
    // If altitude2 is NaN, the statement is false
    multisensor2Works = (altitude2 == altitude2);
    // If the first three values are zero, it probably doesn't work
    accelerometerWorks = !(accelGyro[0] == 0 && accelGyro[1] == 0 && accelGyro[2] == 0);

    float accelY = accelGyro[4] / 16384.0;
    if (accelY > 1) {
      ascentCounter++;
      descentCounter = 0;
      staticCounter = 0;
    }
    else if (accelY < -1) {
      ascentCounter = 0;
      descentCounter++;
      staticCounter = 0;
    }
    else {
      ascentCounter = 0;
      descentCounter = 0;
      staticCounter++;
    }
    
  }

  // Pre-flight. Runs checks to see when ascent begins. 
  // [REQUIRES FINAL TWEAKS]
  if(mode == 0){
      Serial.println(altitude1);
      Serial.println(altitude2);
      // Accelerometer data
      // for(int i = 0; i < 6; i++){
      //   if(i < 3) {
      //     //the first three are gyro data
      //     Serial.print(accelGyro[i]*3.14/180.0);Serial.print("\t");
      //   }
      //   else {
      //     //the following three data are accel data
      //     Serial.print(accelGyro[i]/16384.0);Serial.print("\t");
      //   }
      // }
      Serial.println(multisensor2Works, accelerometerWorks);

      if(multisensor2Works && altitude2 > 300){
        Serial.println(F("m->asc"));
        digitalWrite(A1, HIGH);
        mode++;
      }
      else if(!multisensor2Works && accelerometerWorks && ascentCounter > 15) {
        Serial.println(F("b->asc"));
        digitalWrite(A1,  HIGH);
        mode++;
      }
      else{
        Serial.println(F("no asc"));
      }
  }

  // Ascent. Runs checks to see when descent begins.
  else if(mode == 1){
    // Light sensor
    readLightIntensity();

    if (multisensor2Works && altitude2 > highest_point){
      highest_point = altitude2;
    }

    if (multisensor2Works && altitude2 < highest_point - 50){
      Serial.println(F("m->des"));
      mode++;
      digitalWrite(A1, LOW);
      digitalWrite(valvePin, HIGH);
    }
    else if (!multisensor2Works && accelerometerWorks && descentCounter > 10){
      Serial.println(F("b->des"));
      mode++;
      digitalWrite(A1, LOW);
      digitalWrite(valvePin, HIGH);
    }
    else {
      Serial.println(F("no des"));
    }
  }  

  // Descent. Runs checks for hitting the ground 
  // [REQUIRES BETTER DETECTION]
  else if (mode == 2) {
    if (accelerometerWorks & staticCounter > 60) {
      Serial.println(F("b->end"));
      mode++;
      digitalWrite(buzzerPin, HIGH);
      digitalWrite(valvePin, LOW);
    }
    else{
      Serial.println(F("no end"));
    }

  }

  // SD card implementation
  dataFile = SD.open("CanSatDATA.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print(temp1); dataFile.print(F(", "));
    dataFile.print(hum1); dataFile.print(F(", "));
    dataFile.print(pres1); dataFile.print(F(", "));
    dataFile.print(temp2); dataFile.print(F(", "));
    dataFile.print(hum2); dataFile.print(F(", "));
    dataFile.print(pres2); dataFile.print(F(", "));
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

  // LoRa
  LoRa.beginPacket();
  LoRa.print(temp1); LoRa.print(F(", "));
  LoRa.print(hum1); LoRa.print(F(", "));
  LoRa.print(pres1); LoRa.print(F(", "));
  LoRa.print(temp2); LoRa.print(F(", "));
  LoRa.print(hum2); LoRa.print(F(", "));
  LoRa.print(pres2); LoRa.print(F(", "));
  LoRa.print(lightReadings[0]); LoRa.print(F(", "));
  LoRa.print(lightReadings[1]); LoRa.print(F(", "));
  LoRa.print(lux); LoRa.print(F(", "));
  LoRa.print(accelGyro[0]); LoRa.print(F(", "));
  LoRa.print(accelGyro[1]); LoRa.print(F(", "));
  LoRa.print(accelGyro[2]); LoRa.print(F(", "));
  LoRa.print(accelGyro[3]); LoRa.print(F(", "));
  LoRa.print(accelGyro[4]); LoRa.print(F(", "));
  LoRa.print(accelGyro[5]); LoRa.print(F(", "));
  LoRa.endPacket();
}
