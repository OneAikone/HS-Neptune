#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
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

#define SD_ChipSelect 10

DFRobot_BMI160 bmi160; // Accelerometer
const int8_t i2c_addr = 0x69; // I2C Address for light sensor

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // Light intensity; pass in a number for the sensor identifier (for your use later)
Adafruit_BME280 bmeInside; // use I2C address 0x76

Adafruit_BME280 bmeOutside; // use I2C address 0x77

int16_t accelGyro[6];

uint16_t lightReadings[2] = {0,0};
float lux = 0;

File dataFile;

void dispLightSensorDetails(void)
{

  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  delay(500);
}

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

  // Prevent multisensor + SD from freezing the code indefinitely sometimes
  Wire.setWireTimeout(2000, true);

  pinMode(buzzerPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  pinMode(laserPin, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(SD_ChipSelect, OUTPUT);

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

  if (!bmeOutside.begin(0x77)) {
    Serial.println(F("outside MultiSensor not g"));
  }
  else {
    Serial.println(F("Foutside BME280 g"));
  }

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
  digitalWrite(A1, HIGH);
  digitalWrite(laserPin, HIGH);
  digitalWrite(valvePin, HIGH);
  
  // Multi sensor
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

  // Light sensor
  readLightIntensity();

  Serial.println(F("\nPRE-FLIGHT"));

  // Multi-sensor data
  Serial.print(F("Temp Inside = "));
  Serial.print(temp1);
  Serial.println(" *C");
  Serial.print(F("Hum Inside = "));
  Serial.print(hum1);
  Serial.println(" %");
  Serial.print(F("Pres Inside = "));
  Serial.print(pres1);
  Serial.println(" hPa");
  Serial.print("Approx. Alt = ");
  Serial.print(altitude1);
  Serial.println(" m\n\n");

  
  Serial.print(F("Temp Outside = "));
  Serial.print(temp2);
  Serial.println(" *C");
  Serial.print(F("Hum Outside = "));
  Serial.print(hum2);
  Serial.println(" %");
  Serial.print(F("Pres Outside = "));
  Serial.print(pres2);
  Serial.println(" hPa");
  Serial.print("Approx. Alt = ");
  Serial.print(altitude2);
  Serial.println(" m\n\n");

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
