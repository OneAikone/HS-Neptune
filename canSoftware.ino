#include <Wire.h>
#include <SPI.h>
#include <LoRa.h> // Long Range Communication
#include <Adafruit_BME280.h> // Multi-sensor (pressure, humidity, smth)
#include <Adafruit_Sensor.h> // All adafruit
#include "Adafruit_TSL2591.h" // Light intensity
#include <DFRobot_BMI160.h> // Accelerometer
// connect SCL to I2C Clock
// connect SDA to I2C Data
// connect Vin to 3.3-5V DC
// connect GROUND to common ground


// For LoRa
#define SS 5
#define RST 17s
#define DIO0 4

DFRobot_BMI160 bmi160; // Accelerometer
const int8_t i2c_addr = 0x69; // I2C Address

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // Light intensity; pass in a number for the sensor identifier (for your use later)
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

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
  Serial.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
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
  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
  Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
  Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);
}

void setup(void)
{
  Serial.begin(9600);

  // Light setup
  if (tsl.begin()) {
    Serial.println(F("Found a TSL2591 sensor"));
  } 
  else {
    Serial.println(F("No TSL2591 Light intensity sensor found ... check your wiring?"));
  }
  configureLightSensor();
  dispLightSensorDetails();

  // Multi-sensor setup
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME280 mult-sensor, check wiring!"));
  }
  else {
    Serial.println(F("Found a BME280 sensor"));
  }
  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();

  // LoRa setup
  pinMode(LED_BUILTIN, OUTPUT);
  LoRa.setPins(SS, RST, DIO0);

  if(LoRa.begin(868E6)){
    LoRa.setSyncWord(0x34);          // 0-0xFF sync word to match the receiver
    LoRa.setSpreadingFactor(12);     // (6-12) higher value increases range but decreases data rate
    LoRa.setSignalBandwidth(125E3);  // lower value increases range but decreases data rate
    LoRa.setCodingRate4(8);          // higher value increases range but decreases data rate
    LoRa.enableCrc();                // improves data reliability
  }
  else{
    Serial.println(F("Failed to initialise LoRa, check wiring!"));
  }

  // Accelerometer setup
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println(F("Failed to initialise Accelerometer, check wiring ig gng"));
  }
}

// Flight stages; 0 -> pre-flight; 1 -> Ascent (atomizer enables); 2 -> Descent (valve opens, atomizer disables); 3 -> Landed (Valve closes, buzzer activates)
int mode = 0;
int temporaryCounter = 0;

void loop(void)
{

  delay(1000);
  // Light intensity
  readLightIntensity();

  // Multi sensor
  sensors_event_t temp_event, pressure_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);
  
  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  Serial.print(F("Humidity = "));
  Serial.print(humidity_event.relative_humidity);
  Serial.println(" %");

  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");

  // Lora
  digitalWrite(LED_BUILTIN, HIGH);
  LoRa.beginPacket();
  LoRa.print("Hello, World! ");
  LoRa.print(temp_event.temperature);
  LoRa.endPacket();
  digitalWrite(LED_BUILTIN, LOW);

  // Accelerometer
  int16_t accelData[3]={0}; 
  
  //get acceleration data from bmi160
  //parameter accelData is the pointer to store the data
  int result = bmi160.getAccelData(accelData);
  if (result == 0) {
    for (int i = 0; i < 3; i++){
      //the following three data are accel data
      Serial.print(accelData[i]/16384.0);Serial.print("\t");
    }
    Serial.println();
  }
  else {
    Serial.println("Error getting Acceleration data!");
  }

}