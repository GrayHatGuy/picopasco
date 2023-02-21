/* picoponic 
 *  Sensor monitor and control firmware for an economical and scalable hydroponic system using an RP2040 pico and standard off the shelf sensors and components.
 *  
 *  Reads air temp, humidity, VOC, CO2, Soil H2O, TDS, EC, and water temp.
 *  Intended use for sensor metrics as a feed forward mechanism for closed loop control of lighting, water, and air quality. 
 *  
 *  additional libraries required for a successfull build:
 *  https://github.com/Seeed-Studio/SGP30_Gas_Sensor
 *  https://www.arduinolibraries.info/libraries/sensirion-i2-c-sht4x
 *  custom include for pico pin map
 *  #include "pins_arduino.h" 
 *      
 *  mailto://GrayHatGuy@GrayHatGuy.com
 */
 
#include <Arduino.h>
#include <SensirionI2CSht4x.h>
#include <Wire.h>
#include "pins_arduino.h"
#include <SoftwareSerial.h>
#include "sensirion_common.h"
#include "sgp30.h"
#define TdsSensorPin A1
#define VREF 5.0 // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0, loopdelay = 80; //loopdelay applied to begin and end of loop
float averageVoltage = 0,tdsValue = 0,temperaturetds = 25;
int sensorPin = A2; int sensorValue = 0;  
uint32_t serialNumber;
uint16_t error;
char errorMessage[256];
s16 err;
u32 ah = 0;
u16 scaled_ethanol_signal, scaled_h2_signal;
SensirionI2CSht4x sht4x;
void setup() {
  Serial.begin(115200);
  Wire.setSDA(PIN_WIRE0_SDA);
  Wire.setSCL(PIN_WIRE0_SCL);
  Wire.begin();
  sht4x.begin(Wire);
  error = sht4x.serialNumber(serialNumber);
  while (sgp_probe() != STATUS_OK) {
      Serial.println("SGP failed");
      while (1); }
  err = sgp_measure_signals_blocking_read(&scaled_ethanol_signal,&scaled_h2_signal);
  if (err == STATUS_OK) {
      Serial.println("requesting ram signal! - Please hold..."); } 
  else {
      Serial.println("error reading signals"); }
  sgp_set_absolute_humidity(13000); /* Set absolute humidity to 13.000 g/m^3 adjust calibration*/
  err = sgp_iaq_init();
  if (error) {
      Serial.print("Error trying to execute serialNumber(): "); errorToString(error, errorMessage, 256); Serial.println(errorMessage); } 
  else {
      Serial.print("Serial Number: "); Serial.println(serialNumber); Serial.println(); }
}
void loop() {
  delay(loopdelay);    
  uint16_t error;
  char errorMessage[256];
  float temperature;                               
  float humidity;
  s16 err = 0;                                    
  u16 tvoc_ppb, co2_eq_ppm;
  error = sht4x.measureHighPrecision(temperature, humidity); //TempRH
  if (error) {
      Serial.print("Error trying to execute measureHighPrecision(): "); errorToString(error, errorMessage, 256); Serial.println(errorMessage); } 
  else {
      Serial.print("T[C],"); Serial.print(temperature); Serial.print(",RH[%],"); Serial.print(humidity); }  
  err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);  //VOC CO2
  if (err == STATUS_OK) {
     Serial.print(",tVOC[ppm],"); Serial.print(tvoc_ppb); Serial.print(",CO2eq[ppm],"); Serial.print(co2_eq_ppm); } 
  else {
     Serial.println("error reading IAQ values\n"); }
  averageVoltage =  analogRead(TdsSensorPin) * (float)VREF/ 1024.0; //TDS - read the analog value more stable by the median filtering algorithm, and convert to voltage value
  float compensationCoefficient=1.0+0.02*(temperaturetds-25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  float compensationVoltage=averageVoltage/compensationCoefficient; //temperature compensation
  tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5; //convert voltage value to tds value
  sensorValue = analogRead(sensorPin); // SoilH2O 
  Serial.print(",Vtc[v], "); Serial.print(compensationVoltage,2); Serial.print(",TDS[ppm], ");Serial.print(tdsValue,0); Serial.print(",Soil," ); Serial.print(sensorValue); Serial.print(",t0[ms]," ); Serial.println(millis());     
  delay(loopdelay);
}
