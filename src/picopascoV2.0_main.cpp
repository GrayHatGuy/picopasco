/* picopasco 
 *  Sensor monitor and control firmware for an economical and scalable multinutrient growing system using an RP2040 pico and standard off the shelf sensors and components. Reads air temp, humidity, VOC, CO2, Soil H2O, TDS, EC, and water temp sensors interval per t_loop. Dispenses nutrient solutions based upon user set points: fill volume and concentrations of contituents. 
 *  additional libraries required for a successfull build: https://github.com/Seeed-Studio/SGP30_Gas_Sensor * https://www.arduinolibraries.info/libraries/sensirion-i2-c-sht4x
 *  contact mailto://GrayHatGuy@GrayHatGuy.com */
#include <Arduino.h>
#include <SensirionI2CSht4x.h>
#include <Wire.h>
#include "pins_arduino.h" //include custom header
#include <SoftwareSerial.h>
#include "sensirion_common.h"
#include "sgp30.h"
#define TdsSensorPin A1 //set analog pins
#define moisturePin A2
#define VREF 5.0 // analog reference voltage(Volt) of the ADC.
#define SCOUNT 30 // sum of sample poin.t
int ledState = LOW, t_loop = 0, mix = 0, mixIntSP = 10000,analogBuffer[SCOUNT], analogBufferTemp[SCOUNT], analogBufferIndex = 0,copyIndex = 0, sensorValue = 0; float averageVoltage = 0,tdsValue = 0,temperaturetds = 25;
float temperature, humidity; uint32_t serialNumber; uint16_t error;char errorMessage[256], mixIntPV; //next time to mix millis() // PV elapsed dwell at trigger HIGH state per relay n.
const int n = 4, ctl[n] = {6,7,20,21} , trigger = HIGH, tmixSP[n] = {100,200,500,1000}, ledPin =  LED_BUILTIN, minloop = 60, blinkloops = 10; 
s16 err; u32 ah = 0; u16 scaled_ethanol_signal, scaled_h2_signal;
SensirionI2CSht4x sht4x;
void setup() {
  Serial.begin(115200);Wire.setSDA(PIN_WIRE0_SDA); Wire.setSCL(PIN_WIRE0_SCL);Wire.begin();pinMode(ledPin, OUTPUT);
  mixIntPV = mixIntSP + millis(); 
  for(int i=0; i < n; i++) {
    pinMode(ctl[i], OUTPUT);
    digitalWrite(ctl[i], LOW); 
  }
  while (t_loop < minloop){
    t_loop = minloop;}
  sht4x.begin(Wire); sht4x.serialNumber(serialNumber); err = sgp_measure_signals_blocking_read(&scaled_ethanol_signal,&scaled_h2_signal);
  sgp_set_absolute_humidity(13000); sgp_iaq_init(); /* Set absolute humidity to 13.000 g/m^3 adjust calibration*/
}
void loop() {
  delay(t_loop/2); String meas = scan(); Serial.println(meas);
  if (millis() > mixIntPV){ mix = 1; }
  while (mix == 1){ stiritup(); }
  delay(t_loop/2);
} 
String scan(){ 
    String printscan = " Start Scan ";
    //TDS - read the analog value more stable by the median filtering algorithm, and convert to voltage value //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));//convert voltage value to tds value
    u16 tvoc_ppb, co2_eq_ppm; 
    err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
    error = sht4x.measureHighPrecision(temperature, humidity); 
    averageVoltage =  analogRead(TdsSensorPin) * (float)VREF/ 1024.0; 
    float compensationCoefficient=1.0+0.02*(temperaturetds-25.0); float compensationVoltage=averageVoltage/compensationCoefficient; 
    tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;   
    printscan = printscan + temperature + humidity + co2_eq_ppm + tvoc_ppb + tdsValue + compensationVoltage + analogRead(moisturePin); return printscan;
}
void stiritup(){
  String printmix = " Start Mix ";
  for (int m =0; m<n; m++){
    int pumpon = (millis() + tmixSP[m]); digitalWrite(ctl[m], HIGH);printmix = printmix + " Pump "+String(m)+" ON! for: "+tmixSP[m]+ " ms @ time: " + pumpon ;  delay(tmixSP[m]);Serial.println(printmix);
    while (pumpon < millis()){
      Serial.print("Pump "+String(m)+" Time Remaining "); Serial.println(String(pumpon-millis())); 
      }
    Serial.print("Pump "+String(m)+" complete! ");Serial.print("Dispensed for " + String((millis()-pumpon))+" ms");  mix = 0; Serial.println();
  }
  Serial.print("mix complete! Total time: " + String((millis()-mixIntPV))); mixIntPV =  mixIntSP + millis(); Serial.println(); 
}
