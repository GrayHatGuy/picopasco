/* picopasco zcan all 7 sensors with blink() led dditional libraries required for a successfull build: https://github.com/Seeed-Studio/SGP30_Gas_Sensor https://www.arduinolibraries.info/libraries/sensirion-i2-c-sht4x  *  #include "pins_arduino.h" * mailto://GrayHatGuy@GrayHatGuy.com */
#include <Arduino.h>
#include <SensirionI2CSht4x.h>
#include <Wire.h>
#include "pins_arduino.h" //custom header for seeed grove pico breakout
#include <SoftwareSerial.h>
#include "sensirion_common.h"
#include "sgp30.h"
#define TdsSensorPin A1 
#define moisturePin A2
#define VREF 5.0 // TDS ADC ref.
#define SCOUNT 30 // TDS voltage samples
const int mixIntSP = 25000, n =4; int ledPin = LED_BUILTIN, t_loop = 60, ledState = LOW, ctl[n] = {6,7,20,21}, trigger = LOW, tmixSP[n] = {2000,2000,5000,10000}; //user inputs for UX function inputs
unsigned long previousMillis = millis(); float mixIntPV = mixIntSP + millis(); 
int mix = 0, analogBuffer[SCOUNT], analogBufferTemp[SCOUNT], analogBufferIndex = 0,copyIndex = 0, sensorValue = 0; float averageVoltage = 0,tdsValue = 0,temperaturetds = 25, temperature, humidity; uint32_t serialNumber; uint16_t error; char errorMessage[256];s16 err = 0; u32 ah = 0; u16 scaled_ethanol_signal, scaled_h2_signal; SensirionI2CSht4x sht4x; //sensor() globals
void setup() { 
  Serial.begin(115200); Wire.setSDA(PIN_WIRE0_SDA); Wire.setSCL(PIN_WIRE0_SCL); Wire.begin(); for(int i=0; i < n; i++) { pinMode(ctl[i], OUTPUT); digitalWrite(ctl[i], LOW);  } ; sht4x.begin(Wire); error = sht4x.serialNumber(serialNumber); while (sgp_probe() != STATUS_OK) { Serial.println("SGP failed"); while (1); } ; err = sgp_measure_signals_blocking_read(&scaled_ethanol_signal,&scaled_h2_signal); if (err == STATUS_OK) { Serial.println("requesting ram signal! - Please hold..."); } else { Serial.println("error reading signals"); } ; sgp_set_absolute_humidity(13000); err = sgp_iaq_init(); if (error) { Serial.print("Error trying to execute serialNumber(): "); errorToString(error, errorMessage, 256); Serial.println(errorMessage); } else { Serial.print("Serial Number: "); Serial.println(serialNumber); Serial.println(); } } //sensors //spi i2c setup //relay setup //SHT4x setup // sgp30 setup 
void loop() { 
  delay(t_loop/2); if (millis() > mixIntPV){ mix = 1; Serial.println("MIX = 1"); } sensors(); blink(-1); if (mix == 1){ blink(-1);stiritup(); blink(-1) ;}; delay(t_loop/2); } 
void blink(long led_int) { 
  pinMode(ledPin, OUTPUT);unsigned long currentMillis = millis(); if (currentMillis - previousMillis >= led_int) { previousMillis = currentMillis; if (ledState == LOW) { ledState = HIGH; } else { ledState = LOW; } ; digitalWrite(ledPin, ledState); }; if (led_int < 0){ if (ledState == LOW) { ledState = HIGH; } else { ledState = LOW; } ; digitalWrite(ledPin, ledState); } }
void sensors() { 
  String printStr = ""; error = sht4x.measureHighPrecision(temperature, humidity); printStr = String(millis())+"> t," + temperature + ",rh," + humidity; u16 tvoc_ppb, co2_eq_ppm; err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm); if (err == STATUS_OK) { printStr = printStr + ",voc," + tvoc_ppb + ",co2," + co2_eq_ppm;} averageVoltage =  analogRead(TdsSensorPin) * (float)VREF/ 1024.0; float compensationCoefficient=1.0+0.02*(temperaturetds-25.0); float compensationVoltage=averageVoltage/compensationCoefficient; tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5; sensorValue = analogRead(moisturePin); printStr = printStr + ",ec," + compensationVoltage + ",tds," + tdsValue + ",h20," + sensorValue ; Serial.println(String(printStr)); } //Sensor read
void stiritup(){
  String pspool = "giddyup>"; for (int m =0; m<n; m++){ int pumpon = (millis() + tmixSP[m]); blink(-1); digitalWrite(ctl[m], HIGH); pspool = " Pump "+String(m+1)+" ON! for "+tmixSP[m]+" at "+millis(); delay(tmixSP[m]); pspool = pspool + " complete! at " + millis(); Serial.println(pspool); }; mix = 0;  mixIntPV =  mixIntSP + millis(); pspool = String(" reset Mix=") + mix + " NEXT MIX at " + String(mixIntPV); Serial.println(pspool); delay(t_loop*10); }