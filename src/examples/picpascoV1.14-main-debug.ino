/* picopasco 
 *  Sensor monitor and control firmware for an economical and scalable multinutrient growing system using an RP2040 pico and standard off the shelf sensors and components.
 *  Reads air temp, humidity, VOC, CO2, Soil H2O, TDS, EC, and water temp sensors interval per t_loop.
 *  Dispenses nutrient solutions based upon user set points: fill volume and concentrations of contituents. 
 *  additional libraries required for a successfull build:
 *  https://github.com/Seeed-Studio/SGP30_Gas_Sensor
 *  https://www.arduinolibraries.info/libraries/sensirion-i2-c-sht4x
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
int analogBuffer[SCOUNT], analogBufferTemp[SCOUNT], analogBufferIndex = 0,copyIndex = 0, sensorValue = 0; float averageVoltage = 0,tdsValue = 0,temperaturetds = 25, temperature, humidity; uint32_t serialNumber; uint16_t error;char errorMessage[256];String printStr = "";
const int n = 4, ctl[n] = {6,7,20,21}, trigger = LOW;const int t_loop = 100; //number of n relays // define n pins in n array // init trigger state. // prepost t_loop adjust sensor sampling and serial prints.
const long tMixSP[n] = {100,200,500,1000};const long MixIntSP = 10000; // SP dwell at trigger HIGH state per relay soleniods positions {S1-PartA, S2-PartB, S3-PartC, S4-Water} Turn off with 0 [ms]. //auto mix fill interval [ms].
long MixIntPV;long tMixPV0[n]; //next time to mix millis() // PV elapsed dwell at trigger HIGH state per relay n.
int Mix = 0 ; int TriggerMix = 0 ;int state[n];int cntOff = 0; // 1 - state detect mixing; 0 - not mixing  // 1 - trigger Mix on next loop; 0 - trigger on MixIntSP//trigger state of relays of n array //verify relay trigger off state after mix 
s16 err = 0; u32 ah = 0; u16 scaled_ethanol_signal, scaled_h2_signal;SensirionI2CSht4x sht4x;
void setup() {
  Serial.begin(115200);Wire.setSDA(PIN_WIRE0_SDA); Wire.setSCL(PIN_WIRE0_SCL);  Wire.begin();//update default I2C0 pins needs to be here and in pins_arduino.h to function begin I2C0
  MixIntPV =  MixIntSP + millis(); // init MixIntPV mix start time [ms]
  for(int i=0; i < n; i++) {
    pinMode(ctl[i], OUTPUT);// set relay trigger pin as output
    tMixPV0[i] = MixIntPV + tMixSP[i]; //init mix end time
    if(trigger ==LOW){ //set trigger init state LOW
      digitalWrite(ctl[i], HIGH); state[i]= 1; }
    else{
      digitalWrite(ctl[i], LOW); state[i]= 0; }
  }
  sht4x.begin(Wire); error = sht4x.serialNumber(serialNumber);
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
  delay(t_loop/2); cntOff = 0; u16 tvoc_ppb, co2_eq_ppm; error = sht4x.measureHighPrecision(temperature, humidity); //TempRH
  if (error) {
      Serial.print("Error trying to execute measureHighPrecision(): "); errorToString(error, errorMessage, 256); Serial.println(errorMessage); } 
  else {
      printStr = String(millis())+String(">T[C],") + temperature + ",RH[%]," + humidity; }
  err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);  //VOC CO2
  if (err == STATUS_OK) {
     printStr = printStr + ",tVOC[ppm])," + tvoc_ppb + ",CO2eq[ppm]," + co2_eq_ppm;}
  else {
     Serial.println("error reading IAQ values\n"); }
  averageVoltage =  analogRead(TdsSensorPin) * (float)VREF/ 1024.0; float compensationCoefficient=1.0+0.02*(temperaturetds-25.0); float compensationVoltage=averageVoltage/compensationCoefficient; tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5; sensorValue = analogRead(moisturePin); // SoilH2O 
  printStr = printStr + ",Vtc[v]," + compensationVoltage + ",TDS[ppm]," + tdsValue + ",Soil," + sensorValue ; //SENSOR end
  if ((millis() > MixIntPV)and(Mix==0)){ // MIXING start
    for (int k =0 ; k<n ; k++){ 
      digitalWrite(ctl[k], HIGH); state[k] = 1; 
      if (k ==(n -1)){ 
        Mix=1 ;  TriggerMix = 0; Serial.println("#!*************Start Mix*************");} 
      } 
    } 
  for (int j =0; j<n; j++) { 
    if ((Mix ==1)and(tMixPV0[j] < millis())) { 
      digitalWrite(ctl[j], LOW); state[j]=0;cntOff = cntOff + 1;
      }
      if (cntOff == n){
        cntOff=0; MixIntPV =  MixIntSP + millis(); Serial.println("#!*************End Mix*************"); 
        for (int m=0;m<n;m++){
          tMixPV0[m] = MixIntPV + tMixSP[m];}
        }
    printStr = printStr+",S"+j+","+state[j]+","+(tMixPV0[j]-millis())+","+(tMixSP[j])+",tMixPV0,"+tMixPV0[j]+"," ;  
    } //MIXING end 
  printStr = printStr+","+(MixIntPV-millis())+"<"+millis()+"MIX state,"+Mix; Serial.println(String(printStr));
  delay(t_loop/2);
} 
