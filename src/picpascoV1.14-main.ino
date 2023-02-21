/* picopasco 
 *  Sensor monitor and control firmware for an economical and scalable multinutrient growing system using an RP2040 pico and standard off the shelf sensors and components.
 *  
 *  Reads air temp, humidity, VOC, CO2, Soil H2O, TDS, EC, and water temp sensors interval per t_loop.
 *  t_loop = 80; // pre/post t_loop to adjust sensor sampling and serial prints.
 *  
 *  Dispenses nutrient solutions based upon user set points: fill volume and concentrations of contituents. 
 *  tMixSP[n] = {500,1200,2000,3000}; // SP dwell at trigger HIGH state per relay/soleniods positions n=4 {PartA,PartB,PartC,PartD}
 *  
 *  
 *  
 *  int Mix = 0 ; // 1 mixing state in request in process 0 not in mixing state
 *  const long MixIntSP = 10000; //auto mix fill interval [ms]. Timer for nutruient fill and mix. 
 *  n = 4 // define n relay control GPIO pins.
 *  MixIntSP = 30000; //auto mix fill interval
 *  ctl[n] = {6,7,20,21} //location GPIO pins n=4 relays {S1,S2,S3,S4}*
 *  trigger = LOW; // init trigger state
 *  *see custom pico pin map 
 *  #include "pins_arduino.h" 
 *  
 *  additional libraries required for a successfull build:
 *  https://github.com/Seeed-Studio/SGP30_Gas_Sensor
 *  https://www.arduinolibraries.info/libraries/sensirion-i2-c-sht4x
 *  

 *  Future state: 
 *  PID closed loop control using user setpoint TDS/ec water quality measurement.
 *      
 *  mailto://GrayHatGuy@GrayHatGuy.com
 */
 
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

//Sensor variables 
int analogBuffer[SCOUNT], analogBufferTemp[SCOUNT], analogBufferIndex = 0,copyIndex = 0, sensorValue = 0;  
float averageVoltage = 0,tdsValue = 0,temperaturetds = 25, temperature, humidity;
uint32_t serialNumber; 
uint16_t error;
char errorMessage[256];
String printStr = "";String tmp = "";

//User setpoints:
int t_loop = 120; // prepost t_loop adjust sensor sampling and serial prints.
const int n = 4, ctl[n] = {6,7,20,21}, trigger = LOW; //number of n relays // define n pins in n array // init trigger state.
unsigned long tMixSP[n] = {100,200,500,1000}; // SP dwell at trigger HIGH state per relay soleniods positions {S1-PartA, S2-PartB, S3-PartC, S4-Water} Turn off with 0 [ms].
const long MixIntSP = 10000; //auto mix fill interval [ms].

//Time and state vars init
long MixIntPV; //elapsed time since last mix
long tMixPV0[n]; // PV elapsed dwell at trigger HIGH state per relay n.
int Mix = 0 ; // 1 - state detect mixing; 0 - not mixing 
int TriggerMix = 0 ; // 1 - trigger Mix on next loop; 0 - trigger on MixIntSP
int state[n];  //trigger state of relays of n array 
s16 err = 0;
u32 ah = 0;
u16 scaled_ethanol_signal, scaled_h2_signal;
SensirionI2CSht4x sht4x;

void setup() {
  Serial.begin(115200);
  Wire.setSDA(PIN_WIRE0_SDA); Wire.setSCL(PIN_WIRE0_SCL);  Wire.begin();//update default I2C0 pins needs to be here and in pins_arduino.h to function begin I2C0
  MixIntPV =  MixIntSP + millis(); // init MixIntPV mix start time [ms]
  for(int i=0; i < n; i++) {
    pinMode(ctl[i], OUTPUT);// set relay trigger pin as output
    tMixPV0[i] = MixIntPV + tMixSP[i]; //init mix end time
    if(trigger ==LOW){ //set trigger init state LOW
      digitalWrite(ctl[i], HIGH); state[i]= 1; }
    else{
      digitalWrite(ctl[i], LOW); state[i]= 0; }
  }
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
  
  delay(t_loop/2);  
//Sensors start
  u16 tvoc_ppb, co2_eq_ppm;  
  error = sht4x.measureHighPrecision(temperature, humidity); //TempRH
  if (error) {
      Serial.print("Error trying to execute measureHighPrecision(): "); errorToString(error, errorMessage, 256); Serial.println(errorMessage); } 
  else {
      printStr = String(millis())+String(">T[C],") + temperature + ",RH[%]," + humidity; }
  err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);  //VOC CO2
  if (err == STATUS_OK) {
     printStr = printStr + ",tVOC[ppm])," + tvoc_ppb + ",CO2eq[ppm]," + co2_eq_ppm;}
  else {
     Serial.println("error reading IAQ values\n"); }
  //Analog conversions   
  averageVoltage =  analogRead(TdsSensorPin) * (float)VREF/ 1024.0; //TDS - read the analog value more stable by the median filtering algorithm, and convert to voltage value
  float compensationCoefficient=1.0+0.02*(temperaturetds-25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  float compensationVoltage=averageVoltage/compensationCoefficient; //temperature compensation
  tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5; //convert voltage value to TDS value
  sensorValue = analogRead(moisturePin); // SoilH2O 
  
  printStr = printStr + ",Vtc[v]," + compensationVoltage + ",TDS[ppm]," + tdsValue + ",Soil," + sensorValue ; 

  //placeholder move after nested for append servos/
  Serial.println(String(printStr)); 
//Sensor end 

// MIXING on off handling

  for (int j =0; j<n; j++) {
    //replace if and else with while loop same conditions
    while ((((MixIntPV-millis())<=0)and(Mix==0)or(TriggerMix = 1))){ //if mix interval triggered start mix all triggers high/open
//YOU ARE HERE -> (!*!)
      for (int k =0;k<n;k++){ 
        digitalWrite(ctl[k], HIGH); state[k] = 1; // for loop turn all on same time update trigger state
        if (k ==(n -1)){ 
          //start mix timers //calculate mix end time (!*!)
          Mix=1 ;  TriggerMix = 0; Serial.println("#!*************Start Mix*************");} 
      } //exit for  
    } 
    if ((Mix ==1)and(tMixPV0[j]-millis())<=0) { //check dwell ovr
      digitalWrite(ctl[j], LOW);state[j]=0; //// //add checkoff state //Mix=1 timeout dwell actions //print values  //zero out mix // zero mix interval reset PV dwells //print all pumpts low! (!*!)
    } //exit if  - append results of pumps? (!*!)
  } //exit for end MIXING print all sensor and servo states
  delay(t_loop/2);

} //end loop  
/* Tidbits->
 * if (((tMixPV0[relay]-millis()) <=0) and (state[relay] == 1)and (Mix==1)) { //If mix time ends
 * digitalWrite(ctl[relay], LOW); state[relay] = 0; MixIntPV =  MixIntSP + millis(); tMixPV0[relay] = MixIntPV + tMixSP[relay]; //reset MixInt timer when all relays go off. //Trigger all LOW capture state reset mix timers         
 * tmp = String(tMixSP[relay]-millis()-tMixPV0[relay]);//relay OFF countdown   uq# verify all print lines
 * printStr = printStr +",S"+(relay+1)+","+state[relay]+","+tmp+","+tMixSP[relay]; //uq# verify all print lines
 * MixNext =  MixIntSP-(millis()-MixIntPV); // uq# excess use MixIntPV-millis()
 * printStr = printStr+",MixNext [ms]: "+MixNext; Serial.println(String(printStr)+" <"+millis()); //uq# verify all print lines
 */
