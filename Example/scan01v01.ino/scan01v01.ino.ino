/* picoponic 
 *  Central sensor monitor and control firmware for an economical and scalable hydroponic system.
 *  
 *  Reads air temp, humidity, VOC, CO2, Soil H2O, TDS, EC, and water temp.
 *  Sensor metrics will eventually be used as a feed forward mechanism for closed loop 
 *  control of lighting, mixing, irrigation, and air quality/HVAC.  
 *  
 *  additional libraries required for a successfull build:
 *  https://github.com/Seeed-Studio/SGP30_Gas_Sensor
 *  https://www.arduinolibraries.info/libraries/sensirion-i2-c-sht4x
 *      
 *  mailto://GrayHatGuy@GrayHatGuy.com
 */
 
#include <Arduino.h>
#include <SensirionI2CSht4x.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "sensirion_common.h"
#include "sgp30.h"


SensirionI2CSht4x sht4x;
void setup() 
{
    s16 err;
    u32 ah = 0;
    u16 scaled_ethanol_signal, scaled_h2_signal;
    uint32_t serialNumber;
    uint16_t error;
    char errorMessage[256];
    
    //pinMode(WIO_LIGHT, INPUT);
    Serial.begin(115200);
    Wire.begin();
    sht4x.begin(Wire);
    error = sht4x.serialNumber(serialNumber);
    while (sgp_probe() != STATUS_OK) {
        Serial.println("SGP failed");
        while (1);
    }
    err = sgp_measure_signals_blocking_read(&scaled_ethanol_signal,
                                            &scaled_h2_signal);
    if (err == STATUS_OK) {
        Serial.println("requesting ram signal! - Please hold...");
    } else {
        Serial.println("error reading signals");
    }
    /* Set absolute humidity to 13.000 g/m^3 adjust calibration*/
    sgp_set_absolute_humidity(13000);
    err = sgp_iaq_init();
    if (error) {
        Serial.print("Error trying to execute serialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Serial Number: ");
        Serial.println(serialNumber);
        Serial.println();
    }
  Serial.println("\nI2C Scanner");
  { 
  byte error, address;
  int nDevices;
  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
/* The i2c_scanner uses the return value of
   the Write.endTransmisstion to see if
   a device did acknowledge to the address.*/
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C devaddr 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  }
}

void loop() {
    //Comment the line below or adjust delay time to change start loop delay. See also stop loop delay. 70ms minimum cycle.
    delay(180);    
    uint16_t error;
    char errorMessage[256];
    float temperature;                               //TempRH
    float humidity;
    error = sht4x.measureHighPrecision(temperature, humidity);
    if (error) {
        Serial.print("Error trying to execute measureHighPrecision(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("T:[C],");
        Serial.print(temperature);
        Serial.print(",H:RH%,");
        Serial.print(humidity);
    }  
    s16 err = 0;                                     //VOC CO2
    u16 tvoc_ppb, co2_eq_ppm;
    err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
    if (err == STATUS_OK) {
         Serial.print(",tVOC:[ppm],"); Serial.print(tvoc_ppb);
         Serial.print(",CO2eq:[ppm],"); Serial.print(co2_eq_ppm); 
    } 
    else   {
        Serial.println("error reading IAQ values\n");
    }
    {
    static unsigned long analogSampleTimepoint = millis();
    if(millis()-analogSampleTimepoint > 40U) //every 40 milliseconds,read the analog value from the ADC
    {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT)
    analogBufferIndex = 0;
    }
    static unsigned long printTimepoint = millis();
    if(millis()-printTimepoint > 800U)
    {
    printTimepoint = millis();
    for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
    analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF/ 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient=1.0+0.02*(temperaturetds-25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge=averageVoltage/compensationCoefficient; //temperature compensation
    tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
    Serial.print("voltage:");
    Serial.print(averageVoltage,2);
    //Serial.print("V ");
    Serial.print("TDS Value:");
    Serial.print(tdsValue,0);
    //Serial.println("ppm");
    }
    }
    int getMedianNum(int bArray[], int iFilterLen)
    {
    int bTab[iFilterLen];
    for (byte i = 0; i<iFilterLen; i++)
    bTab[i] = bArray[i];
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++)
    {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
    if (bTab[i] > bTab[i + 1])
    {
    bTemp = bTab[i];
    bTab[i] = bTab[i + 1];
    bTab[i + 1] = bTemp;
    }
    }
    }
    if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
    else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    return bTemp;
    } 
    Serial.flush();                                 // Prep for baud change wait for send of last transmitted data 
    Serial.begin(9600);                             // switch baud 9600
    while(Serial.available()) Serial.read();       // empty buffer fragments from baud change and read only when buffer clears ready
    int sensorPin = A2; int sensorValue = 0;    // SoilH2O 
    sensorValue = analogRead(sensorPin);        
    Serial.print(",Soil," ); Serial.print(sensorValue); Serial.print(",time," ); Serial.print(millis()); Serial.println();      
    Serial.flush();                                 // Prep for baud change wait for send of last transmitted data 
    Serial.begin(115200);                           // switch baud back to 115200
    while(Serial.available()) Serial.read();        // empty buffer fragments from baud change and read only when buffer clears ready 
//Comment the line below or adjust delay time to change stop loop delay. See also start loop delay. 70ms 
    delay(70);
}
