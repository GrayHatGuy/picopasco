#include <Arduino.h>
#include <SensirionI2CSht4x.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "sensirion_common.h"
#include "sgp30.h"
#include "LIS3DHTR.h"

LIS3DHTR<TwoWire> lis;

/*Reads all I2C and Analog grove sensors from K1100 WIO SEEED kit
Temperature Humidity, Volatile Organic Chemicals, Light, Inertia Measurement Unit(IMU), Soil Moisture, and Sound. 
The IMU, sound, and light sensors are on board of WIO Terminal. The TempRH and VOC are external sensors. 
The LoRA-E5 and GroveAI may be connected but are not functional with this code. */

#include <Wire.h>

SensirionI2CSht4x sht4x;

void setup() 
{
    s16 err;
    u32 ah = 0;
    u16 scaled_ethanol_signal, scaled_h2_signal;
    pinMode(WIO_LIGHT, INPUT);
    Serial.begin(115200); 
    lis.begin(Wire1);
 
    if (!lis) {
      Serial.println("ERROR");
      while(1);
    }
  lis.setOutputDataRate(LIS3DHTR_DATARATE_25HZ); //Data output rate
  lis.setFullScaleRange(LIS3DHTR_RANGE_2G); //Scale range set to 2g
    while (!Serial) {
        delay(100);
    }
    Wire.begin();

    uint16_t error;
    char errorMessage[256];

    sht4x.begin(Wire);

    uint32_t serialNumber;
    error = sht4x.serialNumber(serialNumber);
    /*  Init module,Reset all baseline,The initialization takes up to around 15 seconds, during which
        all APIs measuring IAQ(Indoor air quality ) output will not change.Default value is 400(ppm) for co2,0(ppb) for tvoc*/    
    while (sgp_probe() != STATUS_OK) {
        Serial.println("SGP failed");
        while (1);
    }
    /*Read H2 and Ethanol signal in the way of blocking*/
    err = sgp_measure_signals_blocking_read(&scaled_ethanol_signal,
                                            &scaled_h2_signal);
    if (err == STATUS_OK) {
        Serial.println("requesting ram signal! - Please hold...");
    } else {
        Serial.println("error reading signals");
    }
    // Set absolute humidity to 13.000 g/m^3
    //It's just a test value
    //Need to calibrate humidity for application
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
    pinMode(WIO_MIC, INPUT);
    Serial.flush();
    Serial.begin(9600);
    while (!Serial);   
      
    Serial.println("\nI2C Scanner");
{
  byte error, address;
  int nDevices;
  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
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
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  //delay(1000);           // wait 1 seconds for next scan
  Serial.flush();                                 // Prep for baud change wait for send of last transmitted data 
  Serial.begin(115200);                           // switch baud back to 115200
  while(Serial.available()) Serial.read();        // empty buffer fragments from baud change and read only when buffer clears ready 
}
    
}

void loop() {
    uint16_t error;
    char errorMessage[256];
//Comment the line below or adjust delay time to change start loop delay. See also stop loop delay. 70ms minimum cycle.
    delay(180);
    
    float temperature;                               //TempRH get
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
    s16 err = 0;                                     //VOC get
    u16 tvoc_ppb, co2_eq_ppm;
    err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
    if (err == STATUS_OK) {
         Serial.print(",tVOC:[ppm],"); Serial.print(tvoc_ppb);
         Serial.print(",CO2eq:[ppm],"); Serial.print(co2_eq_ppm); 
    } 
    else   {
        Serial.println("error 0xDEADCODE reading IAQ values\n");
    } 
    int light = analogRead(WIO_LIGHT);               //light get
    Serial.print(",Light:[cts],"); Serial.print(light); 
    float x_values, y_values, z_values;              //IMU get
    x_values = lis.getAccelerationX();              
    y_values = lis.getAccelerationY(); 
    z_values = lis.getAccelerationZ(); 
    Serial.print(",xyz,"); 
    Serial.print(x_values); Serial.print(","); 
    Serial.print(y_values); Serial.print(","); 
    Serial.print(z_values); 
    int val = analogRead(WIO_MIC);                  //Microphone check one two
    Serial.print(",Sound:[cts],"); Serial.print(val);
    Serial.flush();                                 // Prep for baud change wait for send of last transmitted data 
    Serial.begin(9600);                             // switch baud 9600
    while(Serial.available()) Serial.read(); {      // empty buffer fragments from baud change and read only when buffer clears ready
        int sensorPin = A2; int sensorValue = 0;    // SoilH2O get
        sensorValue = analogRead(sensorPin);        
        Serial.print(",Soil," ); Serial.print(sensorValue); Serial.print(",time," ); Serial.print(millis()); Serial.println();
        }       
    Serial.flush();                                 // Prep for baud change wait for send of last transmitted data 
    Serial.begin(115200);                           // switch baud back to 115200
    while(Serial.available()) Serial.read();        // empty buffer fragments from baud change and read only when buffer clears ready 
//Comment the line below or adjust delay time to change stop loop delay. See also start loop delay. 70ms 
    delay(70);
}