## picopasco

*- to feed oneself with small portions*

sensor monitor and control hardware/firmware for a personal economical and scalable grow system using a RaspberryPi pico RP2040 and standard off the shelf sensors and components.  Present state includes nutrient mixing capabilities and the following sensors.
 *  air temp
 *  water temp
 *  humidity
 *  VOC
 *  CO2
 *  Soil H2O
 *  TDS
 *  EC
 ![image](https://github.com/GrayHatGuy/picoponic/blob/0f7c91f61f793c428a4101a5d96488dfcb26ee3a/repo_full%20picoponicwire.png?raw=true)
## Features
- ### nutrient mixing 
four liquid reservoirs using triggered solenoid valves for mixing water with up to three nutrients.  User inputs nutrient tank capacity target EC/TDS and nutrient mix ratios then the system automatically mixes and fills the tank.  
- ### water quality
The TDS/EC sensor can be used as a continuous monitor to ensure the water quality is within the target range and compensate for drift by adding water or nutrients.  
- ### Irrigation
Moisture sensor monitors the grow media and can trigger a watering. The nutrient mix can then be used to irrigate multiple grow mediums like hydroponic, aquaponic, or terrestial systems.  Presently, additional valves need to be added for automatic watering.
- ### Air quality
Includes temperature, humidity, C02, and volatile organic chemical (VOC) sensors. _These sensors can be used to automate HVAC triggers with additional relays._
- ### power
  * #### specifications
    * quiescent (sensor only) - 0.25 W 
    * maximum (All valves open with sensors) - 4.00 W
  * #### Requirements 
    * ##### Supply
      * **on grid:** AC to 5V transformer 5W
      * **off grid** Solar battery bank approximate 15W/15000mAh
## libs:
 *  [SGP30 Gas Sensor](https://github.com/Seeed-Studio/SGP30_Gas_Sensor)
 *  [SHT4x TempRH](https://www.arduinolibraries.info/libraries/sensirion-i2-c-sht4x)
## parts: 
 *  [Grove SHT4x Temp/RH](https://www.seeedstudio.com/Grove-Temp-Humi-Sensor-SHT40-p-5384.html?queryID=79f54ab791e4345a5bd143b2f1674b74&objectID=5384&indexName=bazaar_retailer_products)
 *  [Grove SGP30 VOC/CO2](https://www.seeedstudio.com/Grove-VOC-and-eCO2-Gas-Sensor-for-Arduino-SGP30.html?queryID=f5af88e62b89603f700a72fc7083e746&objectID=127&indexName=bazaar_retailer_products)
 *  [Grove Soil Moisture](https://www.seeedstudio.com/Grove-Moisture-Sensor.html?queryID=8f8a40002a96e9bcb9aad1275f9a6cad&objectID=1678&indexName=bazaar_retailer_products)
 *  [Water quality TDS/EC](https://www.amazon.com/dp/B08DGLY3J2)
 *  [Relays 5V SPST - 4X bank](https://www.amazon.com/dp/B098DWS168)
 *  [5V solenoid valves 4X](https://www.amazon.com/dp/B07WR9CSNQ)
 *  [RP2040 grove breakout](https://www.digikey.com/en/products/detail/seeed-technology-co.,-ltd/103100142/13688265)
 *  [RP2040 pico](https://www.raspberrypi.com/products/raspberry-pi-pico/)
## present state
 * ✅ ~sensor firmware~
 * ✅ ~bench verify hardware~
 * ✅ ~wiring schematic and pinout~ 
## future state
 *  Controls
    *  PID relay control mixing(TDS) firmware
    *  timer (lighting)
    *  trigger (irrigation) 
    *  trigger HVAC(air quality) firmware.
 *  Connectivity   
    *  WiFi 
    *  LoRa
    *  MQTT
 *  GUI 
    * local
    * Mobile
### _If you are interested in contributing or participating in this project contact_ GrayHatGuy@GrayHatGuy.com
