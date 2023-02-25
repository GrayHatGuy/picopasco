## picopasco

*- to feed oneself with small portions*

sensor monitor and control hardware/firmware for a personal economical and scalable grow system using a RaspberryPi pico RP2040 and standard off the shelf sensors and components. 

__present state includes nutrient mixing capabilities and the following sensors.___
 *  air temp
 *  water temp
 *  humidity
 *  volatile organic compounds
 *  carbon dioxide
 *  soil moisture
 *  tds
 *  ec
 ![image](https://github.com/GrayHatGuy/picopasco/blob/684087f6126cab00c318301aa4c4ce1e90a30841/repo_full%20picopasco%20wire.png?raw=true)
## Features
- ### nutrient mixing 
four liquid reservoirs using relay triggered solenoid valves for mixing water with up to three nutrients.  User inputs nutrient tank capacity, target EC/TDS, nutrient mix ratios, and mixt interval then the system automatically mixes and fills the tank. Mix may also be triggered externally for point of use.
- ### water quality
The tds/ec sensor can be used as a continuous monitor to ensure the water quality is within the target range and compensate for drift by adding water or nutrients.  
- ### irrigation
Moisture sensor monitors the grow media and can trigger a watering. The nutrient mix can then be used to irrigate multiple grow mediums like hydroponic, aquaponic, or terrestial systems. _requires additional relays and irrigation pump or valve_ 
- ### air quality
includes temperature, humidity, C02, and volatile organic chemical (VOC) sensors. _requires additional relays_

- ### power
  * #### specifications
    * quiescent (sensor only) - 0.25 W 
    * maximum (All valves open with sensors) - 4.00 W
  * #### requirements 
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
 *  yt demo
 *  controls
    *  ✅ ~trigger (irrigation)~
    *  ✅ ~relay firmware~
    *  ✅ ~integrate code into main.cpp~ 
    *  low cost pH sensor
    *  pid relay mix control tds/ec and pH
    *  lighting timers
    *  hvac triggers pid control
 *  connectivity   
    *  wifi 
    *  lora(wan)
    *  mqtt
 *  ui 
    * local wifi/bt 
    * remote mobile app
##### _If you are interested in contributing or participating in this project contact_ GrayHatGuy@GrayHatGuy.com
