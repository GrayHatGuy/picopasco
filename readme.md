## picopasco

  #### *- to feed oneself with small portions*

 ## description
sensor monitor and control hardware/firmware for a personal economical and scalable grow system using a raspberrypi pico and standard off the shelf sensors and components. 

#### **present state** _includes nutrient mixing capabilities and monitor of the following sensors._
 *  air temp
 *  water temp
 *  humidity
 *  volatile organic compounds
 *  carbon dioxide
 *  soil moisture
 *  tds
 *  ec
 *  pH 
  
   **_see [vids](https://github.com/GrayHatGuy/picopasco/blob/main/readme.md#vids) and [picts](https://github.com/GrayHatGuy/picopasco/blob/main/readme.md#picts)     for more details._**
  
## wiring diagram:
 ![image](https://github.com/GrayHatGuy/picopasco/blob/684087f6126cab00c318301aa4c4ce1e90a30841/repo_full%20picopasco%20wire.png?raw=true)
## features
- ### nutrient mixing 
four liquid reservoirs using relay triggered solenoid valves for mixing water with up to three nutrients.  User inputs nutrient tank capacity, target EC/TDS, nutrient mix ratios, and mixt interval then the system automatically mixes and fills the tank. Mix may also be triggered externally for point of use.
- ### water quality
The tds/ec sensor can be used as a continuous monitor to ensure the water quality is within the target range and compensate for drift by adding water or nutrients.  
- ### irrigation*
moisture sensor monitors the grow media and can trigger irrigation. The nutrient mix can then be used to irrigate multiple grow mediums like hydroponic, aquaponic, or terrestial systems. 
- ### air quality*
includes temperature, humidity, co2, and volatile organic chemical (voc) sensors.

  _*-present state only include closed loop feedback for mixing additional relays may be added to control irrigation, air quality, and/or lighting_

- ### power
  * #### specifications
    * quiescent (sensor only) - 0.25 watt 
    * maximum (All valves open with sensors) - 4.00 watts
  * #### requirements 
    * ##### supply
      * **on grid:** ac to 5 volts transformer 5 watts
      * **off grid** solar battery bank approximate 15 watts >=15000 mah
## libs:
 *  [sgp30 gas sensor](https://github.com/Seeed-Studio/SGP30_Gas_Sensor)
 *  [sht4x temp/rh](https://www.arduinolibraries.info/libraries/sensirion-i2-c-sht4x)
## parts: 
  ### minimum required
   *  [rp2040 pico  - (pico w optional)](https://www.raspberrypi.com/products/raspberry-pi-pico/)
   *  [grove sht4x temp/rh](https://www.seeedstudio.com/Grove-Temp-Humi-Sensor-SHT40-p-5384.html?queryID=79f54ab791e4345a5bd143b2f1674b74&objectID=5384&indexName=bazaar_retailer_products)
   *  [grove sgp30 voc/co2](https://www.seeedstudio.com/Grove-VOC-and-eCO2-Gas-Sensor-for-Arduino-SGP30.html?queryID=f5af88e62b89603f700a72fc7083e746&objectID=127&indexName=bazaar_retailer_products)
  queryID=8f8a40002a96e9bcb9aad1275f9a6cad&objectID=1678&indexName=bazaar_retailer_products)
   *  [water quality tds/ec ](https://www.amazon.com/dp/B08DGLY3J2)
   *  [pH sensor](https://www.amazon.com/GAOHOU-PH0-14-Detect-Electrode-Arduino/dp/B0799BXMVJ)
   *  [relays 5 volt spdt - 4x bank](https://www.amazon.com/dp/B098DWS168)
  ### optional items:
   *  [5 volt pumps 4xX](https://www.amazon.com/Gikfun-2-5V-6V-Submersible-Silicone-EK1374/dp/B0957BS936/)
   *  [grove soil moisture -_(optional)](https://www.seeedstudio.com/Grove-Moisture-Sensor.html?
   *  [rp2040 grove breakout - (optional))](https://www.digikey.com/en/products/detail/seeed-technology-co.,-ltd/103100142/13688265)

   *  [5 volt solenoid valves 4x - (optional)](https://www.amazon.com/dp/B07WR9CSNQ)
   *  [SX1262 LoRaWAN raspberry pi pico hat waveshare - (optional)](https://www.waveshare.com/pico-lora-sx1262-868m.htm)
   *  [alt rfm95 lora p2p  - (optional)]()
   *  [alt lorae5 lorawan  - (optional)]()
## present state
 * ✅ ~sensor firmware~
 * ✅ ~bench verify hardware~
 * ✅ ~wiring schematic and pinout~ 
 * ✅ ~relay trigger firmware (mixing)~
 * ✅ ~integrate sensor and mixing code (See V3.0+)~ 
 * ✅ ~platformio and arduino compatible~
 * ✅ ~pH sensor~
## future state
 * ### vids
    *  example of runtime [firmware output](https://www.youtube.com/watch?v=9E_uXJ-so4A&feature=youtu.be)
    *  testing minimum [response time of relays](https://youtu.be/YmTmU25x0V8). Found 3 ms is the mininum otherwise they fail to mechanically actuate.
    *  [max power consumed](https://youtube.com/shorts/H6yiRs7PBLs?feature=share) 350 ma @ 5 volts [1.5 watt] during dry pump cycling.
    *  [initial power on test](https://youtu.be/p2OLT5P7gyc) failed to stop pumping due to a partial vacuum build up induced syphoning effect. 
 *  ### controls
    *  +pid relay mix control tds/ec and pH - _dev code in process_
    *  lighting timers
    *  hvac triggers pid control
 *  ### connectivity   
    *  wifi 
    *  +lora(wan) - _boot check pass - Waveshare SX1262 header piggyback (changed GPIO20 conflict with relay trigger)_
    *  mqtt
 *  ### ui 
    * local wifi/bt  
    * remote mobile app
 *  ### portability  
    *  alternate mcu
        -  xiao
        -  esp32
        -  wioterm
    * platformio build/flash verify
 *  ### picts

    - ***current state*** sensors 7x and relay/pumps 4x (dry fit) 
   ![image](https://github.com/GrayHatGuy/picopasco/blob/5828d865b027139c95d328c9270929ebe838d00b/picts/present_state_relayX4_sensorsX7.jpeg)
    - ***future state*** upgrade 4+ relays and additional sensors. add pid control to mixing.
    ![image](https://github.com/GrayHatGuy/picopasco/blob/a89c131b6d9dfd96e4c88eb03be09111bf86e8a9/picts/future_state_upgrades_relay8X_sensors7X_Heat_Humidity_relayunused2X.jpeg)

 
##### _If you are interested in contributing or participating in this project contact_ GrayHatGuy@GrayHatGuy.com
