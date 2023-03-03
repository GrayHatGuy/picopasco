## picopasco

 *- to feed oneself with small portions*

sensor monitor and control hardware/firmware for a personal economical and scalable grow system using a RaspberryPi pico RP2040 and standard off the shelf sensors and components. 

   **_present state includes nutrient mixing capabilities and the following sensors._**
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
includes temperature, humidity, C02, and volatile organic chemical (VOC) sensors.

  _*-present state only include closed loop feedback for mixing additional relays may be added to control irrigation, air quality, and/or lighting_

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
 *  [5V pumps 4X](https://www.amazon.com/Gikfun-2-5V-6V-Submersible-Silicone-EK1374/dp/B0957BS936/)
 *  [RP2040 grove breakout](https://www.digikey.com/en/products/detail/seeed-technology-co.,-ltd/103100142/13688265)
 *  [RP2040 pico](https://www.raspberrypi.com/products/raspberry-pi-pico/)
 *  [pH sensor](https://www.amazon.com/GAOHOU-PH0-14-Detect-Electrode-Arduino/dp/B0799BXMVJ)
 *  [alt pumps - 5V solenoid valves 4X](https://www.amazon.com/dp/B07WR9CSNQ)
 *  [SX1262 LoRaWAN RPi pico HAT waveshare](https://www.waveshare.com/pico-lora-sx1262-868m.htm)
 *  [alt RFM95 LoRA]()
 *  [alt LoRa E5]()
 *  [pH](https://www.amazon.com/dp/B0799BXMVJ)
## present state
 * ✅ ~sensor firmware~
 * ✅ ~bench verify hardware~
 * ✅ ~wiring schematic and pinout~ 
 * ✅ ~relay trigger firmware (mixing)~
 * ✅ ~integrate sensor and mixing code (See V3.0+)~ 
 * ✅ ~platformio and arduino compatible~
## future state
 * ### vids
    *  Example of runtime [firmware output](https://www.youtube.com/watch?v=9E_uXJ-so4A&feature=youtu.be)
    *  Testing minimum [response time of relays](https://youtu.be/YmTmU25x0V8). Found 3 ms is the mininum otherwise they fail to mechanically actuate.
    *  [Max power consumed](https://youtube.com/shorts/H6yiRs7PBLs?feature=share) 350 mA @ 5V [1.5W] during dry pump cycling.
    *  [Initial power on test](https://youtu.be/p2OLT5P7gyc) failed to stop pumping due to a partial vacuum build up induced syphoning effect. Need to consider pump          water level and tubing length to mitigate issue.
 *  ### controls
    *  +low cost pH sensor - _$30 passes smoke test_
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
    *  alternate MCUs
        -  xiao
        -  esp32
        -  wioterm
    * platformio build/flash verify
 *  ### picts

    - ***current state*** sensors 7x and relay/pumps 4x (dry fit) 
   ![image](https://github.com/GrayHatGuy/picopasco/blob/5828d865b027139c95d328c9270929ebe838d00b/picts/present_state_relayX4_sensorsX7.jpeg)
    - ***future state*** upgrade 4+ relays and additional sensors. add triggers for heating or humidifying. add pH and pid control. 
    ![image](https://github.com/GrayHatGuy/picopasco/blob/a89c131b6d9dfd96e4c88eb03be09111bf86e8a9/picts/future_state_upgrades_relay8X_sensors7X_Heat_Humidity_relayunused2X.jpeg)

 
##### _If you are interested in contributing or participating in this project contact_ GrayHatGuy@GrayHatGuy.com
