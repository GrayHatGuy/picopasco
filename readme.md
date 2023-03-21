## picopasco

  #### *- to feed oneself a little bit*

 ## description
sensor monitor and control hardware/firmware for a personal economical and scalable grow system using a raspberrypi pico and standard off the shelf sensors and components. 

#### **present state** 
_includes nutrient mixing capabilities and monitor of the following sensors._
 *  air temp
 *  water temp
 *  humidity
 *  volatile organic compounds
 *  carbon dioxide
 *  soil moisture - [see irrigation](https://github.com/GrayHatGuy/picopasco/blob/main/readme.md#irrigation)
 *  tds
 *  ec
 *  pH 
  
   **_see [vids](https://github.com/GrayHatGuy/picopasco/blob/main/readme.md#vids) and [picts](https://github.com/GrayHatGuy/picopasco/blob/main/readme.md#picts)     for more details._** 
  
## wiring diagram:
 ![image](https://github.com/GrayHatGuy/picopasco/blob/25a02a05b4e160a8b521379950dd38b4c0e63f31/repo_full%20picopasco%20wire.png?raw=true)
## features
- ### nutrient mixing 
four liquid reservoirs using relay triggered solenoid valves for mixing water with up to three nutrients.  User inputs nutrient tank capacity, target EC/TDS, nutrient mix ratios, and mixt interval then the system automatically mixes and fills the tank. Mix may also be triggered externally for point of use.
- ### water quality
The tds/ec sensor can be used as a continuous monitor to ensure the water quality is within the target range and compensate for drift by adding water or nutrients.  
- ### irrigation
moisture sensor monitors the grow media and can trigger irrigation. The nutrient mix can then be used to irrigate multiple grow mediums like hydroponic, aquaponic, or terrestial systems. See [picopasco-water](https://github.com/GrayHatGuy/picopasco-water) for controlling irrigation based on moisture sensor feedback.
- ### air quality*
includes temperature, humidity, co2, and volatile organic chemical (voc) sensors.

  _*-present state only include closed loop feedback for mixing additional relays may be added to control irrigation, air quality, and/or lighting_

- ### power
  * #### specifications
    * quiescent (sensor only) - 0.25 watt 
    * maximum (all valves open with sensors scanning) - 4.00 watts* _*operation limited to 1 pump at a time with relay triggering 1.75 watts_
  * #### requirements 
    * ##### supply
      * **on grid:** ac to 5 volts transformer 5 watts
      * **off grid** solar battery bank approximate 15 watts >=15000 mah
## libs:
 *  [sgp30 gas sensor](https://github.com/Seeed-Studio/SGP30_Gas_Sensor)
 *  [sht4x temp/rh](https://www.arduinolibraries.info/libraries/sensirion-i2-c-sht4x)
## parts: 
  ### minimum _(by firmware design)_
   *  $6.00  [rp2040 pico  - (pico w optional)](https://www.raspberrypi.com/products/raspberry-pi-pico/)
   *  $5.50  [grove sht4x temp/rh](https://www.seeedstudio.com/Grove-Temp-Humi-Sensor-SHT40-p-5384.html)
   *  $17.50 [grove sgp30 voc/co2](https://www.seeedstudio.com/Grove-VOC-and-eCO2-Gas-Sensor-for-Arduino-SGP30.html)
   *  $14.00 [water quality tds/ec ](https://www.amazon.com/dp/B08DGLY3J2)
   *  $34.99 [pH sensor](https://www.amazon.com/GAOHOU-PH0-14-Detect-Electrode-Arduino/dp/B0799BXMVJ)
   *  $7.99  [relays 5 volt spst - 4x bank](https://www.amazon.com/dp/B098DWS168)
   *  $12.98 [5 volt pumps 4x - (alt solenoid option)](https://www.amazon.com/Gikfun-2-5V-6V-Submersible-Silicone-EK1374/dp/B0957BS936/)
  ### $98.96 Total cost of materials
  ### optional 
  _(intended by firmware design but not required)_
  -  [grove soil moisture](https://www.seeedstudio.com/Grove-Moisture-Sensor.html?)
  -  [rp2040 grove breakout](https://www.digikey.com/en/products/detail/seeed-technology-co.,-ltd/103100142/13688265)

  -  [5 volt solenoid valves 4x](https://www.amazon.com/dp/B07WR9CSNQ)
  -  [sx1262 lora(wan) raspberry pi pico hat](https://www.waveshare.com/pico-lora-sx1262-868m.htm)
  ### future upgrade: 
  -  [rfm95 lora p2p]()
  -  [lorae5 lorawan)]()
  -  wifi/bt/mqtt
## status
 * ✅ ~sensor firmware~
 * ✅ ~bench verify hardware~
 * ✅ ~wiring schematic and pinout~ 
 * ✅ ~relay trigger firmware (mixing)~
 * ✅ ~integrate sensor and mixing code (See V3.0+)~ 
 * ✅ ~platformio and arduino compatible~
 * ✅ ~pH sensor~
 * [ ] pid control mix to target setpoint pH/tds/ec - [_firmware in process_](https://github.com/GrayHatGuy/picopasco/tree/2bdc89169fcf1709cf48569881d38e65c02e824f/dev/picopasco-main-V4.3597447222071-alpha-hartree-pid)
 * [ ] sx1262 lora/wan - [_bench testing_]() gpio20 conflict with relay changed to 15,16,17,18 v4.0
 * [ ] wifi/bt/mqtt
## plan 
 *  ### controls
    *  lighting timers
    *  hvac triggers pid control
 *  ### connectivity   
    *  wifi 
    *  +lora(wan)
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
*  ### vids
    *  example of runtime [firmware output](https://youtu.be/fia2N3TB2j8)
    *  testing minimum [response time of relays](https://youtu.be/YmTmU25x0V8). 3 ms is the mininum trigger dwell otherwise relays fail to actuate mechanically.
    *  [max power consumed](https://youtube.com/shorts/H6yiRs7PBLs?feature=share) 350 ma @ 5 volts [1.5 watt] during dry pump cycling.
    *  [initial power on test](https://youtu.be/p2OLT5P7gyc) failed to stop pumping due to a partial vacuum build up induced syphoning effect. 
        - solved with shorter tube length and shorter elevation head.
        - optional [solenoid shut off valve](https://github.com/GrayHatGuy/picopasco/blob/main/readme.md#optional) in ```<OR>``` with pump SPST relay NO trigger will provide a failsafe engineering control.
* ### [picts](https://github.com/GrayHatGuy/picopasco/tree/ca9e4c73baeb89e9fcb9bd323aaa8c2fb0ac0a06/picts)
 
##### _If you are interested in contributing or participating in this project contact_ GrayHatGuy@GrayHatGuy.com
