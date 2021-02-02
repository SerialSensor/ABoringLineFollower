<img align="left" width="50" height="100" src="https://github.com/SerialSensor/ProjMedia/blob/master/appIcon.png" alt="WTF">

# A boring line follower

What you'll get:

[![Preview](http://img.youtube.com/vi/yorxUHchpSI/0.jpg)](https://www.youtube.com/watch?v=yorxUHchpSI "A boring line follower")
  
## How to get started
- [Get the App](#get-the-app)
- [How to build the lazy bot](#how-to-build-the-lazy-bot)
  - [Bill of Materials](#bill-of-materials)
  - [Parameters to start with](#parameters-to-start-with)
- [How to build the fast bot](#how-to-build-the-fast-bot)
  - [Bill of Materials](#bill-of-materials-1)
  - [Parameters to start with](#parameters-to-start-with-1)
- [Phone mounting](#mounting)
- [Troubleshooting](#troubleshooting)

## Get the App
Serial Sensor is free, you can download it from the [play store](https://play.google.com/store/apps/details?id=com.karl.serialsensor).
For using the camera based sensors, you need to run the calibration, therefore the app will tell you what you need to do.
When you're done with the calibration, select the line- and parameter sensor. Additionally you need to choose either USB or Bluetooth as an connection (see screenshots below).
| | | 
|:-------------------------:|:-------------------------:|
|<img width="1604" alt="sel_sens" src="https://raw.githubusercontent.com/SerialSensor/ProjMedia/master/Line/selected_sensors.png"> | <img width="1604" alt="sel_out" src="https://raw.githubusercontent.com/SerialSensor/ProjMedia/master/Line/selected_output.png"> |
## How to build the lazy bot
Here's how you build the lazy bot. "Lazy" since almost everything comes with the 2WD chassis. Drawback: it's slow and you cannot do other fun stuff with it.
| | | 
|:-------------------------:|:-------------------------:|
|<img width="1604" alt="lazy 0" src="https://raw.githubusercontent.com/SerialSensor/ProjMedia/master/Line/lazy_0.jpg"> |  <img width="1604" alt="lazy 1" src="https://raw.githubusercontent.com/SerialSensor/ProjMedia/master/Line/lazy_1.jpg">|

### Bill of Materials
| What          | Example Link|
|:-------------:|:-------------:|
| Arduino Nano (or clone) | -|
| Motor driver L298N|[ebay](https://www.ebay.com/itm/L298N-DC-Stepper-Motor-Driver-Module-Dual-H-Bridge-Control-Board-for-Arduino-US/392861171296?hash=item5b7859be60:g:l1QAAOSwCMxe21dq)  |
|2WD chassis |[amazon](https://www.amazon.com/Smart-Chassis-Motors-Encoder-Battery/dp/B01LXY7CM3/ref=sr_1_15?dchild=1&keywords=2WD+chassis&qid=1613756390&sr=8-15)|
|Breadboard| [ebay](https://www.ebay.com/itm/5pcs-SYB-170-Mini-Breadboard-Colorful-Breadboard-Prototype-Small-Plates/352623796461?hash=item521a0420ed:g:SRwAAOSwpZpclGfA)|
|Bluetooth module HC-06 (or usb cable, see fast bot)|[ebay](https://www.ebay.com/itm/1PCS-Slave-HC-06-Wireless-Bluetooth-Transeiver-RF-Master-Module-for-Arduino/312551823284?hash=item48c58a57b4:g:wgsAAOSw5m9fQ6rx)|
|Female-Male jumper wires| [ebay](https://www.ebay.com/itm/Wire-Cable-Male-to-Female-Ribbon-Dupont-Jumper/224338334534?hash=item343b9b5346:g:imIAAOSw~xRgGrT0)|
|Magnetic car phone mount| [aliexpress](https://www.aliexpress.com/item/4001141818390.html?spm=a2g0o.productlist.0.0.3332259ajzEPxO&algo_pvid=eb210c7e-4940-40af-9eb8-64d94017fdbb&algo_expid=eb210c7e-4940-40af-9eb8-64d94017fdbb-6&btsid=2100bdca16137527752533041e69f6&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_) or [print it](https://www.thingiverse.com/thing:4763685) |
| electrical tape for the line |-|

### Parameters to start with
To parameterize your application, you can use the parameter sensor. Following parameters are available and need to be set within the Serial Sensor app. For further clarification, check the arduino code comments.
|Byte |  Description |Quantization| Range| Value to start with|
|:-------------:|:-------------:|:-------------:|:-------------:|:-------------:|
| 1 | PWM offset left or right motor|1| 0-255| 0|
| 2 | PWM offset left or right motor|1| 0-255| 0|
| 3 | P-Value of PID controller|0.1| 0-255| 35|
| 4 | D-Value of PID controller|0.05| 0-255| 30|
| 5 | Max allowed pwm speed in percent |0.01| 0-100| 100|
| 6 | Detected line distance for which the last detected point has a weight of 0.5  |0.005| 0-255| 0|
| 7 | Detected line distance for which full speed is applied  |0.005| 0-255| 95|
| 8 | Minimum PWM which is always applied in percent|0.01| 0-100| 35|


## How to build the fast bot
Here's how you build the fast bot. "Fast" since you can buy N20 motors which are having a lot higher RPM than the ones coming with a 2WD chassis. 
| | | |
|:-------------------------:|:-------------------------:|:-------------------------:|
|<img width="1604" alt="fast 1" src="https://raw.githubusercontent.com/SerialSensor/ProjMedia/master/Line/fast_1.jpg"> |  <img width="1604" alt="fast 2" src="https://raw.githubusercontent.com/SerialSensor/ProjMedia/master/Line/fast_2.jpg">|<img width="1604" alt="fast 0" src="https://raw.githubusercontent.com/SerialSensor/ProjMedia/master/Line/fast_0.jpg">|

### Bill of Materials
| What          | Example Link|
|:-------------:|:-------------:|
| Arduino Nano (or clone) | -|
| PCB 120x80mm  | [ebay](https://www.ebay.com/itm/Double-Side-Prototype-PCB-Tinned-Universal-Bread-board-8x12-cm-80x120-mm-FR4-DIY/271817506142?hash=item3f4995fd5e:g:0a0AAOSwstxVEm5v) |
| Ball Caster / Wheel caster   | [polulu](https://www.pololu.com/product/2692)|
| N20 DC motor 6V (300 - 500RPM) | [ebay](https://www.ebay.com/itm/GA12-N20-DC-3V-6V-12V-Micro-Electric-Gear-Motor-Speed-Reduction-Metal-Gearbox/402608204445?hash=item5dbd51aa9d:g:cFUAAOSw7yxf2HUC)     |
| Small battery 7.4V  | [Joom](https://www.joom.com/en/products/5cb9bb918b2c370101ba066b)     |
| JST plugs | [ebay](https://www.ebay.com/itm/10Pairs-Male-Female-10cm-JST-Connector-Plug-Cable-Line-for-RC-BEC-Lipo-Battery-s/313353616464?hash=item48f554bc50:g:ayIAAOSwB-1Y3Fwa)     |
| Female headers | [ebay](https://www.ebay.com/itm/16P-Female-Header-Pins-2-54mm-Pitch-PCB-Socket-Single-Row-Connector-Straight/223503325170?hash=item3409d617f2:g:EgsAAOSwD2NczByO)     |
| Motor brackets | [robu.in](https://robu.in/product/mounting-bracket-n20-micro-gear-motors/) |
| USB cable (you shouldn't use bluetooth here due to high latency) | [ebay](https://www.ebay.com/itm/USB-Type-c-to-Mini-USB-Cable-USB-C-Male-to-Mini-B-Male-Adapter-ConvertODUS/114238371143?hash=item1a99233547:g:YekAAOSwpUVcd5Eu) |
| Motor driver 8835| [pololu](https://www.pololu.com/product/2135) |
| Screw terminal block| [ebay](https://www.ebay.com/itm/10pcs-2-Poles-KF128-2-54mm-PCB-Universal-Screw-Terminal-Block-Gut-xm/184399768007?hash=item2aef1529c7:g:LoAAAOSw5E5avvGm) |
|Magnetic car phone mount| [aliexpress](https://www.aliexpress.com/item/4001141818390.html?spm=a2g0o.productlist.0.0.3332259ajzEPxO&algo_pvid=eb210c7e-4940-40af-9eb8-64d94017fdbb&algo_expid=eb210c7e-4940-40af-9eb8-64d94017fdbb-6&btsid=2100bdca16137527752533041e69f6&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_) or [print it](https://www.thingiverse.com/thing:4763685) |
| electrical tape for the line |-|

### Parameters to start with
To parameterize your application, you can use the parameter sensor. Following parameters are available and need to be set within the Serial Sensor app. For further clarification, check the arduino code comments.
|Byte |  Description |Quantization| Range| Value to start with|
|:-------------:|:-------------:|:-------------:|:-------------:|:-------------:|
| 1 | PWM offset left or right motor|1| 0-255| 0|
| 2 | PWM offset left or right motor|1| 0-255| 0|
| 3 | P-Value of PID controller|0.1| 0-255| 30|
| 4 | D-Value of PID controller|0.05| 0-255| 15|
| 5 | Max allowed pwm speed in percent |0.01| 0-100| 100|
| 6 | Detected line distance for which the last detected point has a weight of 0.5  |0.005| 0-255| 15|
| 7 | Detected line distance for which full speed is applied  |0.005| 0-255| 95|
| 8 | Minimum PWM which is always applied in percent|0.01| 0-100| 8|

## Phone mounting
* Make always sure to adapt the mounting height of your phone within the calibration menu, if you want the estimated distances to make any sense. 
* The tilt angle of the phone is automatically accounted for, nothing to do here.

## Troubleshooting
When using a camera based sensor, always make sure that there is nothing of the bot visible to the camera!
For other problems, check the app internal help.
