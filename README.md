Qwiic PC Fan Controller
========================================

![Qwiic PC Fan Controller](https://cdn.sparkfun.com//assets/parts/1/8/0/3/2/18570-Qwiic_PC_Fan_Controller-01.jpg)

[*Qwiic PC Fan Controller (SPX-18570)*](https://www.sparkfun.com/products/18570)

Whether for active cooling or ventilation, a tiny fan can make a big difference. And by that logic a big fan can make an even bigger difference! Luckily, 4-Wire PC cooling fans come in all shapes and sizes, so there's definitely one out there that's perfect for your project. The only downside is that they can be a pain to drive if you don't need them to be 100% on the throttle all the time. To remedy this, we've modified our Qwiic Blower Fan board to drive any 3- or 4-wire PC Fan with any voltage from 5 to 20VDC!

The Qwiic PC Fan Controller allows you easily control almost any PC fan over the Qwiic bus using the on board ATtiny microcontroller and control firmware. The control firmware monitors the tachometer output of the fan in order to implement PI Control over the fan speed, allowing you to set your desired speed in real units. It's also possible to disable the PI control loop and set the speed as a proportion of the maximum. The accompanying Arduino library includes example code for controlling the fan, tweaking settings, and even setting the fan speed based on an attached Qwiic Temperature Sensor and a lookup table. The on board trimpots allow you to very quickly tune the PI loop parameters to your application, but they can also be overridden by the Qwiic bus for remote configuration.

Heads Up! Without any external power supply connected, the Qwiic PC Fan Controller will attempt to drive the connected fan from the Qwiic bus. Even a very small 5V fan operates at the upper limit of the Qwiic bus' power delivery capability. If there are few other devices on the bus and the fan is not allowed to rapidly change throttle, it is possible to power a small fan directly from the bus. In most applications, however, it will be necessary to provide the PC Fan Controller with a dedicated fan power supply.

Repository Contents
-------------------

* **/Firmware** - Control firmware for the ATtiny
* **/Hardware** - Eagle design files (.brd, .sch)
* **/Production** - Panelized hardware files for production

License Information
-------------------

This product is _**open source**_! 

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please contact technical support on our [SparkFun forums](https://forum.sparkfun.com/viewforum.php?f=152).

Distributed as-is; no warranty is given.

- Your friends at SparkFun.

_<COLLABORATION CREDIT>_

