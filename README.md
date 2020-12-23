# NRF51 SENSOR TAG / ITC PolyU HK

The project is reimplementing firmware for **NRF51 SENSOR TAG** also known as **ITC PolyU HK**. 

Because the [default firmware](https://s3.amazonaws.com/cutedigi/Sensor/BLEsensortag/factory_default.zip) doesn't work for me.

The project is created with [platformio](https://platformio.org). 
As IDE I used vscode with platfromio extension.

![nrf51_sensor_tag][Nrf51_sensor_tag.jpg]

[schematic](https://s3.amazonaws.com/cutedigi/Sensor/BLEsensortag/sensor_tag_sch_mpu6050.pdf) and other sources I find here.

## Redistribution

feel free to copy/update/redistribute

If you find any issue then please create merge request.

## Known issues

Ambient sensor **AP3216** is causing disconnection of BLE.

Debugging does'nt work properly. Double restart of debug session is needed for proper jlink connection.

## Other links I have found
https://forum.mysensors.org/topic/6951/nrf5-multi-sensor-board-12-14

https://github.com/kasbert/nrf_beacon_itcpolyuhk

https://www.linksprite.com/wiki/index.php?title=Bluetooth_4.0_BLE_Sensor_Tag/iBeacon_Station_NRF51822

Have fun!
