# ESP-DRONE-TRACKER
ESP32 WITH BG96 DRONE TRACKER 


## Firmware Testing:
- LTE BG96  --->  PASS
- UBLOX M6N --->  PASS
- IMU       --->  PASS
- PMB280    --->  PASS 
- LTE LED   --->  PASS
- Anti-temp --->  NOT YET
- Battery   --->  Failed  
    battery test faild due to GPIO36 internal damage, Always read 3.3v !!

### To do:
- complete state detection 
- complete event setting 