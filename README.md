# APWebServer project
## IoT board
![Arduino NANO 33 IoT](/img/img1.png)
- Arduino NANO 33 IoT
  - ARM Cortex M0+ controller
  - NINA WiFI/BLE module
  - IMU ((Inertial Measurement Unit)
  - Crypto chip
  - Reset button
  - micro-USB
- School provided sensor board
  - BME280
  - TSL2591
  - SR602
  - Button
  - NTC sensor and magnetometer
  - Terminal strip
 - Board setup:
 ![Setup](/img/img2.png)
### Code based on Arduino sample projects from each library:
- WiFiNINA
  - AP_SimpleWebServer
- TSL2591
  - tsl2591
- BME280
  - bme280test
---
## Instructions
- Clone repo
- Create arduino_secrets.h file
- Write your WiFi information to the file:

> #define SECRET_SSID "wifi"\
> #define SECRET_PASS "wifipw"

- Include all necessary libraries to the project. (Check paths in the c_cpp_properties.json if you are using VS Code) 

Verify & Upload to Arduino.

Check COM port for log information!
