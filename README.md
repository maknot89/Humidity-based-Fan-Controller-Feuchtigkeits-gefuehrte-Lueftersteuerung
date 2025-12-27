Humidity based FanController for 2 PWM Fans with 2 DHT22 Sensors created with ChatGPT

I needed a way to control the humidity in my basement workshop, but I couldn’t find anything online that suited my needs. So I consulted ChatGPT — actually for the first time 😄
After many weeks, countless versions, and plenty of issues, I am now happy with the code and how it works.

The system is currently running with two Noctua Industrial fans and a condenser dryer heat exchanger. The ForceFanPin is planned for a soldering smoke extraction system.

Features:

- Two DHT22 sensors for room and intake air temperature and humidity
- Sensors can be calibrated using two values per sensor
- HumOffset defines how much higher the indoor humidity may be compared to the intake air before the fans ramp up
- If MinHumidity is reached, the fans run at MinPWM
- If FanOffHumidity is reached, the fans turn off until MinHumidity is reached again
- Two PWM-controlled fans (intake and outlet) with RPM display
- Configurable MinPWM, MaxPWM, and a FanOffset to make the intake fan run slightly faster
- Code is designed for an LCD1602 with I²C, three buttons (Menu, +, −), and a status LED
- Pin 4 forces both fans to 100% when pulled to ground
- Pin 5 goes high when PreheatTemp is reached, with a hysteresis of +1 °C
- PID values can be changed in the menu
- Status LED behavior:
  - Pulsing in normal operation
  - Slow flashing when the fans are off
  - Rapid flashing if a sensor error occurs
- If no button is pressed for 10 seconds while in the menu, it exits automatically
- The display turns off after 30 seconds without input
- Values are written to EEPROM only when leaving the menu to avoid unnecessary writes
- LED on/off and pulsing timings can be adjusted at the top of the code


I hope someone finds this helpful and can make use of my code.


For calibration, I placed the sensors in a sealed container:

- With dried silica gel for several hours to get the low value (≈10% RH)
- With saturated salt water (table salt) for the high value (≈75% RH)
