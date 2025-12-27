Humidity based FanController for 2 PWM Fans with 2 DHT22 Sensors created with ChatGPT

I needed a Way to control the Humidity in my Basement Workshop but I found nothing on the Internet that suited my needs so I consulted ChatGBT actually for the first Time ^^
After many Weeks and many many Versions and Issues I am now happy with the Code and how it is working.
I am using it with two Noctua Industrial Fans and a Condeser Dryer Heat Exchanger the ForceFanPin is planed for a Soldering Smoke Extraction

Features:

- Two DHT22 Sensors for Room and Intake Air Temperature and Humidity
- The Sensors are Calibratable with 2 Values per Sensor
- HumOffset sets how much higher the Humidity on the Inside can be in comparisson to the Intake before the Fans ramp up
- If MinHumidity is reached the Fans spin with MinPWM
- If FanOffHumidity is reached the Fans turn Off until MinHumidity is reached again 
- Two PWM Fans for Intake and Outlet with RPM Display
- Setting a minPWM, maxPWM and an FanOffset to let the Intake spin a bit faster
- The Code is designt for a LCD1602 + I2C, 3 Buttons (Menue,+ and -) an an Status LED
- Pin 4 is forcing the Fans to 100% if pulled to Ground
- Pin 5 is pulled high if PreheatTemp is reached with a Hysteresis of +1 Degree
- The PID Values can be canched in the Menue
- The LED is pulsing in normal Mode, slowly flashing if the Fans are off and rapidly flashing if there is a Sensor Error
- If no Buttons is pressed for 10 Second while in Menue it is exited automaticly
- The Display turns off after 30 Seconds with no Input
- The Values are only written to the EEPROM when leaving the Menue to protect it from unnecessary Writes
- You can change the LEDs on, off and pulsing Times on Top of the Code

I Hope someone finds this helpfull and can use my Code

For Calibrating I Placed the Sensors in a sealed Container with dried Silica Gel a few Hours for the Low Value (should be around 10%) and with saturated Table Salt Water for the High Value (should be around 75%)
