# SentryGun
The software and coding belonging to a Nerf Vulcan autonomous sentry gun (still in development; unstable and non-functioning as of now)

Software/Libraries/Hardware To Be Used:

Arduino Uno
Camera module for Arduino Uno (I believe it's an ESP8266)
Arduino IDE v1.0.3 (uses customized C/C++ to command the Arduino and some libraries to define functions for the Arduino's operations) 
Processing v1.5.1 (uses customized Java to make GUIs and other visual programming easy - in this case, it runs the ProjectSentryGun GUI and allows for manual control in the GUI, as well as incorporating some camera recognition modules)
Assorted axial ball bearings
A lot of wood
The bottom half of a drum stool
An HS-805BB+ servo motor to rotate gun about the x-axis
A BMS-660DMG+HS servo motor to rotate gun and its mount about the y-axis
The ProjectSentryGun GUI and some of its DLLs and plugins would be used for camera recognition, full functionality, and a few other features (to be honest, I'm not sure what half of it does; I just know that it makes sentry guns work, and that's good enough for me)


TODO: Test servos with various power sources in school, seeing as how servos for both axes don't work.

Goal: Have a working, autonomous sentry gun which can identify, track, and fire at targets. Being able to manually control it via WiFi (an HTML page, a VPN, port forwarding, and a lot of router configuration would be used to make this happen) would be nice, but at this point, I want to focus on the camera aspect first.
