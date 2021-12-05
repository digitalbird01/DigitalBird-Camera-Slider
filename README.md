# DigitalBird-Camera-Slider
Professional level DIY Camera slider
This repository is for all the software used to drive the Digital bird camera motion control system.

Currently the system consists of 5 parts. The Slider, Pan Tilt Head, Cinematic Turntable and Focus motor and the WIFI remote display
Any of the parts can be used indevidually or together and are all controled from the Single WIFI remote.

All parts of the system now use the ESP32 processor
For detailed Instructions on how to install this software read the file titled "DigitalBird-Software Installation Guide.pdf"
The first step is to click on the green "CODE" button and select download ZIP, Unpack the zip to your computer but do not unpack the library zips included inside it.
Then follow the instruction in the aforementioned .pdf which will come down with the code

Note there are currently two different .ino files for the pan Tilt head. THe one ending in OT is for the original over the top model. The one ending B is for the newer balanced model. The only real differance is speed settings to compenate for the differing gear ratios on the different models.

WARNING Notice:
If updating the code on the built device be sure to remove the battery from the device before you plug in the usb cable.
Failure too do so may fry your ESP32 with to much power and also damage your PC usb port.
I will not be held liabal for either. This is generally the case with all project boards.
