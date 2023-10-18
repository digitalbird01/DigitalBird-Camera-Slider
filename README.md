# Important Notes

Important Notes: Currently the Digital bird system will only work with the following setup:

* Arduino IDE 1.8.15  to Arduino IDE 1.8.19
* esp32 expressive boards version 1.06
* Fastaccelstepper version 0.25




# DigitalBird Camera motion control system
I have an issue conserning the encoders not working correctly with the latest esp32 board firmware. Unfortunatly Arduino IDE 2 and above board manager only offers
us two versions of the esp32 board firmware both of which are causing a problem. Until I have uncovered and fixed the issues this notice will stay in place. You can onlu use the Arduino IDE 1.8.19
with the supplied libraries to compile and upload the code.

Professional level DIY Camera Motion Control System
This repository is for all the software used to drive the Digital bird camera motion control system.
This projects aim is to provide the most feature rich DIY camera motion control system on the internet.

All of the 3D printed parts can be found on the Printables or Thingiverse website and I Supply kits for all the non 3D Print parts
including pre programed PCB main boards on the Digital Bird shoppify channel. No soldering or programming required.

Also check out the digital bird Youtube and Autodesk Instructables website for detailed build instructions.

Currently the system consists of 7 parts. The Slider, a choice of 3 Pan Tilt Heads , Cinematic Turntable, Focus motor, compact WIFI controller
PTZplus WIFI Controller and the mini jib. Any of the parts can be used indevidually or together and are all controled from the Single WIFI remote.

The system offers the following functionality:
* A-B straight moves with ease or ramp, bounce, and timlapse control.
* 6 Key sequencer control for more complex moves
* Basic stop motion animation functions
* 3 camera PTZplus control for realtime camera control with each camera system able to store 6 key poses
* The New DB3 PTZ head also supports VISCA over IP with support for OBS & vMix


All parts of the system use the ESP32 processor
For detailed Instructions on how to install this software read the file titled "DigitalBird-Software Installation Guide.pdf"
The first step is to click on the green "CODE" button and select download ZIP, Unpack the zip to your computer but do not unpack the library zips included inside it.
Then follow the instruction in the aforementioned .pdf which will come down with the code

Note there are currently two different .ino files for the pan Tilt head. The one ending in OT should be used for the original over the top model. The one ending B is for the newer balanced model. The only real differance is speed settings to compensate for the differing gear ratios on the different models.
If you are updating software from previose versions please note that you will have to update all parts of the system at the same time in order for them to stay compatable.

WARNING Notice:
If updating the code on the built device be sure to remove the battery from the device before you plug in the usb cable.
Failure too do so may fry your ESP32 with to much power and also damage your PC usb port.
I will not be held liable for either. This is generally the case with all project boards.

Software Updates:

7th Oct 2023- All parts of the system have been updated to work with the most up to date version of the PTZplus WIFI controller and the new DB3 PTZ head
This update adds the ability to control an additional 4th camera system together with 3 additional presets/Poses for each camera system. The updated DB3 software fixes a bug
which intermitantly provided an incorrect IP adress to the VISCA base. In order to use this update you will have to update all parts of your system to the latest version.
Be sure to use the libraries provided in the Digital Bird repository and not any updates provided by the original author's since it is not practical for me to change the Digital Bird system
every time a library is updated.


                                  

