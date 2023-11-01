


# DigitalBird Camera motion control system


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

# Software Updates:
Update Notes v6.00: 23rd Oct 2023

  1) Now supports up to four WIFI camera systems rather than the original three.
  2) Also know supports 9 pose/preset positions over the original 6. 16 are available for wired VISCA control
  3) Two poses 15 & 16 have been reserved in order to allow triggering of the current A-B and sequencer setups with or withought bounce.
  these can also be triggered from VISCA by calling pose 15 or 16 respectivly.
  4)The setup menu also now alows you to set a delay at the start and end of bounce moves. This also sets a recording pause at the end of A-B moves
  before the system returns to its home position. The pause can be 1-10 seconds. This setting is stored in memorry and recalled on reboot.
  5) A number of small bugs have been fixed along the way however I am unsure which of these were in the last version and which were required as
  a result of the added functionality so I will not list.
  6)ESP32 board updates were giving issues with the encoders this has know been fixed and we are know able to compile the code with Arduino IDE 2.*
  7)The application also now uses the currently current! FastAccelStepper library.
  8) Firmware numbering has been updated across the hole system. To keep things simple all parts of the system must be running the first part of the version number so this update is 6.
  If indevidual parts of the system require an update which does not effect the other parts this can happen as here v6.02. The first number "6" will only increment if the update effect all parts of the system
  and it will increment on all firmware parts. So in the case of this update all parts of the system must be updated to v6.** stay compatable.
                                  

