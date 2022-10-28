# DigitalBird Camera motion control system
Professional level DIY Camera Motion Control System
This repository is for all the software used to drive the Digital bird camera motion control system.
This projects aim is to provide the most feature rich DIY camera motion control system on the internet.

All of the 3D printed parts can be found on the Thingiverse website and I Supply kits for all the non 3D Print parts
including pre programed PCB main boards on the Digital Bird shoppify channel. No soldering or programming required.

Currently the system consists of 7 parts. The Slider, Pan Tilt Head, Cinematic Turntable,Focus motor, compact WIFI controller
PTZplus WIFI Controller and the mini jib. Any of the parts can be used indevidually or together and are all controled from the Single WIFI remote.

The system offers the following functionality:
* A-B straight moves with ease or ramp, bounce, and timlapse control.
* 6 Key sequencer control for more complex moves
* Basic stop motion animation functions
* 3 camera PTZplus control for realtime camera control with each camera system able to store 6 key poses
* Unlimited programed camera functions at the same time as the 3 PTZplus cameras are being controled

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

Software Updates 

DB V4.06 --- This update fixes some issues arround Bounce timing and Clr keys function the update is only for the Pan Tilt, Turntable & Slider
              which must all be running the same version 4.06. The controler software remains at 4.00.
             


DB V4.00 --- All parts of the system must be running the same version of the software
             
             * All parts of the system know recogize the mini jib
             
             * Some small bug fixes
             
             * A new clear key points button has been added to the home menu which will Zero all key positions which are otherwise stored even when the system is powered down.

DB V3.00 ---  This is a major update with a host of new features including:
                                
         * PTZplus Live camera control for up to 3 Live cameras with up to six pose memory for each camera system.
         (requires PTZplus controller)
         
         * The system now saves all key positions to non volatile memory so even after the power is recycled the key points are stored.
         
         * Cleaner more efficient user interface
         
         * Additional menu functions for stop motion animation

                                  

