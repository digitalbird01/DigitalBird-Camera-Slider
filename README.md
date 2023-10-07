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

Software Updates:

7th Oct 2023- All parts of the system have been updated to work with the most up to date version of the PTZplus WIFI controller and the new DB3 PTZ head
This update adds the ability to control an additional 4th camera system together with 3 additional presets/Poses for each camera system. The updated DB3 software fixes a bug
which intermitantly provided an incorrect IP adress to the VISCA base. In order to use this update you will have to update all parts of your system to the latest version.
Be sure to use the libraries provided in the Digital Bird repository and not any updates provided by the original author's since it is not practical for me to change the Digital Bird system
every time a library is updated.

DB3 VISCA pan Tilt head version 1.0 has been added. Note this requires you are running version 5 of the PTZplus WIFI controler software.

Easy_Nextion_Library-DB.zip  This library has been customized for Digital Bird use. You must use this version of the library for Digital Bird system coding


DB panTilt_B_4.19   This fixes a bug preventing the focus motor limits being set in either clockwise or anticlockwise orientation.


DB V4.09 --- This update fixes some issues for the Mini Jib and the way it comumicates with the other devices. Not worth updating unless you have a Mini Jib.


DB V4.08 21/11/22 --- Bug fix causing timing problems and camera shutter release on Timlapse and Stop motion moves. This fix requires update to be performed on both Pan Tilt heads, The Slider and the Turntable. Controller software is not effected.



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

                                  

