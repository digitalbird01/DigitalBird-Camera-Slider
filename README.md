# DigitalBird-Camera-Slider
Professional level DIY Camera slider
This repository is for all the software used to drive the Digital bird camera motion control system.

Currently the system consists of 4 parts. The Slider, Pan Tilt Head, Cinematic Turntable and Focus motor

All parts of the system  know use the ESP32 processor

To Build the Controler you will need the following hardware
1) Adafruit HUZZAH 32 Feather ( no pins attached. This board is used along side the Nextion display inside the remote)
2) 3.2" Nextion display ( not the enhanced version we dont need the extras)
3) Adafruit 5V mini boost (To bring the boards 3.3v up to 5v for the Nextion
4) Switch harness and cables
5) A 1200 mAh lipo battery with JST-PH 2.0mm plug
6) Your own 3D printed case files downloadable from thiniverse

To Build the Slider you will need the following:
1) A slider kit from digital bird or all the componants included with the kit from your own sorces
2) You will need the Digital Bird Main board and an ESP32 Dev board typicaly the Espressif ESP32-DevKitC-32D AS5600 encoder and other parts (see kit specifications)

First download all the files as a zip The software was written using the Arduino IDE which can be downloaded free here: https://www.arduino.cc/en/software
Because we are working with the ESP32 board and not an Arduino we need to install the drivers for our ESP32 boards into the Arduino IDE
In The arduino IDE go to <File> <Preferances> and in the box labeled "Additional Boards Manager URL's" add the following link https://dl.espressif.com/dl/package_esp32_index.json
Next select <tools><Boards><Boards Manager> and you should see a window apear with the board manager. In the dialog box type ESP32 and return.
Bellow the dialoge box you should know see the listing for the ESP32 drivers. Select install and all the ESP32 drivers will be added to your system.

The software makes use of the following libraries which must be installed into the Arduino IDE before uploading the software to the boards

FastAccelStepper.h           (To control the steppers)

AS5600.h                     (For the AS5600 magnetic encoder used on the slider and Turntable)

EasyNextionLibrary.h         (For control of the Nextion display)

These librarys are included as zip files here in the Digital bird repository and should be in the folder you downloaded and unpacked the repository to.
For each library there should be a zip file we dont need to unpack these files. 
From the Arduino IDE Menu Bar at the top of the screen select <Sketch> <Include Library> <Add .ZIP library> and point this at each of the three zipped libraries one at a time.
You only need to do this ones to install all the digital bird software.

The next step is to install the slider software on the Espressif ESP32-DevKitC-32D board.
First off all make sure there are no other power supplies attached to the board no batteries plugged into the slider or external supply. 
This is important to ensure we dont over power and fry the board. Pluggin your micro USB cablle into the board and into your computer the green LED should light up on the board
and the computer should achnolage the attachment of a new device.
In the Arduino IDE click on <Tools> <Port> and select the USB port the board is using If more than one com port is on the list Unplug your board and see which one disapears!
When you have identified the correct comport select it.
  
Next we need to tell the Arduino IDE which type of board we are using to do this select <Tools> <Board> <ESP32> <ESP32 Dev Module>
  
Know we are ready to install our software
Go to <File> <Open> and navigate to the downloaded Digital Bird repository on your computer. Find the file named DBsliderESP32-01.ino and select it.
With the file open in The Arduino IDE click <sketch> <Upload> from the menu bar and the software will be compiled with the libraries and uploaded to the ESP32. This may take a few minuits. Provided all is well and the libraries have all been correctly installed etc you should get the message "Upload complete" and the job is done. If not and you get a red compilor error read the error and if it is not obviouse what has gone wrong, copy down the message and let me know.
  
  Know Plug in the Adafruit HUZZAH 32 Feather board. this board will be used to interface the Nection display with the rest of the system.
  First we need to tell the Arduino IDE that we want to use a different board. Once again navigate to <Tools> <Board> <ESP32> <Adafruit ESP32 feather>
  know find the software for the feather <sketch> <Upload> and find the filename DBNextionESP32-01.ino
  once loaded select <sketch> <Upload> and the sketch should upload to the board with the message "upload complete" when done.
  
  The last step is to install the user interface to the Nextion display. For this we do not need the Arduino iDE just an empty micro SD card.
  copy the file titled DBNextionESP32.tft onto the micro SD card.
  Place the card into the Nextion SD card slot and power on the device. The Nextion should find the file and automaticly install the software.
  When complete remove the SD card and power down the display.
  When we next power up the display you should see the Digital Bird Menu screen in all its glory.
  
  Job done
  
