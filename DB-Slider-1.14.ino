


//****Slider Code***** (see also Pan Tilt code thay work together over wifi link)
/* Stepper Camera Slider with Nextion Touchscreen LCD Control wifi connectivity and Encoder flexability *\

  Created by Colin Henderson for the 3D printable DigitalBird 3 axis camera Slider available from Thingverse

  This code is in the public domain...
  You can: copy it, use it, modify it, share it or cry over it as I have on many occasions!


*/


//Wifi Setup
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(10, 9); // CE, CSN
const byte address[][6] = {"0001", "0002"};
void(* resetFunc) (void) = 0;


//AMS AS5600 Encoder setup   The encoder keeps track of the camera position without having to have the stepper driven
#include <AS5600.h>
AS5600 encoder;
int revolutions = 0;   // number of revolutions the encoder has made
long E_position = 0;    // the calculated value the encoder is at
long output;          // raw value from AS5600
long E_outputPos = 0;    //Ajustment value
long E_lastOutput;        // last output from AS5600
long S_position = 0;    // Number of stepper steps at 16 microsteps/step interpolated by the driver to 256 microsteps/step (confusing!)
long E_Trim = 0;        // A Trim value for the encoder effectively setting its 0 position to match Step 0 position
long E_Current = 0;
int E_Turn = 0;
int E_outputTurn = 0;
long E_outputHold = 32728;
long loopcount = 0;
long  S_lastPosition = 0;

//main setup
#include <Nextion.h>          // Nextion Library by Bentley Born https://github.com/bborncr/nextion
#include <SoftwareSerial.h>   // Software Serial Port (included in Arduino IDE "no download")
#include <AccelStepper.h>     // Accelstepper by Mike Mcauley https://www.airspayce.com/mikem/arduino/AccelStepper/index.html
#include <MultiStepper.h>
#define home_switch A1       // Pin A1 defined for limit switches
//#define stepperDriver 4       // Pin 4 Stepper driver actice inactive=high active pull low
#define camera_trigger A0     // Pin A0 Camera trigger
const int pinSTEP = 7;        // Pin 6 connected to STEP pin of Stepper Driver
const int pinDIR = 6;         // Pin 5 connected to DIR pin of Stepper Driver
const int stepperDriver = 8;



AccelStepper stepper(1, pinSTEP, pinDIR);  // Setup of the AccelStepper Library

SoftwareSerial nextion(0, 1);            // On Nextion RX yellow connected to Arduino pin 0 and TX Blue to pin 1

Nextion NextionLCD(nextion, 9600);         // Setup of Nextion library with name NextionLCD at 9600bps

long Rail_Length = 500;                     //Edit this to use any rail length you require in mm
long Max_Steps = 0;                         //Variable used to set the max number of steps for the given leangth of rail

long set_speed = 0;
long in_position = 0;                       // variable to hold IN position for slider
long out_position = 0;                      // variable to hold OUT position for slider
long travel_dist = 0;
long step_speed = 0;                        // default travel speed between IN and OUT points
int crono_time = 0;
int crono_hours = 0;
int crono_minutes = 0;
int crono_seconds = 0;
String DBFirmwareVersion= "frm V:1.00";
String nextion_message;                   // variable to hold received message for Nextion LCD
int move_left = 0;                        // variable to detect move left on Nextion LCD
int move_right = 0;                       // variable to confirm move right on Nextion LCD
int start_cycle = 0;                      // variable to confirm start of travel between IN and OUT points
long Home_position = 0;                    // Variable to st home position on slider
int ease_InOut = 0;                       // Variable to collect Ease_InOut from nextion screen value range 1-10
int ease_Value = 0;                       // Variable to hold actual Ease value used for acceleration calculation
int ease_time = 0;                        // Extra time allowed for ease in out calculation
int move_finished = 1;                    // Used to check if move is completed
long initial_homing = -1;                 // Used to Home Stepper at startup
int nextion_focus = 0;
int Bounce = 0;
int fps = 0;
int fps_countdown = 0;
int fps_step_dist = 0;
int TL_delay = 2000;
int Buttonset = 0;
int timelapse_projected = 0;
int getEncoder = 0;
int inpointPress = 0;
int outpointPress = 0;
int TiltPanComand = 0;
int playbuttoncounter = 1;
String dataTx;
String wifimessage;
int PanTiltPlay;
int PanTiltINpoint;
int PanTiltOUTpoint;

int WIFIOUT[8];    //set up 5 element array for sendinf values to pantilt



void setup() {
  pinMode (7, OUTPUT);    //STEP pin of Stepper Driver
  pinMode (6, OUTPUT);    //DIR pin of Stepper Driver
  pinMode (A0, OUTPUT);   //Camera Shutter
  pinMode (A1, OUTPUT);   //Limit Switch
  pinMode (8, OUTPUT);    //Stepper Activation

  // set Pins
  digitalWrite(8, LOW);    //Stepper Activation
  digitalWrite(A1, LOW);   //Limit Switch
  digitalWrite(A0, LOW);   //Camera Shutter


  //wifi setup
  //Serial.begin(9600);
  Serial.println("Listening1");
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH); //set power levelto HIGH max range.
  radio.setDataRate(RF24_1MBPS); //Data rate The lower or slower the data rate the higher the recever sensitivity
  radio.setChannel(124);                    //Channel to use. I may allow this to be changed in future
  radio.openWritingPipe(address[1]);       //open writing and reading pipes on each radio with opposite addresses
  radio.openReadingPipe(1, address[0]);
  radio.stopListening();

  // Nextion setup
  NextionLCD.init();
  nextion_message = NextionLCD.listen(); //check for message from Nextion touchscreen


  String dims = "dims=" + String(100);  // set display brightness to 100%
  NextionLCD.sendCommand(dims.c_str());
  // AS5600 Encoder setup


  // Setup for the rail length
  Max_Steps = (((Rail_Length - 43) * 58) / 2);

NextionLCD.setComponentText("t9", String(DBFirmwareVersion));           // Show current firmware version on display

  //Motor homing code
  pinMode(home_switch, INPUT_PULLUP);//Pin A1 pulled high for limit switches
  delay(100);  // Wait for StepperDriver to wake up


  //  Set Max Speed and Acceleration of each Steppers at startup for homing
  stepper.setMaxSpeed(12000);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(9000);  // Set Acceleration of Stepper
  stepper.setSpeed(9000);


  // Start Homing procedure of Stepper Motor at startup

  while (digitalRead(home_switch)) {  // Make the Stepper move CCW until the switch is activated
    stepper.moveTo(initial_homing);  // Set the position to move to
    initial_homing = initial_homing - 10; // Decrease by 10 for next move if needed
    stepper.run();  // Start moving the stepper
    delay(1);                          //Just keeps it slow
  }

  stepper.setCurrentPosition(0);  // Set the current position as zero for now
  stepper.setMaxSpeed(8000);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(300);  // Set Acceleration of Stepper

  delay(5);

  stepper.moveTo(600);              // Move back off the home switch 200 steps to set the stepper

  stepper.runToPosition();

  stepper.setCurrentPosition(0);    //Call this position 0 or home. Setup complete
  encoder.setZero();

}




void loop() {


  //**********************************ENCODER START**************************************


  if (getEncoder != 0)   {


    //**************************************Encoder is in First range***************************************
    output = (encoder.getPosition() );                            //Read the encoders Raw value

    if (output < 0) {                                             //IF the value is negative encoder is in first range
      E_outputPos = (output * -1);

    }
    //**************************************Encoder is in Second range***************************************

    if (output > 0) {                                               //IF the value is Posative encoder is in Second range
      E_outputPos = (E_outputHold + (E_outputHold - output)) ; //Encoder outside range. Add the first range to the current range
      //NextionLCD.setComponentText("t4",String(E_outputPos));
    }

    //*************************************** Set the New position********************************************
    if (getEncoder == 1) {      //=1 in point pressed

      S_position = ((E_outputPos / 2.56));               //Ajust encoder to stepper values
      if (output < 0) {                                  //carrage was moved to the left Add
        S_position = (S_position + in_position);

      } else {                                           //carrage was moved to the Right Subtract
        //NextionLCD.setComponentText("t4",String(S_position));
        S_position = (25567 - S_position);
        S_position = (in_position - S_position );
      }


    } else {                                             //=2 outpoint pressed
      S_position = ((E_outputPos / 2.56));               //Ajust encoder to stepper values
      if (output < 0) {                                  //carrage was moved to the left Add
        S_position = (S_position + out_position);

      } else {                                           //carrage was moved to the Right Subtract
        S_position = (25567 - S_position);
        S_position = (out_position - S_position);
      }
    }

//  *****Error checking if encoder calc out off slider range --reset the slider*******
  
  if (S_position >13262){
    ErrorReset();
    resetFunc();
  }
  }
  //**************************************ENCODER END*********************************************


  //**************************************wifi LISTEN FOR COMMANDS FROM PANTILT HEAD*********************************************
  radio.startListening();

  if (radio.available()) {
    unsigned char dataRx;

    radio.read(&dataRx, sizeof(unsigned char));
    if (dataRx) {


      //Play Button receved
      if (dataRx == 1) {    // Play Button action if not at atart) {

        digitalWrite(8, LOW);
        delay(20);
        if (crono_seconds == 0) {
          crono_seconds = 10;
          NextionLCD.setComponentText("t2", "10");   //set seconds at 10sec defalt to avoid false starts
        }

        ++PanTiltPlay;
        if (PanTiltPlay == 1) { //First press
          SetSpeed();
          TimerSet();
          delay(20);
          Start_1();
        }
        if (PanTiltPlay == 2) {  //Second press
          if (stepper.currentPosition() == in_position)  { // Play Button ready to play
            SetSpeed();
            TimerSet();
            delay(20);
            if (fps == 0) {
              digitalWrite(A0, HIGH);                      //Start camera
              delay (200);
              digitalWrite(A0, LOW);                      //Start camera
              Start_2();
            } else {
              Start_4();
            }
            PanTiltPlay = 1;
          }
        }
      }


      //Inpoint button received
      if (dataRx == 2 ) {
        playbuttoncounter = 1;
        INpointSet();
        dataRx = 0;
        playbuttoncounter++;
        radio.startListening();
      }


      //outpoint button received
      if (dataRx == 3 ) {
        OUTpointSet();
        dataRx = 0;
        radio.startListening();
      }


      dataRx = 0;
      radio.startListening();
    }

  }


  //*************************************Listen for Nextion LCD comands*******************************************
  nextion_message = NextionLCD.listen(); //check for message from Nextion touchscreen

  if (nextion_message != "") { // if a message is received...


    //****Estabish menu focus****
    if (nextion_message == "65 0 14 1 ffff ffff ffff") { // nextion_focus set to m0 hours
      nextion_focus = 0;
      Buttonset = 1;
      inpointPress = 0;
      outpointPress = 0;
    }
    if (nextion_message == "65 0 15 1 ffff ffff ffff") { // nextion_focus set to m1 minits
      nextion_focus = 1;
      Buttonset = 1;
      inpointPress = 0;
      outpointPress = 0;
    }
    if (nextion_message == "65 0 16 1 ffff ffff ffff") { // nextion_focus set to m2 seconds
      nextion_focus = 2;
      Buttonset = 1;
      inpointPress = 0;
      outpointPress = 0;
    }
    if (nextion_message == "65 0 17 1 ffff ffff ffff") { // nextion_focus set to m3 Ease
      nextion_focus = 3;
      Buttonset = 1;
      inpointPress = 0;
      outpointPress = 0;
    }
    if (nextion_message == "65 0 18 1 ffff ffff ffff") { // nextion_focus set to m4 Bounce
      nextion_focus = 4;
      Buttonset = 1;
    }
    if (nextion_message == "65 0 19 1 ffff ffff ffff") { // nextion_focus set to m5 fps
      nextion_focus = 5;
      Buttonset = 1;
      inpointPress = 0;
      outpointPress = 0;
    }
    if (nextion_message == "65 0 1a 1 ffff ffff ffff" || TiltPanComand == 2) { // nextion_focus set to m6 Inpoint
      nextion_focus = 6;

      PanTiltPlay = 0;  //tell the play button to reset
      outpointPress = 0;
      ++PanTiltINpoint;
      if (PanTiltINpoint == 1) {
        INpointSet();
        PanTiltPlay = 0;
        SendPeramiters ();

      }
      if (PanTiltINpoint == 2) {
        INpointSet();
        PanTiltPlay = 0;
        SendPeramiters ();
        PanTiltINpoint = 0;
      }
      if (PanTiltINpoint == 3) {
        PanTiltINpoint = 0;
      }

    }

    if (nextion_message == "65 0 1b 1 ffff ffff ffff") { // nextion_focus set to m7 OutPoint
      nextion_focus = 7;
      //NextionLCD.setComponentText("t4", String(PanTiltOUTpoint));
      PanTiltPlay = 0;  //tell the play button to reset
      inpointPress = 0;
      ++PanTiltOUTpoint;
      if (PanTiltOUTpoint == 1) {
        OUTpointSet();
        SendPeramiters ();
      }
      if (PanTiltOUTpoint == 2) {
        OUTpointSet();
        SendPeramiters ();
        PanTiltOUTpoint = 0;
      }
      if (PanTiltOUTpoint == 3) {
        PanTiltOUTpoint = 0;
      }
    }

    //***End find focus****

    //******Set run time on Nextion HOURS***********
    if (nextion_message == "65 0 1c 1 ffff ffff ffff" && nextion_focus == 0 ) {  // + time Hours on nextion

      if (crono_hours >= 0 && crono_hours <= 23) {                           //limit hours that can be set between 0 & 24
        crono_hours = crono_hours + 1;                                      // increase hour by 1
        NextionLCD.setComponentText("t0", String(crono_hours));           // display Hours in t0 box on Nextion
        crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
        PanTiltPlay = 1;
        SendPeramiters();
      }
    }
    if (nextion_message == "65 0 1d 1 ffff ffff ffff" && nextion_focus == 0 ) {   // - time Hours on nextion
      if (crono_hours >= 1) {                            //limit hours that can be set between 0 & 24
        crono_hours = crono_hours - 1;                                     // decrease hour by 1
        NextionLCD.setComponentText("t0", String(crono_hours));           // display Hours in t0 box on Nextion
        crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
        PanTiltPlay = 1;
        SendPeramiters();
      }
    }

    //******Set run time on Nextion MINUTES***********
    if (nextion_message == "65 0 1c 1 ffff ffff ffff" && nextion_focus == 1 ) {  // + time minutes on nextion
      if (crono_minutes >= 0 && crono_hours <= 59) {                         //limit minutes that can be set between 0 & 60
        crono_minutes = crono_minutes + 1;                                 // increase minutes by 1
        NextionLCD.setComponentText("t1", String(crono_minutes));         // display minutes in t1 box on Nextion
        crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
        PanTiltPlay = 1;
        SendPeramiters();
      }
    }
    if (nextion_message == "65 0 1d 1 ffff ffff ffff" && nextion_focus == 1 ) {   // - time minutes on nextion
      if (crono_minutes >= 1) {                        //limit minutes that can be set between 0 & 60
        crono_minutes = crono_minutes - 1;                                 // decrease minutes by 1
        NextionLCD.setComponentText("t1", String(crono_minutes));         // display minutes in t1 box on Nextion
        crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
        PanTiltPlay = 1;
        SendPeramiters();
      }
    }

    //******Set run time on Nextion SECONDS***********
    if (nextion_message == "65 0 1c 1 ffff ffff ffff" && nextion_focus == 2 ) { // + time seconds on nextion
      if (crono_seconds >= 0 && crono_seconds <= 59) {                        //limit seconds that can be set between 0 & 60
        crono_seconds = crono_seconds + 1;                                // increase minutes by 1
        NextionLCD.setComponentText("t2", String(crono_seconds));        // display seconds in t2 box on Nextion
        crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
        PanTiltPlay = 1;
        SendPeramiters();
      }
    }
    if (nextion_message == "65 0 1d 1 ffff ffff ffff" && nextion_focus == 2 ) { // - time seconds on nextion
      if (crono_seconds >= 1) {                      //limit seconds that can be set between 0 & 60
        crono_seconds = crono_seconds - 1;                               // decrease seconds by 1
        NextionLCD.setComponentText("t2", String(crono_seconds));       // display seconds in t2 box on Nextion
        crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
        PanTiltPlay = 1;
        SendPeramiters();
      }
    }

    //****Set Ease Value*********
    if (nextion_message == "65 0 1c 1 ffff ffff ffff" && nextion_focus == 3 ) { //+Ease Value focus
      if (ease_InOut >= 0 && ease_InOut <= 2) {                          //limit Ease 0-3
        ease_InOut = ease_InOut + 1;                                     // increase  by 1
        NextionLCD.setComponentText("t3", String(ease_InOut));          // display value in t3 Ease box on Nextion
        PanTiltPlay = 1;
        SendPeramiters();
      }
    }

    if (nextion_message == "65 0 1d 1 ffff ffff ffff" && nextion_focus == 3 ) { // - Ease Value
      if (ease_InOut >= 1) {                               //limit Ease 0-10
        ease_InOut = ease_InOut - 1;                                      // decrease seconds by 1
        NextionLCD.setComponentText("t3", String(ease_InOut));          // display value in t3 Ease box on Nextion
        PanTiltPlay = 1;
        SendPeramiters();
      }
    }

    //****Set Bounce*********
    if (nextion_message == "65 0 1c 1 ffff ffff ffff" && nextion_focus == 4 ) { //+Bounce Value focus
      if (Bounce >= 0 ) {
        Bounce = Bounce + 1;                                     // increase  by 1
        NextionLCD.setComponentText("t4", String(Bounce));          // display value in t4 Bounce box on Nextion
        PanTiltPlay = 1;                                         //make sure that current play button setup is 0
        SendPeramiters();
      }
    }

    if (nextion_message == "65 0 1d 1 ffff ffff ffff" && nextion_focus == 4 ) { // - Bounce Value
      if (Bounce >= 1) {                                             //limit Ease 0-20
        Bounce = Bounce - 1;                                         // decrease seconds by 1
        NextionLCD.setComponentText("t4", String(Bounce));          // display value in t3 Ease box on Nextion
        PanTiltPlay = 1;
        SendPeramiters();
      }
    }



    //******fps for timelapse***********
    if (nextion_message == "65 0 1c 1 ffff ffff ffff" && nextion_focus == 5 ) { // + fps on nextion
      if (fps >= 0 ) {
        fps = fps + 25;                                                    // increase fps by 25
        NextionLCD.setComponentText("t5", String(fps));                  // display seconds in t2 box on Nextion
        if (fps != 0) {
          timelapse_projected = (5 + (fps * 2.501));
          crono_seconds = 2;
          NextionLCD.setComponentText("t2", "2");   //set seconds deay at 2
          NextionLCD.setComponentText("t1", "0");   //set min at 0
          NextionLCD.setComponentText("t0", "0");   //set hours at 0
          NextionLCD.setComponentText("t8", String(timelapse_projected));
          PanTiltPlay = 1;
          SendPeramiters();
        }
      }
    }
    if (nextion_message == "65 0 1d 1 ffff ffff ffff" && nextion_focus == 5 ) {    // - fps for timelapse
      if (fps > 0) {
        fps = fps - 25;                                                      // decrease fps by 1
        NextionLCD.setComponentText("t5", String(fps));                    // display fps in t2 box on Nextion
        if (fps == 0) {
          timelapse_projected = 0;
          crono_seconds = 10;                       //Back to 10 sec default values
          crono_time = 10;                          //Back to 10 sec default values
          PanTiltPlay = 1;
          SendPeramiters();
          NextionLCD.setComponentText("t8", String(timelapse_projected));
          NextionLCD.setComponentText("t2", "10");   //set seconds at 10
          NextionLCD.setComponentText("t1", "0");   //set min at 0
          NextionLCD.setComponentText("t0", "0");   //set hours at 0

        } else {
          timelapse_projected = (5 + (fps * 2.501));
          crono_seconds = 2;
          PanTiltPlay = 1;
          SendPeramiters();
          NextionLCD.setComponentText("t2", "2");   //set seconds delay at 2
          NextionLCD.setComponentText("t1", "0");   //set min at 0
          NextionLCD.setComponentText("t0", "0");   //set hours at 0
          NextionLCD.setComponentText("t8", String(timelapse_projected));

        }
      }
    }


    if (nextion_message == "65 0 1c 1 ffff ffff ffff" && nextion_focus >= 6 && (stepper.currentPosition() >= 50)) { // Move Left Button

      digitalWrite(8, LOW);
      getEncoder = 0;
      delay(10);

      stepper.setMaxSpeed(12000);
      stepper.setAcceleration(1000);
      stepper.setSpeed(4000);
      move_left = 1;
    }
    if (nextion_message == "65 0 1c 0 ffff ffff ffff" && nextion_focus >= 6) {  // Release Move Left Button

      move_left = 0;
      while (stepper.distanceToGo() != 0) {  // wait for stepper to reach last destination
        stepper.run();
      }
      if (nextion_focus == 6) {
        in_position = stepper.currentPosition();                                                                         // set the IN point to the current stepper position
        NextionLCD.setComponentText("t6", String(in_position));                                                          // display position in t6 box on Nextion InPosition
        PanTiltINpoint = 0;

      }
      if (nextion_focus == 7) {
        out_position = stepper.currentPosition();                                                                        // set the OUT point to the current stepper position
        NextionLCD.setComponentText("t7", String(out_position));
        PanTiltOUTpoint = 0;

      }

    }

    if (nextion_message == "65 0 1d 1 ffff ffff ffff" && nextion_focus >= 6 && (stepper.currentPosition() <= Max_Steps)) { // Move Right Button

      digitalWrite(8, LOW);
      getEncoder = 0;
      delay(10);
      move_right = 1;
      stepper.setMaxSpeed(12000);
      stepper.setAcceleration(1000);
      stepper.setSpeed(4000);
    }

    if (nextion_message == "65 0 1d 0 ffff ffff ffff" && nextion_focus >= 6 ) { // Release Move Right Button
      move_right = 0;
      while (stepper.distanceToGo() != 0) {  // wait for stepper to reach last destination
        stepper.run();
      }
      if (nextion_focus == 6) {
        in_position = stepper.currentPosition();                                                                         // set the IN point to the current stepper position
        NextionLCD.setComponentText("t6", String(in_position));                                                          // display position in t6 box on Nextion InPosition
        PanTiltINpoint = 0;
      }
      if (nextion_focus == 7) {
        out_position = stepper.currentPosition();                                                                        // set the OUT point to the current stepper position
        NextionLCD.setComponentText("t7", String(out_position));                                                          // display position in t7 box on Nextion InPosition
        PanTiltOUTpoint = 0;
      }
    }


    //********************Nextion PLAY Button Press******************

    if (nextion_message == "65 0 20 1 ffff ffff ffff") {    // Play Button action if not at atart
      digitalWrite(8, LOW);
      delay(20);
      if (crono_seconds == 0) {
        crono_seconds = 10;
        NextionLCD.setComponentText("t2", "10");   //set seconds at 10sec defalt to avoid false starts
      }

      ++PanTiltPlay;
      if (PanTiltPlay == 1) {
        SendPeramiters ();
        SetSpeed();
        TimerSet();
        Start_1();
      }
      if (PanTiltPlay == 2) {
        SendPeramiters ();
        if (stepper.currentPosition() == in_position)  { // Play Button ready to play
          SetSpeed();
          TimerSet();
          if (fps == 0) {
            digitalWrite(A0, HIGH);                      //Start camera
            delay (200);
            digitalWrite(A0, LOW);                      //Start camera
            Start_2();
          } else {
            Start_4();
          }
          PanTiltPlay = 1;
          SendPeramiters ();
        }
      }
      outpointPress = 0;
      inpointPress = 0;
    }





  }// close nextion listening if


  //********Move left Command *********
  if (move_left == 1) {  // Move left function move the slider to the left while holding the left arrow on Nextion


    if (stepper.currentPosition() <= 500) {
      while (stepper.currentPosition() >= 2) {
        stepper.moveTo(stepper.currentPosition() - 2); //If the stepper is near the Home or end stops moce slower to avoid overrun
        stepper.runSpeedToPosition();
      }
    }
    else {
      //stepper.setMaxSpeed(16000);
      // stepper.setAcceleration(1000);
      stepper.moveTo(stepper.currentPosition() - 300); //Otherwise move as fast as you like to the required position
      stepper.run();

    }

  }
  //******Move right command********
  if (move_right == 1) {                                  // Move left function move the slider to the left while holding the left arrow on Nextion


    if (stepper.currentPosition() > (Max_Steps - 500)) {         //do not let the carage hit the end of thye rail slow toward the end
      while (stepper.currentPosition() <= Max_Steps) {
        stepper.moveTo(stepper.currentPosition() + 2); //If the stepper is near the Home or end stops move slower to avoid overrun
        stepper.runSpeedToPosition();
      }
    }
    else {
      //stepper.setMaxSpeed(16000);
      //stepper.setAcceleration(1000);
      stepper.moveTo(stepper.currentPosition() + 300); //Otherwise move as fast as you like to the required position
      stepper.run();

    }

  }


}// END Void loop

//***********************************************************FUNCTIONS****************************************
//*********************************************************INpoint set*************************************
int INpointSet() {
  nextion_focus = 6;
  Buttonset = 1;
  digitalWrite(8, LOW);
  delay(10);
  if (inpointPress == 0) {  //First press


    if (stepper.currentPosition() != (in_position)) { //Run to in position
      stepper.setMaxSpeed(8000);
      stepper.setAcceleration(3000);
      stepper.moveTo(in_position);
      stepper.runToPosition();

    }

    encoder.setZero();
    output = (encoder.getPosition() );
    //NextionLCD.setComponentText("t4", String(output));
    encoder.setZero();
    output = (encoder.getPosition() );
    //NextionLCD.setComponentText("t4", String(output));

    digitalWrite(8, HIGH);
    delay(20);
    getEncoder = 1;
    inpointPress = 1;
    outpointPress = 0;
    return;

  } else {                    //Second inpoint press take the encoder values and set as inpoint


    NextionLCD.setComponentText("t6", String(S_position));
    stepper.setCurrentPosition((S_position));
    in_position = (S_position);
    //NextionLCD.setComponentText("t4", String(output / 2.56));
    digitalWrite(8, LOW);
    delay(20);
    S_position = 0;
    E_outputPos = 0;
    getEncoder = 0;
    inpointPress = 0;
    radio.startListening();
    return;
  }
}

//*********************************************************OUTpoint set*************************************
int OUTpointSet() {
  nextion_focus = 7;
  //NextionLCD.setComponentText("t4", String(outpointPress));
  Buttonset = 1;
  digitalWrite(8, LOW);
  delay(10);
  if (outpointPress == 0) {


    while (stepper.currentPosition() != (out_position)) { //Run to out position
      stepper.setMaxSpeed(8000);
      stepper.setAcceleration(3000);
      stepper.moveTo(out_position);
      stepper.runToPosition();

    }

    getEncoder = 2;
    outpointPress = 1;
    inpointPress = 0;
    PanTiltPlay = 0;

    digitalWrite(8, HIGH);      //Release the stepper for manual positioning
    encoder.setZero();
    output = (encoder.getPosition() );
    //NextionLCD.setComponentText("t4", String(output));
    encoder.setZero();
    output = (encoder.getPosition() );
    //NextionLCD.setComponentText("t4", String(output));
    delay(10);
    return;
  } else {                                                 //Second inpoint press take the encoder values and set as inpoint

    NextionLCD.setComponentText("t7", String(S_position));
    stepper.setCurrentPosition((S_position));
    out_position = (S_position);
    outpointPress = 0;
    digitalWrite(8, LOW);
    S_position = 0;
    E_outputPos = 0;
    getEncoder = 0;
    delay(20);
    Start_1();

    //NextionLCD.setComponentText("t4", String(output / 2.56));
    PanTiltPlay = 1;
    radio.startListening();
    return;
  }
}

//*********************************************ARM The SLIDER READY TO START**************************************************
int Start_1() {
  //  Arm the slider to start point ready for second play press
  digitalWrite(8, LOW);

  delay(100);

  while (stepper.currentPosition() != (in_position)) {
    stepper.setMaxSpeed(8000);
    stepper.setAcceleration(3000);                                  // speed that the head moves back to the start
    stepper.moveTo(in_position);                                    // get start position
    stepper.runToPosition();                                        // Move to start

  }

  NextionLCD.setComponentValue("TimerStop", 0);
  NextionLCD.setComponentValue("rewind", 0);

  inpointPress = 0;
  outpointPress = 0;
  radio.startListening();
  return;
}

//************************************************RUN THE SEQUENCE BASIC**************************************************************
int Start_2() {
  digitalWrite(8, LOW);

  stepper.setMaxSpeed(step_speed);
  stepper.setAcceleration(ease_Value);
  stepper.moveTo(out_position);
  stepper.runToPosition();

  if (Bounce >= 1) {
    //NextionLCD.setComponentValue("TimerStop",0);
    Start_3();

  } else  {                                              // Count down number of bounces required                                                          // Decrease by 1
    NextionLCD.setComponentValue("TimerStop", 0);
    Start_1();

  }
  inpointPress = 0;
  outpointPress = 0;

  radio.startListening();
  digitalWrite(A0, HIGH);                      //Stop camera
  delay (200);
  digitalWrite(A0, LOW);
  return;
}


//******************************************RUN THE SEQUENCE with Bounce************************************************
int Start_3() {

  digitalWrite(8, LOW);
  stepper.setMaxSpeed(step_speed);
  stepper.setAcceleration(ease_Value);
  stepper.moveTo(in_position);
  stepper.runToPosition();

  Bounce = Bounce - 1;
  NextionLCD.setComponentText("t4", String(Bounce));
  inpointPress = 0;
  outpointPress = 0;
  if (Bounce >= 1) {
    Start_2();
  } else {
    NextionLCD.setComponentValue("TimerStop", 0);

    Start_1();

  }
  radio.startListening();
  return;
}

//**********************************************RUN THE SEQUENCE Timelapse*******************************************
int Start_4() {

  TL_delay = (crono_seconds * 1000);                                //use the timer seconds as delay time for timelapse
  fps_countdown = fps;                                              //number of moves
  fps_step_dist = travel_dist / fps_countdown;

  digitalWrite(8, LOW);
  step_speed = (travel_dist / 5);
  stepper.setMaxSpeed(step_speed);
  stepper.setAcceleration(step_speed);
  radio.stopListening();
  while (fps_countdown >= 1) {

   
    unsigned char datatx = 4; //Trigger pantilt head timelapse move
    radio.write(&datatx, sizeof(unsigned char));
    //delay(10);
    


    stepper.moveTo(stepper.currentPosition() + fps_step_dist);           //Current position plus one fps move
    stepper.runToPosition();
    NextionLCD.setComponentText("t5", String(fps_countdown - 1));
    delay (1000);                                //Take a second to ensure camera is still
    digitalWrite(A0, HIGH);                      //Fire shutter pinA0 high
    delay (TL_delay);                            //Time the shutter is open for
    digitalWrite(A0, LOW);                       //Cose the shutter and move on
    delay (500);
    fps_countdown = fps_countdown - 1;
  }
  NextionLCD.setComponentValue("TimerStop", 0);
  NextionLCD.setComponentText("t5", String(fps));
  Start_1();                               //send the camera back to the start
  radio.startListening();
  return;
}
//*********************************************************NEXTION Timer set*****************************************
int TimerSet() {
  //  if (Buttonset == 1) {
  if (stepper.currentPosition() == in_position) {                           //if at start position start timer
    NextionLCD.setComponentValue("rewind", 0);
    NextionLCD.setComponentValue("TimerStop", 1); // timer ON
    Buttonset = 0;
    delay(20);
  } else {
    NextionLCD.setComponentValue("rewind", 1);                               // if not do not start timer rewind in progress
    NextionLCD.setComponentValue("TimerStop", 0); // timer off
    Buttonset = 0;
    delay(20);
  }

  //  }
  return;
}
//*********************************************** Speed & Distance formulas*******************************************************
int SetSpeed() {
  //******Set stepper speed based on time distance and amount od ease InOut***********
  travel_dist = (out_position - in_position);                           // Distance we have to go
  crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds); // Convert crono time to seconds this is the total time we want the sequence to run for in seconds
  step_speed = (travel_dist / (crono_time));

  switch (ease_InOut) {
    case 0:                   //No ease                                   // ease in/out values 1-10
      ease_Value = (step_speed);                                    //0 ease
      break;
    case 3:
      ease_Value = ((step_speed / 100) * 16);
      step_speed = (step_speed + ((step_speed / 100) * 30));
      break;
    case 2:
      ease_Value = ((step_speed / 100) * 20);
      step_speed = (step_speed + ((step_speed / 100) * 20));
      break;
    case 1:
      ease_Value = ((step_speed / 100) * 30);
      step_speed = (step_speed + ((step_speed / 100) * 10));
      break;

  }

  ease_time = ((step_speed - ease_Value) / ease_Value);    //Calculate the amount of extra time you need in seconds to allow for ease in out
  return;
}




//************************************WIFI SEND Peramiters to PanTilt*****************************************************
int SendPeramiters () {
  if (fps != 0) {
    TL_delay = crono_seconds * 1000;
  }

  WIFIOUT[0] = crono_time;
  WIFIOUT[1] = Bounce;
  WIFIOUT[2] = ease_InOut;
  WIFIOUT[3] = fps;
  WIFIOUT[4] = TL_delay;
  WIFIOUT[5] = PanTiltPlay;
  WIFIOUT[6] = PanTiltINpoint;
  WIFIOUT[7] = PanTiltOUTpoint;
  radio.stopListening();
  radio.write(WIFIOUT, sizeof(WIFIOUT));
  radio.startListening();
  delay(10);
  return;
}

int ErrorReset() {
NextionLCD.setComponentValue("TimerStop", 999);
return;
}

  
