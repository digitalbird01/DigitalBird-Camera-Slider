/*Digital Bird Mini Jib part of the Digital Bird Motion Control System Created by Colin Henderson 2023
  Version 6.00
  Requires you must be running V6 or above of the PTZplus WIFI controller firmware

  Update Notes v6.00: 23rd Oct 2023

  1) Now supports up to 4 WIFI camera systems rather than the original 3.
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



  Copyright  2023 Colin Henderson Digital Bird Motion Control

  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  Thx!

*/////////////////////////////////////////////////////////////////////////////////////////////////////////////



//#include <Boards.h>
#include <nvs_flash.h>
#include <FastAccelStepper.h>
#include "esp_task_wdt.h"
#include <AS5600.h>

#include <esp_now.h>
//#include <trigger.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET 4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT   10
#define LOGO_WIDTH    25

// 'Bat25', 25x10px
const unsigned char logo_bmp1 [] PROGMEM = {
  0x3f, 0xff, 0xff, 0x00, 0x40, 0x00, 0x00, 0x80, 0x40, 0x00, 0x1e, 0x80, 0xc0, 0x00, 0x1e, 0x80,
  0xc0, 0x00, 0x1e, 0x80, 0xc0, 0x00, 0x1e, 0x80, 0xc0, 0x00, 0x1e, 0x80, 0x40, 0x00, 0x1e, 0x80,
  0x40, 0x00, 0x00, 0x80, 0x3f, 0xff, 0xff, 0x00
};
// 'Bat50', 25x10px
const unsigned char logo_bmp2 [] PROGMEM = {
  0x3f, 0xff, 0xff, 0x00, 0x40, 0x00, 0x00, 0x80, 0x40, 0x03, 0xde, 0x80, 0xc0, 0x03, 0xde, 0x80,
  0xc0, 0x03, 0xde, 0x80, 0xc0, 0x03, 0xde, 0x80, 0xc0, 0x03, 0xde, 0x80, 0x40, 0x03, 0xde, 0x80,
  0x40, 0x00, 0x00, 0x80, 0x3f, 0xff, 0xff, 0x00
};
// 'Bat75', 25x10px
const unsigned char logo_bmp3 [] PROGMEM = {
  0x3f, 0xff, 0xff, 0x00, 0x40, 0x00, 0x00, 0x80, 0x40, 0x7b, 0xde, 0x80, 0xc0, 0x7b, 0xde, 0x80,
  0xc0, 0x7b, 0xde, 0x80, 0xc0, 0x7b, 0xde, 0x80, 0xc0, 0x7b, 0xde, 0x80, 0x40, 0x7b, 0xde, 0x80,
  0x40, 0x00, 0x00, 0x80, 0x3f, 0xff, 0xff, 0x00
};
// 'Bat100', 25x10px
const unsigned char logo_bmp4 [] PROGMEM = {
  0x3f, 0xff, 0xff, 0x00, 0x40, 0x00, 0x00, 0x80, 0x4f, 0x7b, 0xde, 0x80, 0xcf, 0x7b, 0xde, 0x80,
  0xcf, 0x7b, 0xde, 0x80, 0xcf, 0x7b, 0xde, 0x80, 0xcf, 0x7b, 0xde, 0x80, 0x4f, 0x7b, 0xde, 0x80,
  0x40, 0x00, 0x00, 0x80, 0x3f, 0xff, 0xff, 0x00
};

Preferences SysMemory;


#include "algorithm"

// Setup Stepper pins
#define step1_pinSTEP 26                      // Pin Gpio 26 connected to STEP pin of Stepper Driver
#define step1_pinDIR  27                      // Pin Gpio 27 connected to DIR pin of Stepper Driver
#define StepD 13                              // Pin GPio 13 Activate Deactivate stepper

#define Mount_PIN A12                         //Switch Gpio 2 to mount the head
#define DisMount_PIN A13                      //Switch Gpio 15 to dismount the head

const int BatPin = 39;                        //Battery monitoring pin
float BatV;


//Other pins
#define CAM A10                                //Pin Gpio 2/A10 Camera trigger
#define home_switch 15                         //Limit switch Gpio 15/A12


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;





//AMS AS5600 Encoder setup   The encoder keeps track of the camera position without having to have the stepper driven
AS5600 encoder;





//Variables

unsigned long BD = 1000;
int Snd;
int stopM_active;
int OLED_ID;
//****************************PTZ Variables*************************************//
int JBpressent;                               //Tell the system the Jib is pressent
int Joy_Tilt_Speed;                           //PTZ variables
int Joy_Tilt_Accel;                           //PTZ variables
int JTilt_is_moving;                           //PTZ variables
int Ez;
int usejoy;                                  //Joystick button state
int lastP1;                                  //Variables used if joystick command is active
int lastP2;
int lastP3;
int lastP4;
int lastP5;
int lastP6;
int lastInP;
int lastOutP;

int Rec;                                      //Record request 0 -1
int lastRecState;                             //Last Record button state for camera
int PTZ_Cam = 0;                              //Current PTZ camera defaulted to 0
int PTZ_Cam_Current = 0;                      //Place holder for PTZ_Cam
int PTZ_ID = 4;                               //PTZ Hardwae ID defaulted to 0
int lastPTZ_ID = PTZ_ID;                      //Stores the last known PTZ_ID
int PTZ_ID_Nex;                               //incomming ID request from nection control
int PTZ_Pose;                                 //Curent PTZ Pose key
int PTZ_SaveP;                                //PTZ save position comand true or false
int lastPTZ_Pose;                             //Store last PTZ_position for future save too
int X_position;                               //Holding variable for PTZ positions
int F_Stop;
int B_Stop;
int LM;                                       // Not used by the slider but here for referance
int stopM_play;                               //Stopmotion Trigger SM
int stopM_cancel;                             //Stopmmotion cancel SC
int SM;
int SC;
int stopM_frames;                             //The Total number of frames required for stopmotion
int SMC;                                      //stopmotion counter
int But;
int clrK;                                     //Clear all keys 1 or 0

int P1_S = 10;
int P2_S = 10;
int P3_S = 10;
int P4_S = 10;
int P5_S = 10;
int P6_S = 10;
int P7_S = 10;
int P8_S = 10;
int P9_S = 10;
int P10_S = 10;
int P11_S = 10;
int P12_S = 10;
int P13_S = 10;
int P14_S = 10;
int P15_S = 10;
int P16_S = 10;


int Mount = 0;                                //Is the Mount function active 1 true 0 false
long revolutions = 0;                         // number of revolutions the encoder has made
double E_position = 0;                        // the calculated value the encoder is at
double output;                                // raw value from AS5600
long lastOutput;                              // last output from AS5600
long E_outputPos = 0;                         // Ajustment value
long E_lastOutput;                            // last output from AS5600
long S_position = 0;                          // Number of stepper steps at 16 microsteps/step interpolated by the driver to 256 microsteps/step (confusing!)
long E_Trim = 0;                              // A Trim value for the encoder effectively setting its 0 position to match Step 0 position
long E_Current = 0;
int E_Turn = 0;
int E_outputTurn = 0;
long E_outputHold = 32728;
long loopcount = 0;
long  S_lastPosition = 0;
float factor;



long Max_Steps = 0;                         //Variable used to set the max number of steps for the given leangth of rail
long set_speed = 0;
long in_position = 0;                       // variable to hold IN position for turntable
long out_position = 0;                      // variable to hold OUT position for turntable

long step_speed = 0;                        // default travel speed between IN and OUT points
int Crono_time = 10;
int crono_hours = 0;
int crono_minutes = 0;
int crono_seconds = 0;
String DBFirmwareVersion = "frm V:6.00";
String nextion_message;                   // variable to hold received message for Nextion LCD
int move_left = 0;                        // variable to detect move left on Nextion LCD
int move_right = 0;                       // variable to confirm move right on Nextion LCD
int start_cycle = 0;                      // variable to confirm start of travel between IN and OUT points
long Home_position = 0;                   // Variable to st home position on turntable
int ease_InOut = 0;                       // Variable to collect Ease_InOut from nextion screen value range 1-3
int Ease_Value = 0;                       // Variable to hold actual Ease value used for acceleration calculation
int ease_time = 0;                        // Extra time allowed for ease in out calculation
int move_finished = 1;                    // Used to check if move is completed
long initial_homing = -1;                 // Used to Home Stepper at startup
int nextion_Func = 0;
int Bounce = 0;
int Tps = 0;
int Tps_countdown = 0;
int Tps_step_dist = 0;
int TpsD = 0;
int Buttonset = 0;
int timelapse_projected = 0;
int getEncoder = 0;
int InP = 0;
int OutP = 0;
int TiltPanComand = 0;
int playbuttoncounter = 1;
String dataTx;
String wifimessage;
int PanTiltPlay;
long PanTiltINpoint;
long PanTiltOUTpoint;
int Nextion_Func;
int Nextion_play;
int But_Com;
int TpsM;

int ResetEncoder = 0;
int Sld;                                             //Is the Sld  present 1,0,2
int TT;                                              //Is the turntable present 1 or 0
int PT;                                              //Is the Pan Tilt present 1,0,2
int JB = 1;                                          //Is the Jib present 1,0,2
//Sequencer
int SEQ = 0;
int K1A;                                             //Actual interpolated Accel values
int K2A;
int K3A;
int K4A;
int K5A;
int K6A;

int a1 = 0;                                          //Nextion values for Accel 1-3
int a2 = 0;
int a3 = 0;
int a4 = 0;
int a5 = 0;
int a6 = 0;

int P1;                                            //Nextion button state
int P2;
int P3;
int P4;
int P5;
int P6;

int s1 = 50;                                       //Nextion speed as a percentage
int s2 = 50;
int s3 = 50;
int s4 = 0;
int s5 = 0;
int s6 = 0;

int s1_speed;                                      //Actual interpolated speeds
int s2_speed;
int s3_speed;
int s4_speed;
int s5_speed;
int s6_speed;
int k1_position;
int k2_position;
int k3_position;
int k4_position;
int k5_position;
int k6_position;
int SEQmin = 4000;                                        //Min slowest speed for sequencer larger number slower speed
int  SEQmoves = 2;
int Last_key;

int KeyDist;
int KeyA;
int KeyB;

int k1_positionH;
int k2_positionH;
int k3_positionH;
int k4_positionH;
int k5_positionH;
int k6_positionH;

int BounceReturn = 1;                               //position memory for bounce 'H' for hold
int s1H;
int s2H;
int s3H;
int s4H;
int s5H;
int s6H;

int a1H;                                         //Nextion values for Accel 1-3
int a2H;
int a3H;
int a4H;
int a5H;
int a6H;

int K1AH;
int K2AH;
int K3AH;
int K4AH;
int K5AH;
int K6AH;

int LastMove;
int Last_accel;
int Last_speed;
int Base_accel = 0;

int move1_Dest;
int move2_Dest;
int move3_Dest;
int move4_Dest;
int move5_Dest;

int k1_k2_Dir;
int k2_k3_Dir;
int k3_k4_Dir;
int k4_k5_Dir;
int k5_k6_Dir;

int Stp_1_active;
int Stp_active = 1;
int Key_Crono_time;
int Seq_Time = 5;
int BounceActive;
int ActiveMove;
int Travel_dist;


int move2_Rst = 0;
int move3_Rst = 0;
int move4_Rst = 0;
int move5_Rst = 0;


//Structure to send WIFI data. Must match the receiver structure currently 136bytes of 250 max fo ESP-Now
typedef struct struct_message {

  int Snd;                                             //1= Controller 2=slider, 3=Pan Tilt 4=Turntable 5=Jib
  int But;                                             //But_Com Nextion button return value
  int Jb;                                              //Joystick button state true or false
  //Joystick controler peramiters
  int Ts;                                                 //Tilt speed
  int Ps;
  int Fs;
  int Zs;
  int Ss;
  int Ma;                                              //Master acceloration pot
  // PTZ peramiters
  int ID;                                             //Hardware ID for PTZ
  int CA;                                             //Camera selection for PTZ
  int SP;                                             //PTZ_SaveP PTZ save pose
  int Pz;                                             //PTZ_Pose PTZ current pose
  int Rc;                                             //Record button state for PTZ
  int LM;                                             //Focus/Zoom limits set
  // stopMotion peramiters
  int SM;                                             //stopMotion play true or false
  int SC;                                             //stopMotion cancel
  int SMC;                                            //Stop motion count

  int Ez;                                             //Ease Value
  int Bo;                                             //Number of bounces
  int TT;                                             //Is the turntable present 1 or 0
  int PT;                                             //Is the turntable present 1 or 0 2 PanTilt Bounce trigger prevents cumulative speed errors
  int Sld;                                            //Is the Slider  present 1 or 0
  int JB;                                             //Is the Jib  present 1 or 0
  long In;                                            //Slider In position in steps
  int P1;                                             //Key button preses
  int P2;
  int P3;
  int P4;
  int P5;
  int P6;
  int s1;                                                //Key Speeds
  int s2;
  int s3;
  int s4;
  int s5;
  int s6;
  //  int a1;                                                //Key Accelorations
  //  int a2;
  //  int a3;
  //  int a4;
  //  int a5;
  //  int a6;
  int Cro;                                             //Time in seconds of move
  //int Fnc;                                                //Nection Function
  int Tps;                                                //Timlapse frames required
  long Out;                                               //Slider Out position in steps

  int InP;                                                //Inpoint button press
  int clrK;                                               //Clear all keys 1 or 0
  int OutP;                                            //Out point button press
  int TpsD;                                               //Timlapse Delay in seconds
  int TpsM;                                               //Timlapse shutter time in seconds
  int Play;                                               //Play button press

  int IPR;                                                //Request for IP data 0-3
  int IP1;
  int IP2;
  int IP3;
  int IP4;
  int IPGW;
  int UDP;
  int TLY;
  unsigned long BD;
  //int mess;
} struct_message;

struct_message NextionValues;                             //Define structure for sending values to remote Nextion Control
struct_message incomingValues;                            //Define structure for receiving values from remote Nextion control

int WIFIOUT[8];                                           //set up 5 element array for sendinf values to pantilt
#define RXD2 16                                           //Hardware Serial2 on ESP32 Dev (must also be a common earth between nextion and esp32)
#define TXD2 17

TaskHandle_t C1;
TaskHandle_t C2;




void setup() {

  //*************************************Setup OLED****************************
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  //*************************************Setup a core to run Encoder****************************
  xTaskCreatePinnedToCore(coreas1signments, "Core_1", 10000, NULL, 2, &C1, 0);
  delay(100);
  disableCore1WDT();

  engine.init();                                                     //Startup the FastAccelStepper library
  stepper1 = engine.stepperConnectToPin(step1_pinSTEP);
  if (stepper1) {
    stepper1->setDirectionPin(step1_pinDIR);
    stepper1->setEnablePin(StepD);
    stepper1->setAutoEnable(false);
  }



  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  Wire.begin();
  Serial.println("\nRunning DigitalBird Turntable ESP32 V2.0\n");
  //Pin Settups

  pinMode(BatPin, INPUT);
  pinMode (CAM, OUTPUT);                                        //Camera Shutter
  pinMode(home_switch, INPUT_PULLUP);                           //Pin A1 pulled high for limit switches

  digitalWrite(StepD, LOW);                                     //Stepper Activation
  digitalWrite(CAM, LOW);                                       //Camera Shutter


  //Max_Steps = (((Rail_Length - 43) * 58) / 2);                  // Setup for the rail length

  //********************Setup Radio**********************************//
  WiFi.mode(WIFI_STA);

  Serial.println("\nmacAdress is:\n");
  Serial.print(WiFi.macAddress());
  WiFi.disconnect();

  //******************Setup ESP_now***************************
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Sucess");
    esp_now_register_recv_cb(receiveCallback);
    esp_now_register_send_cb(sentCallback);
  }
  else
  { Serial.println("ESPNow Init Failed");
    delay(3000);
    ESP.restart();
  }

  pinMode(Mount_PIN, INPUT_PULLUP);                             //pulled high Mount switch
  pinMode(DisMount_PIN, INPUT_PULLUP);                          //pulled high DisMount switch
  digitalWrite(CAM, LOW);                                       //Camera Shutter
  output = encoder.getPosition();
  lastOutput = output;
  E_position = output;


  //SysMemory Recover last setup
  SysMemory.begin("ID", false);                                 //Recover the last PTZ_ID from memory before shutdown
  PTZ_ID = SysMemory.getUInt("PTZ_ID", 4);
  SysMemory.end();
  PTZ_ID_Nex = 0;
  lastPTZ_ID = PTZ_ID;                                           //set last PTZ_ID for comparison when changing ID
  // Serial.printf("PTZ_ID is set to: %d \n", PTZ_ID);
  Load_SysMemory();
  SendNextionValues();
  homeStepper();
}



void loop() {

  if (PTZ_Cam != PTZ_Cam_Current) {       //If we have just moved in or out of the PTZ menu use the opertunity to update the controller regarding the presence of the jib
    PTZ_Cam_Current = PTZ_Cam;           //Note since all of the devices will get this at the same time there will be delays set for each device in order to give the controller a chance
    SendNextionValues();
  }

  //IF the Jib is present then make sure that the ease value is at least 1 for all moves.
  if (ease_InOut < 1) {
    Ease_Value = 1;
    ease_InOut = 1;
  }



  if ((PTZ_ID_Nex != 0) && (PTZ_ID_Nex != lastPTZ_ID)) {

    SetID();
    lastPTZ_ID = PTZ_ID;
  }


  if ((PTZ_Cam == PTZ_ID)  && LM == 0) {
    PTZ_Control();
  }

  if (usejoy == true ) {

    if (InP == 1) {
      PTZ_Control();
    }
    if (OutP == 1) {
      PTZ_Control();
    }
    if (P1 == 1) {
      PTZ_Control();
    }
    if (P2 == 1) {
      PTZ_Control();
    }
    if (P3 == 1) {
      PTZ_Control();
    }
    if (P4 == 1) {
      PTZ_Control();
    }
    if (P5 == 1) {
      PTZ_Control();
    }
    if (P6 == 1) {
      PTZ_Control();
    }
  }
  //*************************************Action Nextion Comands*****************************************
  if (InP != 0 && (InP != lastInP) && (PTZ_ID == 5)) {
    switch (InP) {
      case 1:                                                  //First Press: Request a new inpoint
        INpointSet();
        lastInP = InP;
        break;
      case 2:                                                  //Second Press: Record position
        INpointSet();
        //SendNextionValues();
        InP = 0;                                               //Reset Inpoint press back to 0
        lastInP = 0;
        break;
    }

  }
  if (OutP != 0 && (OutP != lastOutP) && (PTZ_ID == 5)) {

    switch (OutP) {
      case 1:                                                 //First Press: Request a new inpoint
        OUTpointSet();
        lastOutP = OutP;
        break;
      case 2:                                                 //Second Press: Record position
        OUTpointSet();
        //SendNextionValues();
        OutP = 0;                                            //Reset Inpoint press back to 0
        lastOutP = 0;
        break;
    }
  }


  if  (Nextion_play == 1 && (PTZ_ID == 5)) {                 //Play comand receved from menu 1
    if (stepper1->getCurrentPosition() != in_position)  {    // Play Button action if not at atart)
      digitalWrite(StepD, LOW);
      delay(20);
      SetSpeed();
      delay(20);
      Start_1();
    } else {                                                 //System is at the start and ready to play
      SetSpeed();
      delay(20);
      if (Tps == 0) {                                       //No timelapse
        digitalWrite(CAM, HIGH);                             //Start camera
        delay (200);
        digitalWrite(CAM, LOW);                              //Start camera
        Start_2();
      } else {                                               //Play Timelapse
        Travel_dist = (out_position - in_position);
        Tps_step_dist = (Travel_dist / Tps);
        Start_4();
      }
    }
    Nextion_play = 0;                                        //Reset play to 0

    SendNextionValues();
  }


  //*********StopMotion cancel****************
  if (stopM_cancel == 1) {
    SMC = Tps;
    stopM_play = 0;
    stopM_active = 0;
    Start_1();
  }
  //*********StopMotion Play comand****************
  if  (stopM_play == 1 && (PTZ_ID == 5)) {
    Start_7();
  }
  //**************************************Listen for PTZ Comands**********************************
  if ((PTZ_Pose != 0) && (PTZ_Cam == PTZ_ID)) {                    //If a position is requested and this devise is paired with the current PTZ camera
    lastPTZ_Pose = PTZ_Pose;                                     //Store last PTZ_position for future save to
    //Serial.printf("\nlastPTZ_Pose set to  %d \n", lastPTZ_Pose);
    delay(10);
    PTZ_MoveP();
    //  PTZ_Pose = 0;
  }
  if ((PTZ_SaveP != 0) && (PTZ_Cam == PTZ_ID)) {                   //If a position save is requested and this devise is paired with the current PTZ camera
    PTZ_Save();
    PTZ_SaveP = 0;
  }

  //**************************************Listen for Sequencer commands**********************************
  if (P1 != 0 && (P1 != lastP1) && (PTZ_ID == 5)) {
    switch (P1) {
      case 1:                                                  //First Press: Request a new position
        k1set();
        lastP1 = P1;
        break;
      case 2:                                                  //Second Press: Record position
        k1set();
        P1 = 0;                                               //Reset press back to 0
        lastP1 = 0;
        break;
    }
  }
  if (P2 != 0 && (P2 != lastP2) && (PTZ_ID == 5)) {
    switch (P2) {
      case 1:
        k2set();
        lastP2 = P2;
        break;
      case 2:
        k2set();
        P2 = 0;
        lastP2 = 0;
        break;
    }
  }
  if (P3 != 0 && (P3 != lastP3) && (PTZ_ID == 5)) {
    switch (P3) {
      case 1:
        k3set();
        lastP3 = P3;
        break;
      case 2:
        k3set();
        P3 = 0;
        lastP3 = 0;
        break;
    }
  }
  if (P4 != 0 && (P4 != lastP4) && (PTZ_ID == 5)) {
    switch (P4) {
      case 1:
        k4set();
        lastP4 = P4;
        break;
      case 2:
        k4set();
        P4 = 0;
        lastP4 = 0;
        break;
    }
  }
  if (P5 != 0 && (P5 != lastP5) && (PTZ_ID == 5)) {
    switch (P5) {
      case 1:
        k5set();
        lastP5 = P5;
        break;
      case 2:
        k5set();
        P5 = 0;
        lastP5 = 0;
        break;
    }
  }
  if (P6 != 0 && (P6 != lastP6) && (PTZ_ID == 5)) {
    switch (P6) {
      case 1:
        k6set();
        lastP6 = P6;
        break;
      case 2:
        k6set();
        P6 = 0;
        lastP6 = 0;
        break;
    }
  }

  if  (Nextion_play == 2 && (PTZ_ID == 5)) {                 //Sequencer Play comand receved
    if (stepper1->getCurrentPosition() != k1_position)  {  // Play Button action if not at atart)
      digitalWrite(StepD, LOW);
      delay(20);
      Start_1();                                             //Go to start
    } else {                                                 //System is at the start and ready to play
      delay(20);
      digitalWrite(CAM, HIGH);                               //Start camera
      delay (200);
      digitalWrite(CAM, LOW);                                //Start camera
      SEQ = 1;
      StorePositions();
      Start_5();
    }
    Nextion_play = 0;                                        //Reset play to 0
    SEQ = 0;
    SendNextionValues();
  }

  //*********StopMotion Cancel****************
  if (stopM_cancel == 1) {
    SMC = Tps;
    stopM_play = 0;
    stopM_active = 0;
    Start_1();
  }
  //*********StopMotion Play comand****************
  if  (stopM_play == 1 && (PTZ_ID == 5)) {
    Start_7();
  }

  //Clear keys command received
  if (clrK == 1) {
    ClearKeys();
    clrK = 0;
  }
} //end loop



void coreas1signments( void * pvParameters ) {
  for (;;) {
    StartEncoder();
    vTaskDelay(10);
    BatCheck();
  };
}

//**********************************Homing at Start**********************************

void homeStepper() {


  while (digitalRead(home_switch) == HIGH) {            // Make the Stepper move CCW until the switch is activated
    stepper1->setSpeedInHz(200);
    stepper1->setAcceleration(200);
    stepper1->moveTo(-3000);                             //"MoveTo" move to a position and stop,
  }
  if (digitalRead(home_switch) == LOW) {
    stepper1->forceStopAndNewPosition(5);                  //Stops dead

    delay(20);
    stepper1->move(800);
    while (stepper1->isRunning()) {                        //delay until move complete
      delay(10);
    }
    stepper1->setCurrentPosition(0);                       //Set this position as home 0
    ResetEncoder = 1;
    StartEncoder();
    vTaskDelay(10);
    SendNextionValues();
    //    stepper1->moveTo(in_position);
    //    while (stepper1->isRunning()) {                        //delay until move complete
    //      delay(10);
    //    }
    //    if (out_position < in_position) {
    //      out_position = out_position + 600;                   //In the event that the system is reovering from an error and the gantry for out is hard against the switch
    //
    //    }
  }
  return;
}

void StartEncoder() {

  if (ResetEncoder == 1) {
    encoder.setZero();
    lastOutput = 0;
    S_position = 0;
    E_position = 0;
    revolutions = 0;
    ResetEncoder = 0;
  }
  output = encoder.getPosition();           // get the raw value of the encoder

  if ((lastOutput - output) > 2047 )        // check if a full rotation has been made
    revolutions++;
  if ((lastOutput - output) < -2047 )
    revolutions--;

  E_position = revolutions * 4096 + output;   // calculate the position the the encoder is at based off of the number of revolutions

  //Serial.println("Encoder E_Position=");
  //Serial.println(E_position);

  lastOutput = output;                      // save the last raw value for the next loop
  E_outputPos = E_position;

  S_position = ((E_position / 2.56));               //Ajust encoder to stepper values the number of steps eqiv
  //Serial.println(S_position);
}



//*********************************************************INpoint set*************************************
void INpointSet() {
  // Buttonset = 1;
  //digitalWrite(StepD, LOW);
  delay(10);
  switch (InP) {
    case 1:                                                       //First Press: Request a new OUTpoint
      if (stepper1->getCurrentPosition() != (in_position)) {     //Run to Out position
        stepper1->setSpeedInHz(300);
        stepper1->setAcceleration(200);
        stepper1->moveTo(in_position);
        while (stepper1->isRunning()) {                          //delay until move complete
          delay(10);
        }
      }
      if (usejoy == false) {
        digitalWrite(StepD, HIGH);                               //Power down the stepper for manual positioning
        delay(10);
      }
      if (usejoy) {
        PTZ_Control();
      }
      break;

    case 2:                                                      //Second inpoint press take the encoder values and set as inpoint
      in_position = (S_position);
      stepper1->setCurrentPosition(in_position);                 //Make sure the stepper knows where it is know
      InP = 0;
      SendNextionValues();                                       //Update nextion with new inpoint values
      delay(10);
      SysMemory.begin("In_positions", false);                    //save to memory
      SysMemory.putUInt("in_position", in_position);
      SysMemory.end();

      Start_1();                                                 //Send the stepper back to In position
      break;
  }
  return;
}

//*********************************************************OUTpoint set*************************************
void OUTpointSet() {

  // Buttonset = 1;
  //digitalWrite(StepD, LOW);
  delay(10);
  switch (OutP) {
    case 1:                                                       //First Press: Request a new OUTpoint
      if (stepper1->getCurrentPosition() != (out_position)) {     //Run to Out position
        stepper1->setSpeedInHz(300);
        stepper1->setAcceleration(200);
        stepper1->moveTo(out_position);
        while (stepper1->isRunning()) {                          //delay until move complete
          delay(10);
        }
      }
      if (usejoy == false) {
        digitalWrite(StepD, HIGH);                                  //Power down the stepper for manual positioning
        delay(10);
      }
      if (usejoy) {
        PTZ_Control();
      }
      break;

    case 2:                                                       //Second inpoint press take the encoder values and set as inpoint
      out_position = (S_position);
      stepper1->setCurrentPosition(out_position);                 //Make sure the stepper knows where it is
      OutP = 0;
      delay(50);
      SendNextionValues();                                        //Update nextion with new inpoint values

      SysMemory.begin("Out_positions", false);                         //save to memory
      SysMemory.putUInt("out_position", out_position);
      SysMemory.end();
      Start_1();                                                  //Send the stepper back to In position
      break;
  }
  return;
}

//*********************************************************Move to pre recorded PTZ pose*************************************
void PTZ_MoveP() {
  lastPTZ_Pose = PTZ_Pose;
  delay(10);

  switch (lastPTZ_Pose) {
    case 1:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P1_S);
      }
      break;
    case 2:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P2_S);
      }
      break;
    case 3:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P3_S);
      }
      break;
    case 4:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P4_S);
      }
      break;
    case 5:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P5_S);
      }
      break;
    case 6:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P6_S);
      }
      break;
    case 7:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P7_S);
      }
      break;
    case 8:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P8_S);
      }
      break;
    case 9:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P9_S);
      }
      break;
    case 10:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P10_S);
      }
      break;
    case 11:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P11_S);
      }
      break;
    case 12:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P12_S);
      }
      break;
    case 13:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P13_S);
      }
      break;
    case 14:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P14_S);
      }
      break;
    case 15:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P15_S);
      }
      break;
    case 16:
      if (PTZ_Cam == PTZ_ID) {
        X_position = (P16_S);
      }
      break;
  }
  if (lastPTZ_Pose <= 14) {
    //if (stepper1->getCurrentPosition() != (X_position) && (X_position != 10)) {      //Run to  position
    stepper1->setSpeedInHz(300);
    stepper1->setAcceleration(300);
    stepper1->moveTo(X_position);
    while (stepper1->isRunning()) {                          //delay until move complete
      delay(10);
      //}

    }
    if (X_position == 10) {
      //Do nothing
    }
    PTZ_Pose = 0;
    return;
  }



  if (lastPTZ_Pose == 15) {      //A-B programed move
    if (stepper1->getCurrentPosition() != (in_position)  ) {
      digitalWrite(StepD, LOW);
      delay(20);
      SetSpeed();
      delay(20);
      Start_1();

      delay(20);

      Start_2();

    } else {                                                   //System is at the start and ready to play
      SetSpeed();                                              // This causes reboot?????????????
      delay(20);

      Start_2();
    }
    //PTZ_Cam = PTZ_ID;
  } else {

    if (lastPTZ_Pose == 16) {      //Sequencer programed move
      if (stepper1->getCurrentPosition() != k1_position )  {  // Play Button action if not at atart)
        digitalWrite(StepD, LOW);
        delay(20);
        Start_1();                                             //Go to start
        delay(20);

        SEQ = 1;
        StorePositions();
        Start_5();

        SEQ = 0;
        SendNextionValues();


      } else {                                                 //System is at the start and ready to play
        delay(20);

        SEQ = 1;
        StorePositions();
        Start_5();
        SEQ = 0;
        SendNextionValues();
        //PTZ_Cam = PTZ_ID;
      }

    }
  }
  PTZ_Cam = PTZ_Cam_Current;
}


void PTZ_Save() {

  switch (lastPTZ_Pose) {
    case 1:
      if (PTZ_Cam == PTZ_ID) {
        P1_S = (stepper1->getCurrentPosition());
        SysMemory.begin("P1pos", false);                         //save to memory
        SysMemory.putUInt("P1_S", P1_S);
        SysMemory.end();
      }
      break;
    case 2:
      if (PTZ_Cam == PTZ_ID) {
        P2_S = (stepper1->getCurrentPosition());
        SysMemory.begin("P2pos", false);                         //save to memory
        SysMemory.putUInt("P2_S", P2_S);
        SysMemory.end();
      }
      break;
    case 3:
      if (PTZ_Cam == PTZ_ID) {
        P3_S = (stepper1->getCurrentPosition());
        SysMemory.begin("P3pos", false);                         //save to memory
        SysMemory.putUInt("P3_S", P3_S);
        SysMemory.end();
      }
      break;
    case 4:
      if (PTZ_Cam == PTZ_ID) {
        P4_S = (stepper1->getCurrentPosition());
        SysMemory.begin("P4pos", false);                         //save to memory
        SysMemory.putUInt("P4_S", P4_S);
        SysMemory.end();
      }
      break;
    case 5:
      if (PTZ_Cam == PTZ_ID) {
        P5_S = (stepper1->getCurrentPosition());
        SysMemory.begin("P5pos", false);                         //save to memory
        SysMemory.putUInt("P5_S", P5_S);
        SysMemory.end();
      }
      break;
    case 6:
      if (PTZ_Cam == PTZ_ID) {
        P6_S = (stepper1->getCurrentPosition());
        SysMemory.begin("P6pos", false);                         //save to memory
        SysMemory.putUInt("P6_S", P6_S);
        SysMemory.end();
      }
      break;
    case 7:
      if (PTZ_Cam == PTZ_ID) {
        P7_S = (stepper1->getCurrentPosition());
        SysMemory.begin("P7pos", false);                         //save to memory
        SysMemory.putUInt("P7_S", P7_S);
        SysMemory.end();
      }
      break;
    case 8:
      if (PTZ_Cam == PTZ_ID) {
        P8_S = (stepper1->getCurrentPosition());
        SysMemory.begin("P8pos", false);                         //save to memory
        SysMemory.putUInt("P8_S", P8_S);
        SysMemory.end();
      }
      break;
    case 9:
      if (PTZ_Cam == PTZ_ID) {
        P9_S = (stepper1->getCurrentPosition());
        SysMemory.begin("P9pos", false);                         //save to memory
        SysMemory.putUInt("P9_S", P9_S);
        SysMemory.end();
      }
      break;
  }
}
//*********************************************************K1set*************************************
void k1set() {

  delay(10);
  switch (P1) {
    case 1:                                                      //First Press: Request a new position
      if (stepper1->getCurrentPosition() != (k1_position)) {    //Run to  position
        stepper1->setSpeedInHz(300);
        stepper1->setAcceleration(200);
        stepper1->moveTo(k1_position);
        while (stepper1->isRunning()) {                          //delay until move complete
          delay(10);
        }
      }
      if (usejoy == false) {
        digitalWrite(StepD, HIGH);                                  //Power down the stepper for manual positioning
      }
      delay(10);
      break;

    case 2:                                                      //Second press take the encoder values and set as new position
      k1_position = (S_position);

      SysMemory.begin("K1_pos", false);                               //save to memory
      SysMemory.putUInt("k1_pos", k1_position);
      SysMemory.end();

      stepper1->setCurrentPosition(k1_position);                 //Make sure the stepper knows where it is know
      P1 = 0;
      SendNextionValues();                                       //Update nextion with new values
      digitalWrite(StepD, LOW);                                 //Power UP the stepper
      delay(10);

      break;
  }
  return;
}
//*********************************************************K2set*************************************
void k2set() {

  delay(10);
  switch (P2) {
    case 1:
      if (stepper1->getCurrentPosition() != (k2_position)) {
        stepper1->setSpeedInHz(300);
        stepper1->setAcceleration(200);
        stepper1->moveTo(k2_position);
        while (stepper1->isRunning()) {
          delay(10);
        }
      }
      if (usejoy == false) {
        digitalWrite(StepD, HIGH);                                  //Power down the stepper for manual positioning
      }
      delay(10);
      break;

    case 2:
      k2_position = (S_position);

      SysMemory.begin("K2_pos", false);                               //save to memory
      SysMemory.putUInt("k2_pos", k2_position);
      SysMemory.end();

      stepper1->setCurrentPosition(k2_position);
      P2 = 0;
      SendNextionValues();
      delay(10);
      digitalWrite(StepD, LOW);                                 //Power UP the stepper
      break;
  }
  return;
}
//*********************************************************K3set*************************************
void k3set() {

  delay(10);
  switch (P3) {
    case 1:
      if (stepper1->getCurrentPosition() != (k3_position)) {
        if (k3_position == 0) {
        } else {
          stepper1->setSpeedInHz(300);
          stepper1->setAcceleration(200);
          stepper1->moveTo(k3_position);
          while (stepper1->isRunning()) {
            delay(10);
          }
        }
      }
      delay(50);
      if (usejoy == false) {
        digitalWrite(StepD, HIGH);                                  //Power down the stepper for manual positioning
      }
      P3 = 0;
      break;

    case 2:
      k3_position = (S_position);

      SysMemory.begin("K3_pos", false);                               //save to memory
      SysMemory.putUInt("k3_pos", k3_position);
      SysMemory.end();

      stepper1->setCurrentPosition(k3_position);
      P3 = 0;
      digitalWrite(StepD, LOW);                                 //Power UP the stepper
      delay(100);
      SendNextionValues();
      break;
  }
  return;
}
//*********************************************************K4set*************************************
void k4set() {

  delay(10);
  switch (P4) {
    case 1:
      if (stepper1->getCurrentPosition() != (k4_position)) {
        if (k3_position == 0) {

        } else {
          stepper1->setSpeedInHz(300);
          stepper1->setAcceleration(200);
          stepper1->moveTo(k4_position);
          while (stepper1->isRunning()) {
            delay(10);
          }
        }
      }
      delay(50);
      if (usejoy == false) {
        digitalWrite(StepD, HIGH);                                  //Power down the stepper for manual positioning
      }
      P4 = 0;
      break;

    case 2:
      k4_position = (S_position);

      SysMemory.begin("K4_pos", false);                               //save to memory
      SysMemory.putUInt("k4_pos", k4_position);
      SysMemory.end();

      stepper1->setCurrentPosition(k4_position);
      P4 = 0;
      digitalWrite(StepD, LOW);                                 //Power UP the stepper
      delay(100);
      SendNextionValues();
      break;
  }
  return;
}
//*********************************************************K5set*************************************
void k5set() {


  delay(10);
  switch (P5) {
    case 1:
      if (stepper1->getCurrentPosition() != (k5_position)) {
        if (k5_position == 0) {

        } else {
          stepper1->setSpeedInHz(300);
          stepper1->setAcceleration(200);
          stepper1->moveTo(k5_position);
          while (stepper1->isRunning()) {
            delay(10);
          }
        }
      }
      delay(50);
      if (usejoy == false) {
        digitalWrite(StepD, HIGH);                                  //Power down the stepper for manual positioning
      }
      P5 = 0;
      break;

    case 2:
      k5_position = (S_position);

      SysMemory.begin("K5_pos", false);                               //save to memory
      SysMemory.putUInt("k5_pos", k5_position);
      SysMemory.end();

      stepper1->setCurrentPosition(k5_position);
      P5 = 0;
      digitalWrite(StepD, LOW);                                 //Power UP the stepper
      delay(100);
      SendNextionValues();
      break;
  }
  return;
}
//*********************************************************K6set*************************************
void k6set() {

  delay(10);
  switch (P6) {
    case 1:
      if (stepper1->getCurrentPosition() != (k6_position)) {
        if (k6_position == 0) {

        } else {
          stepper1->setSpeedInHz(300);
          stepper1->setAcceleration(200);
          stepper1->moveTo(k6_position);
          while (stepper1->isRunning()) {
            delay(10);
          }
        }
      }
      delay(50);
      if (usejoy == false) {
        digitalWrite(StepD, HIGH);                                  //Power down the stepper for manual positioning
      }
      P6 = 0;
      break;

    case 2:
      k6_position = (S_position);

      SysMemory.begin("K6_pos", false);                               //save to memory
      SysMemory.putUInt("k6_pos", k6_position);
      SysMemory.end();

      stepper1->setCurrentPosition(k6_position);
      P6 = 0;
      digitalWrite(StepD, LOW);                                 //Power UP the stepper
      delay(100);
      SendNextionValues();
      break;
  }
  return;
}



// ********************************Mac Adress handeling ******************************
void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength) {
  snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}
void receiveCallback(const uint8_t *macAddr, const uint8_t *incomingData,  int Len) {
  // only allow a maximum of 250 characters in the message + a null terminating byte
  char buffer[ESP_NOW_MAX_DATA_LEN + 1];
  // int msgLen = min(ESP_NOW_MAX_DATA_LEN, dataLen);

  memcpy(&incomingValues, incomingData, sizeof(incomingValues));
  //memcpy(&incomingValues, Data, sizeof(incomingValues));

  //Read all current Nextion values
  //  a1 = int(incomingValues.a1);
  //  a2 = int(incomingValues.a2);
  //  a3 = int(incomingValues.a3);
  //  a4 = int(incomingValues.a4);
  //  a5 = int(incomingValues.a5);
  //  a6 = int(incomingValues.a6);
  P1 = int(incomingValues.P1);
  P2 = int(incomingValues.P2);
  P3 = int(incomingValues.P3);
  P4 = int(incomingValues.P4);
  P5 = int(incomingValues.P5);
  P6 = int(incomingValues.P6);
  s1 = int(incomingValues.s1);                   //Sequencer keys % speed

  s1_speed = (float(100 - s1) / 100) * SEQmin ;
  s2 = int(incomingValues.s2);
  s2_speed = (float(100 - s2) / 100) * SEQmin ;
  s3 = int(incomingValues.s3);
  s3_speed = (float(100 - s3) / 100) * SEQmin ;
  s4 = int(incomingValues.s4);
  s4_speed = (float(100 - s4) / 100) * SEQmin ;
  s5 = int(incomingValues.s5);
  s5_speed = (float(100 - s5) / 100) * SEQmin ;
  s6 = int(incomingValues.s6);
  s6_speed = (float(100 - s6) / 100) * SEQmin ;


  Snd = int(incomingValues.Snd);                //Senders Ident 1=Controller 2=Slider 3=PanTilt 4=TurnT 5=Jib

  switch (Snd) {                                //Only Change the Device state if the message was sent by the device
    case 1:
      Sld = int(incomingValues.Sld);            //Slider available 1 or 0
      TT = int(incomingValues.TT);               //Turntable available 1 or 0
      PT = int(incomingValues.PT);               //Pan Tilt available 1 or 0
      break;
    case 2:
      Sld = int(incomingValues.Sld);            //Slider available 1 or 0
      break;
    case 3:
      PT = int(incomingValues.PT);               //Pan Tilt available 1 or 0
      break;
    case 4:
      TT = int(incomingValues.TT);               //Turntable available 1 or 0
      break;
  }

  ease_InOut = int(incomingValues.Ez);          //Acceleration control
  Bounce = int(incomingValues.Bo);              //Number of bounces
  Crono_time = int(incomingValues.Cro);         //Time in sec of system move
  //Nextion_Func = int(incomingValues.Fnc);     //Current Func of nextion display
  Tps = int(incomingValues.Tps);                //Timlapse frames
  TpsD = int(incomingValues.TpsD);              //Timplapse delay or shutter open time in Bulb mode
  TpsM = int(incomingValues.TpsM);              //Timelapse move comand used to trigger next move from nextion
  //in_position = long(incomingValues.In);      //The slider is the originator so referance only
  //out_position = long(incomingValues.Out);    //The slider is the originator so referance only
  Nextion_play = int(incomingValues.Play);      //playbutton pressed on nextion
  But_Com = int(incomingValues.But);            //playbutton pressed on Slave
  InP = int(incomingValues.InP);                //Inpoint button state 0-2
  OutP = int(incomingValues.OutP);              //Outpoint button state 0-2

  Joy_Tilt_Speed = int(incomingValues.Zs);      //Joystick Controler speed
  Joy_Tilt_Accel = int(incomingValues.Ma);      //Joystick Controler Acceloration

  PTZ_ID_Nex = int(incomingValues.ID);           //PTZ messagefor ID
  PTZ_Cam = int(incomingValues.CA);              //Current PTZ camera
  PTZ_SaveP = int(incomingValues.SP);            //PTZ save position 1 or 0

  PTZ_Pose = int(incomingValues.Pz);             //Current PTZ pose 1-6
  usejoy = int(incomingValues.Jb);               //Joystick button state
  stopM_play = int(incomingValues.SM);           //Stopmotion Trigger SM
  stopM_frames = int(incomingValues.Tps);
  stopM_cancel = int(incomingValues.SC);         //Stopmmotion cancel SC
  Rec = int(incomingValues.Rc);                  //Record button state for PTZ
  //if (Snd = 1) {
    BD = long(incomingValues.BD);                     //Return/bounce delay
    BD = (BD * 1000);
  //}
  clrK = int(incomingValues.clrK);               //Command to clear all keys 1 or 0
  char macStr[18];
  formatMacAddress(macAddr, macStr, 18);
  // debug log the message to the serial port
  //Serial.printf("Received message from: %s - %s\n", macStr, buffer);
  return;

}


void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status) // callback when data is sent
{
  char macStr[18];
  formatMacAddress(macAddr, macStr, 18);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


//Setup ESP_NOW connection
void broadcast(const String & message)
{
  // this will broadcast a message to everyone in range
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t peerInfo = {};
  memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
  if (!esp_now_is_peer_exist(broadcastAddress))
  {
    esp_now_add_peer(&peerInfo);
  }
  esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)message.c_str(), message.length());

  // and this will send a message to a specific device
  /*uint8_t peerAddress[] = {0x3C, 0x71, 0xBF, 0x47, 0xA5, 0xC0};
    esp_now_peer_info_t peerInfo = {};
    memcpy(&peerInfo.peer_addr, peerAddress, 6);
    if (!esp_now_is_peer_exist(peerAddress))
    {
    esp_now_add_peer(&peerInfo);
    }
    esp_err_t result = esp_now_send(peerAddress, (const uint8_t *)message.c_str(), message.length());*/
  if (result == ESP_OK)
  {
    Serial.println("Broadcast message success");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_INIT)
  {
    Serial.println("ESPNOW not Init.");
  }
  else if (result == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Invalid Argument");
  }
  else if (result == ESP_ERR_ESPNOW_INTERNAL)
  {
    Serial.println("Internal Error");
  }
  else if (result == ESP_ERR_ESPNOW_NO_MEM)
  {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.println("Peer not found.");
  }
  else
  {
    Serial.println("Unknown error");
  }
}

void SendNextionValues() {                  //Remember any value you fail to send will register as 0 at the reciving systems!!!!
  broadcast ("");
  NextionValues.Snd = 5;                    //Tells the controller who sent the message 1=Controller 2=Slider 3=PanTilt 4=Turntable 5=Jib
  NextionValues.Ez = ease_InOut;            // Acceloration control
  NextionValues.Bo = Bounce;                // Number of bounces
  NextionValues.Cro = Crono_time;           // Time in sec of system move
  //NextionValues.Fnc = Nextion_Func;       // Current Func of nextion display
  NextionValues.Tps = Tps;                  // Timlapse frames
  //NextionValues.TpsD = TpsD;              // Timplapse delay or shutter open time in Bulb mode
  //NextionValues.TpsM = TpsM;              // Timelapse move command
  NextionValues.In = in_position;           // InPoint step position value
  NextionValues.Out = out_position;         // OutPoint Step position value
  NextionValues.Play = Nextion_play;        // play comand
  NextionValues.But = But_Com;              // Button reply
  NextionValues.InP = InP;                  // Inpoint button state 0-2
  NextionValues.OutP = OutP;                // Outpoint button state 0-2
  // NextionValues.Sld = Sld;               // Sld present 0-1
  //NextionValues.PT = PT;                  // Pan Tilt present 0-1,2
  //NextionValues.TT = TT;                    // Turntable present 0-1
  NextionValues.CA = PTZ_Cam;               //Send back confirmation regarding current PTZ
  NextionValues.JB = JB;                    // Turntable present 0-1
  NextionValues.s1 = s1;                    // Sequencer key speed
  NextionValues.s2 = s2;                    // Sequencer key speed
  NextionValues.s3 = s3;                    // Sequencer key speed
  NextionValues.s4 = s4;                    // Sequencer key speed
  NextionValues.s5 = s5;                    // Sequencer key speed
  NextionValues.s6 = s6;                    // Sequencer key speed
  NextionValues.P1 = P1;                    // k1 set button 0-2
  NextionValues.P2 = P2;                    // k2 set button 0-2
  NextionValues.P3 = P3;                    // k3 set button 0-2
  NextionValues.P4 = P4;                    // k4 set button 0-2
  NextionValues.P5 = P5;                    // k5 set button 0-2
  NextionValues.P6 = P6;                    // k6 set button 0-2
  //  NextionValues.a1 = a1;                    // Acceloration values
  //  NextionValues.a2 = a2;
  //  NextionValues.a3 = a3;
  //  NextionValues.a4 = a4;
  //  NextionValues.a5 = a5;
  //  NextionValues.a6 = a6;
  NextionValues.SMC = SMC;                  //Stop motion counter

  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(broadcastAddress, (uint8_t *) &NextionValues, sizeof(NextionValues));
}




//*********************************************ARM The JIB READY TO START**************************************************
void Start_1() {
  //  Arm the turntable to start point ready for second play press
  digitalWrite(StepD, LOW);
  delay(100);

  stepper1->setSpeedInHz(300);
  stepper1->setAcceleration(200);

  if (SEQ == 0) {
    stepper1->moveTo(in_position);
    delay(10);
  }
  if (SEQ == 1) {
    stepper1->moveTo(k1_position);
    delay(10);

  }
  while (stepper1->isRunning() ) {                              //delay until move complete (block)
    if (digitalRead(home_switch) == LOW) {
      stepper1->forceStopAndNewPosition(5);                  //Stops dead error if the home switch is triggered unexpectadly
      ESP.restart();
    }
    delay(10);
  }

  But_Com = 5;
  if (PTZ_Cam == 0) {
    SendNextionValues();
    delay(100);
  }
  But_Com = 0;
}

//************************************************RUN THE SEQUENCE BASIC**************************************************************
void Start_2() {
 long Pause;
 Pause = BD;
 
  if (Bounce >= 1 && PT == 0) {                       //If no PT then take charge of the bounce moves
    delay(Pause);
    JB = 2;                                          //Tell the other devices to start move
    But_Com = 4;
    SendNextionValues();
    delay(10);
    JB = 1;
    But_Com = 0;
  }

  if (Bounce >= 1 && PT != 0) {                         //If the PT is present give it control of the bounce moves
    while (PT != 2) {
      delay(10);
    }
    PT = 1;
  }
  stepper1->setSpeedInHz(step_speed);
  stepper1->setAcceleration(Ease_Value);
  delay(2);                                               //Timed to start with PanTilt head
  stepper1->moveTo(out_position);
  while (stepper1->isRunning()) {                         //delay until move complete (block)
    if (digitalRead(home_switch) == LOW) {
      stepper1->forceStopAndNewPosition(5);                  //Stops dead error if the home switch is triggered unexpectadly
      ESP.restart();
    }
    delay(2);
  }
  //******************************************Bounce Return************************************************
  //*************************Formally start_3 which ran into watchdog problems looping between two functions********************
  if (Bounce >= 1 ) {

    if (PT == 0) {                                          //If PT not pressent take charge of bounce control
      delay(Pause);
      JB = 2;                                                //Tell the other devices to start there move
      SendNextionValues();
      delay(10);
      JB = 1;
    }
    if (PT != 0) {                                           //If PT is present give it control of the bounce moves
      while (PT != 2) {
        delay(10);
      }
      PT = 1;
    }

    stepper1->moveTo(in_position);
    while (stepper1->isRunning()) {                         //delay until move complete (block)

      if (digitalRead(home_switch) == LOW) {
        stepper1->forceStopAndNewPosition(5);                  //Stops dead error if the home switch is triggered unexpectadly
        ESP.restart();
      }

      delay(2);
    }

    if (PT == 0) {                                          //Only take charge of the counting if no PT
      Bounce = Bounce - 1;
      But_Com = 6;                                          //Send current bounce counter back to nextion
      SendNextionValues();
      delay(10);
      But_Com = 0;
    }

    if (Bounce > 0) {                                         //If Bounce is not finished go back to Start_2
      delay(Pause);
      Start_2();
    } else {
      if (PTZ_ID == 5) {
        digitalWrite(CAM, HIGH);                                //Stop camera and go back to start_1
        delay (200);
        digitalWrite(CAM, LOW);
      }
      if (PT == 0) {
        if (PTZ_ID == 5) {
          But_Com = 5;                                           //Tell the nextion to stop the timer
        }
        JB = 1;
        SendNextionValues();
        delay(10);
        But_Com = 0;
      }
      delay(Pause);
      Start_1();
    }
    //**************end of Bounce section***************


  } else  {                                               
    delay(Pause);
    if (PTZ_ID == 5) {
      digitalWrite(CAM, HIGH);                              //Stop camera
      delay (200);
      digitalWrite(CAM, LOW);
    }
    if (PT == 0) {
      if (PTZ_ID == 5) {
      But_Com = 5;                                        //Tell the nextion to stop the timer
      }
      JB = 1;
      SendNextionValues();
      delay(10);
      But_Com = 0;
    }
    
    Start_1();
  }
}



//**********************************************Timelapse*******************************************
void Start_4() {

  digitalWrite(StepD, LOW);
  step_speed = (Travel_dist / 5);
  stepper1->setSpeedInHz(100);
  stepper1->setAcceleration(50);

  while (Tps >= 1) {

    //The jib is normally the slowest moving so give it control over timelapse

    //Trigger pantilt head timelapse move
    SendNextionValues();                                                 //Trigger pantilt head timelapse move
    stepper1->moveTo(stepper1->getCurrentPosition() + Tps_step_dist);    //Current position plus one fps move


    if (Tps != 0) {
      Tps = Tps - 1;
    }
    But_Com = 7;                                                         //Tell the nextion to update the frame counter
    SendNextionValues();
    But_Com = 0;
    while (stepper1->isRunning()) {                                      //delay until move complete (blocking)
      delay(20);
    }
    JB = 2;                                                             // Tell everything the jib is ready to take the shot
    SendNextionValues();
    delay (1000);                                                        //Take a second to ensure camera is still longer for the jib
    digitalWrite(CAM, HIGH);                                             //Fire shutter
    if (TpsD <= 1000) {
      TpsD = 1000;
    }
    delay(TpsD);                                                         //Time the shutter is open for
    digitalWrite(CAM, LOW);                                              //Cose the shutter and move on
    JB = 1;
    SendNextionValues();
    delay (500);

  }
  But_Com = 5;                                                             //Tell the nextion to stop the timer
  SendNextionValues();
  But_Com = 0;

  Start_1();                                                               //send the camera back to the start
}

void Start_7() {
  //If no slider or Jib take charge of the count
  if (stopM_active = 1) {                                               // Is stopM active?
    SMC = SMC - 1;                                                      //Subtract frame
  } else {
    SMC = Tps;
    SMC = SMC - 1;
  }
  But_Com = 8;
  SendNextionValues();                                                  //Tell the nextion to update the stopmotion frame counter

  Travel_dist = (out_position - in_position);
  Tps_step_dist = (Travel_dist / stopM_frames);                           //The Total number of frames

  digitalWrite(StepD, LOW);
  step_speed = (Travel_dist / 5);
  stepper1->setSpeedInHz(150);
  stepper1->setAcceleration(50);

  if (SMC >= 0  && stopM_cancel != 1) {
    JB = 2;                                                           //Trigger pantilt head timelapse move
    SendNextionValues();
    stepper1->moveTo(stepper1->getCurrentPosition() + Tps_step_dist);     //Current position plus one fps move
    while (stepper1->isRunning()) {                                       //delay until move complete (blocking)
      delay(10);
    }

    delay (1000);                                                       //Take a second to ensure camera is still

    digitalWrite(CAM, HIGH);                                              //Fire shutter
    if (TpsD <= 1000) {
      TpsD = 1000;
    }                                                                    //Time the shutter is open for
    delay(TpsD);                                                         //Time the shutter is open for if on ball. If not on ball camera has control of shutter time and user must set this higher than shutter speed
    digitalWrite(CAM, LOW);                                               //Cose the shutter and move on
    //delay (500);
    JB = 1;
    But_Com = 5;
    SendNextionValues();
    But_Com = 0;
  }
  stopM_play = 0;

  if (SMC == 0) {                                                                  // If the Stopmotion has finished or received a cancel. send the camera back to the start                                                                      // No longer need the total number of frames so 0
    SMC = Tps;
    stopM_active = 0;                                                   // StopM finished inactive
    Start_1();
  }
}
//**********************************************Sequencer*******************************************
void Find_KeyDist() {
  if (KeyB >= 0 && KeyA >= 0) {
    KeyDist = abs(KeyB) - abs(KeyA);
  }
  if (KeyB <= 0 && KeyA <= 0) {
    KeyDist = abs(KeyB) - abs(KeyA);
  }

  if (KeyB > 0 && KeyA <= 0) {
    KeyDist = abs(KeyB) + abs(KeyA);
  }
  if (KeyB < 0 && KeyA > 0) {
    KeyDist = abs(KeyB) + abs(KeyA);
  }
  return;
}
void Set_forward_values() {
  //Set forward values before start of next bounce move

  k1_position = k1_positionH;
  k2_position = k2_positionH;
  k3_position = k3_positionH;
  k4_position = k4_positionH;
  k5_position = k5_positionH;
  k6_position = k6_positionH;

  s1 = s1H;
  s2 = s2H;
  s3 = s3H;
  s4 = s4H;
  s5 = s5H;
  s6 = s6H;

  K1A = K1AH;
  K2A = K2AH;
  K3A = K3AH;
  K4A = K4AH;
  K5A = K5AH;
  K6A = K6AH;
}
void Set_reverse_values() {
  switch (LastMove) {
    case 3:                                                                                   //No ease
      k1_position = k3_positionH;
      k2_position = k2_positionH;
      k3_position = k1_positionH;

      s1 = s3H;
      s2 = s2H;
      s3 = s1H;

      K1A = K3AH;
      K2A = K2AH;
      K3A = K1AH;
      break;
    case 4:                                                                                   //No ease
      k1_position = k4_positionH;
      k2_position = k3_positionH;
      k3_position = k2_positionH;
      k4_position = k1_positionH;

      s1 = s4H;
      s2 = s3H;
      s3 = s2H;
      s4 = s1H;

      K1A = K4AH;
      K2A = K3AH;
      K3A = K2AH;
      K4A = K1AH;
      break;

    case 5:                                                                                   //No ease

      k1_position = k5_positionH;
      k2_position = k4_positionH;
      k3_position = k3_positionH;
      k4_position = k2_positionH;
      k5_position = k1_positionH;


      s1 = s5H;
      s2 = s4H;
      s3 = s3H;
      s4 = s2H;
      s5 = s1H;

      K1A = K5AH;
      K2A = K4AH;
      K3A = K3AH;
      K4A = K2AH;
      K5A = K1AH;
      break;
    case 6:
      k1_position = k6_positionH;
      k2_position = k5_positionH;
      k3_position = k4_positionH;
      k4_position = k3_positionH;
      k5_position = k2_positionH;
      k6_position = k1_positionH;

      s1 = s6H;
      s2 = s5H;
      s3 = s4H;
      s4 = s3H;
      s5 = s2H;
      s6 = s1H;

      K1A = K6AH;
      K2A = K5AH;
      K3A = K4AH;
      K4A = K3AH;
      K5A = K2AH;
      K6A = K1AH;
      break;
  }
}
void StorePositions() {
  //Save original forward values in holding variables ready for Bounce

  k1_positionH = k1_position;
  k2_positionH = k2_position;
  k3_positionH = k3_position;
  k4_positionH = k4_position;
  k5_positionH = k5_position;
  k6_positionH = k6_position;

  s1H = s1;
  s2H = s2;
  s3H = s3;
  s4H = s4;
  s5H = s5;
  s6H = s6;

  K1AH = K1A;
  K2AH = K2A;
  K3AH = K3A;
  K4AH = K4A;
  K5AH = K5A;
  K6AH = K6A;
}



void Start_5() {
  unsigned long Pause = BD;

  int PTZ_IDhld = (PTZ_ID);

  //Establish how many keys are set
  if (s1 != 0) {
    LastMove = 1;
  }
  if (s2 != 0) {
    LastMove = 2;
  }
  if (s3 != 0) {
    LastMove = 3;
  }
  if (s4 != 0) {
    LastMove = 4;
  }
  if (s5 != 0) {
    LastMove = 5;
  }
  if (s6 != 0) {
    LastMove = 6;
  }
  if (BounceReturn == 1) {                       //If Bounce move is forward

    Set_forward_values();                         //Set forward values before start of next bounce move
  }

  if (BounceReturn == 2) {                       //Reverse all the values
    Set_reverse_values();
  }

  But_Com = 4;
  SendNextionValues();
  But_Com = 0;
  delay(20);
  int StartTime;

  Last_accel = 0;
  Last_speed = 0;


  Base_accel = 0;


  move1_Dest = 0;
  move2_Dest = 0;
  move3_Dest = 0;
  move4_Dest = 0;
  move5_Dest = 0;

  s1_speed = 0;
  s2_speed = 0;
  s3_speed = 0;
  s4_speed = 0;
  s5_speed = 0;
  s6_speed = 0;

  int AccelLock;

  Stp_1_active = 0;

  Stp_active = 0;
  float Ramp_Time = 0;






  stepper1_plan();
  delay(10);

  //  Serial.print("move1_Dest:   ");
  //  Serial.print(move1_Dest);

  //  Serial.print("move2_Dest:   ");
  //  Serial.print(move2_Dest);

  //  Serial.println("**********************");
  //  Serial.print("move3_Dest:   ");
  //  Serial.print(move3_Dest);
  //  mess = move3_Dest;
  //  SendNextionValues();

  //****************************************************move1-2*****************************************

  KeyB = k2_position;
  KeyA = k1_position;
  Find_KeyDist();
  if (KeyDist != 0 && s2 != 0) {
    Key_Crono_time = int(float(100 / s1) * Seq_Time);                                         //Function to find out the distance between key frames under any + - condition
    factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
    factor = (1 / factor);
    s1_speed = abs(factor * 1000000);
    Stp_1_active = 1;
    Stp_active = 1;

    switch (a1) {
      case 0:                                                                                   //No ease
        K1A = (KeyDist / (Key_Crono_time));
        K1A = K1A * 3;
        break;
      case 3:
        K1A = (KeyDist / (Key_Crono_time ));
        break;

      case 2:
        K1A = (KeyDist / (Key_Crono_time ));
        K1A = K1A * 2;
        break;

      case 1:
        K1A = (KeyDist / (Key_Crono_time ));
        K1A = K1A * 3;
        break;
    }

    stepper1->setAcceleration(K1A);
    stepper1->setSpeedInUs(s1_speed);


    switch (move1_Dest) {                                                      //Pick up how far the motor should move before it either stopes or changes direction
      case 2:
        stepper1->moveTo(k2_position);
        break;
      case 3:
        s1_speed = s1_speed + ((s1_speed / 100) * 9);                        //remember the higher the speed value the slower it moves!
        stepper1->setSpeedInUs(s1_speed);
        stepper1->moveTo(k3_position);
        break;
      case 4:
        s1_speed = s1_speed + ((s1_speed / 100) * 9);
        stepper1->setSpeedInUs(s1_speed);
        stepper1->moveTo(k4_position);
        break;
      case 5:
        s1_speed = s1_speed + ((s1_speed / 100) * 9);
        stepper1->setSpeedInUs(s1_speed);
        stepper1->moveTo(k5_position);
        break;
      case 6:
        s1_speed = s1_speed + ((s1_speed / 100) * 9);
        stepper1->setSpeedInUs(s1_speed);
        stepper1->moveTo(k6_position);
        break;
    }
    Base_accel =  K1A;
    Last_accel = K1A;
    Last_speed = s1_speed;
    delay(10);
    ActiveMove = 1;
  } else {
    Stp_1_active = 0;
  }

  //**************************************move 1 mid way check***********************************


  //***************************Test to chech for end of move**********************************************
  while (Stp_active == 1) {


    if (stepper1->isRunning()) {

      if (k2_position > k1_position && stepper1->isRunning()) {                   //get direction
        if (stepper1->getCurrentPosition() >= (k2_position)) {                        //if + direction
          Stp_1_active = 0;
        }
      } else {
        if (stepper1->getCurrentPosition() <= (k2_position)) {                        //if - direction
          Stp_1_active = 0;
        }
      }
    } else {
      Stp_1_active = 0;

    }




    if (Stp_1_active == 0  ) {                     //Final test to see if all steppers have finished move
      Stp_active = 0;
    }
  }

  //**********************************************************************************************************
  //****************************************************move2-3***********************************************
  //**********************************************************************************************************


  KeyB = k3_position;
  KeyA = k2_position;
  Find_KeyDist();
  if (KeyDist != 0 && s3 != 0) {
    //    mess = KeyDist;
    //    SendNextionValues();
    Key_Crono_time = int(float(100 / s2) * Seq_Time);                                         //Function to find out the distance between key frames under any + - condition
    factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
    factor = (1 / factor);
    s2_speed = abs(factor * 1000000);
    Stp_1_active = 1;
    Stp_active = 1;

    switch (a2) {
      case 0:                                                                                   //No ease
        K2A = abs(KeyDist / (Key_Crono_time ));
        K2A = K2A * 3;
        break;
      case 3:
        K2A = (KeyDist / (Key_Crono_time ));
        break;

      case 2:
        K2A = (KeyDist / (Key_Crono_time ));
        K2A = K2A * 2;
        break;

      case 1:
        K2A = (KeyDist / (Key_Crono_time ));
        K2A = K2A * 3;
        break;
    }

    if (stepper1->isRunning() == true && move1_Dest > 2 && move2_Rst != 1 ) {                                   //If the stepper is running on from last move
      stepper1->setAcceleration(K2A);
      stepper1->setSpeedInUs(s2_speed);
      stepper1-> applySpeedAcceleration();
      delay(10);
    } else {
      stepper1->setAcceleration(K2A);                                                              //If starting fresh Max accel to catch up on turnarround
      stepper1->setSpeedInUs(s2_speed);

      switch (move2_Dest) {                                  //Pick up how far the motor should move before it either stopes or changes direction
        case 3:
          stepper1->moveTo(k3_position);
          break;
        case 4:
          s2_speed = s2_speed + ((s2_speed / 100) * 9);
          stepper1->setSpeedInUs(s2_speed);
          stepper1->moveTo(k4_position);
          break;
        case 5:
          s2_speed = s2_speed + ((s2_speed / 100) * 9);
          stepper1->setSpeedInUs(s2_speed);
          stepper1->moveTo(k5_position);
          break;
        case 6:
          s2_speed = s2_speed + ((s2_speed / 100) * 9);
          stepper1->setSpeedInUs(s2_speed);
          stepper1->moveTo(k6_position);
          break;
      }
      delay(10);
    }
    Base_accel = K2A;
    Last_accel = K2A;
    Last_speed = s2_speed;
    ActiveMove = 2;
  } else {
    Stp_1_active = 0;
  }




  //**************************************move 2-3 Second ramp********************************************************************


  //********************************Test to chech for end of move**************************************
  while (Stp_active == 1) {

    if (stepper1->isRunning() ) {

      if (k3_position > k2_position ) {                               //get direction
        if (stepper1->getCurrentPosition() >= (k3_position)) {                                    //if + direction
          Stp_1_active = 0;
        }
      } else {
        if (stepper1->getCurrentPosition() <= (k3_position)) {                                    //if - direction
          Stp_1_active = 0;
        }
      }
    } else {
      Stp_1_active = 0;
    }




    if (Stp_1_active == 0  ) {                                                                        //Final test to see if all steppers have finished move
      Stp_active = 0;
    }
  }

  //**********************************************************************************************************
  //****************************************************move3-4***********************************************
  //**********************************************************************************************************
  if (s4 != 0) {

    //************************Tlt move*****************************************************
    KeyB = k4_position;
    KeyA = k3_position;
    Find_KeyDist();
    if (KeyDist != 0 && s4 != 0) {
      Key_Crono_time = int(float(100 / s3) * Seq_Time);                                              //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                                  //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s3_speed = abs(factor * 1000000);
      Stp_1_active = 1;
      Stp_active = 1;


      switch (a3) {
        case 0:                                                                                       //No ease
          K3A = (KeyDist / (Key_Crono_time));
          K3A = K3A * 3;
          break;
        case 3:
          K3A = (KeyDist / (Key_Crono_time));
          break;

        case 2:
          K3A = (KeyDist / (Key_Crono_time));
          K3A = K3A * 2;
          break;

        case 1:
          K3A = (KeyDist / (Key_Crono_time));
          K3A = K3A * 3;
          break;
      }

      if (stepper1->isRunning() && move2_Dest > 3 && move3_Rst != 1 ) {                                           //If the stepper is running on from last move
        stepper1->setAcceleration(K3A);
        s3_speed = s3_speed + ((s3_speed / 100) * 9);
        stepper1->setSpeedInUs(s3_speed);
        stepper1-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper1->setAcceleration(K3A);                                                              //If starting fresh Max accel to catch up on turnarround
        stepper1->setSpeedInUs(s3_speed);
        //        mess = move3_Dest;
        //        SendNextionValues();
        switch (move3_Dest) {                                                                    //Pick up how far the motor should move before it either stopes or changes direction

          case 4:
            //s3_speed = s3_speed + ((s3_speed / 100) * 9);
            stepper1->setSpeedInUs(s3_speed);
            stepper1->moveTo(k4_position);
            break;
          case 5:
            s3_speed = s3_speed + ((s3_speed / 100) * 9);
            stepper1->setSpeedInUs(s3_speed);
            stepper1->moveTo(k5_position);
            break;
          case 6:
            s3_speed = s3_speed + ((s3_speed / 100) * 9);
            stepper1->setSpeedInUs(s3_speed);
            stepper1->moveTo(k6_position);
            break;
        }
        delay(10);
      }
      Base_accel = K3A;
      Last_accel = K3A;
      Last_speed = s3_speed;
      ActiveMove = 3;
    } else {
      Stp_1_active = 0;
    }


    //**************************************move 3-4 Second ramp********************************************************************
    //If the running steppers current end point is ahead of this key ajust its speed to match the next keys speed for a smooth entry
    //*******************************************TLT Ramp2**************************************************************************



    //********************************Test to chech for end of move**************************************
    while (Stp_active == 1) {

      if (stepper1->isRunning() ) {

        if (k4_position > k3_position ) {                               //get direction
          if (stepper1->getCurrentPosition() >= (k4_position)) {                                    //if + direction
            Stp_1_active = 0;
          }
        } else {
          if (stepper1->getCurrentPosition() <= (k4_position)) {                                    //if - direction
            Stp_1_active = 0;
          }
        }
      } else {
        Stp_1_active = 0;
      }

      if (Stp_1_active == 0  ) {                               //Final test to see if all steppers have finished move
        Stp_active = 0;
      }

    }
  }

  //**********************************************************************************************************
  //****************************************************move4-5***********************************************
  //**********************************************************************************************************
  if (s5 != 0) {

    KeyB = k5_position;
    KeyA = k4_position;
    Find_KeyDist();
    if (KeyDist != 0 && s5 != 0) {
      Key_Crono_time = int(float(100 / s4) * Seq_Time);                                     //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s4_speed = abs(factor * 1000000);
      Stp_1_active = 1;
      Stp_active = 1;

      switch (a4) {
        case 0:                                                                                   //No ease
          K4A = (KeyDist / (Key_Crono_time));
          K4A = K4A * 3;
          break;
        case 3:
          K4A = (KeyDist / (Key_Crono_time));
          break;

        case 2:
          K4A = (KeyDist / (Key_Crono_time));
          K4A = K4A * 2;
          break;

        case 1:
          K4A = (KeyDist / (Key_Crono_time));
          K4A = K4A * 3;
          break;
      }

      if (stepper1->isRunning() && move3_Dest > 4 && move4_Rst != 1) {                                           //If the stepper is running on from last move
        stepper1->setAcceleration(K4A);
        s4_speed = s4_speed + ((s4_speed / 100) * 9);
        stepper1->setSpeedInUs(s4_speed);
        stepper1-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper1->setAcceleration(K4A);                                                              //If starting fresh Max accel to catch up on turnarround
        stepper1->setSpeedInUs(s4_speed);

        switch (move4_Dest) {                                                                     //Pick up how far the motor should move before it either stopes or changes direction
          case 5:
            stepper1->moveTo(k5_position);
            break;
          case 6:
            s4_speed = s4_speed + ((s4_speed / 100) * 9);
            stepper1->setSpeedInUs(s4_speed);
            stepper1->moveTo(k6_position);
            break;
        }
        delay(10);
      }
      Base_accel = K4A;
      Last_accel = K4A;
      Last_speed = s4_speed;
      ActiveMove = 4;
    } else {
      Stp_1_active = 0;
    }
    //**************************************move 4-5 Second ramp********************************************************************
    //If the running steppers current end point is ahead of this key ajust its speed to match the next keys speed for a smooth entry
    //*******************************************TLT Ramp2**************************************************************************


    //********************************Test to chech for end of move**************************************
    while (Stp_active == 1) {

      if (stepper1->isRunning() ) {

        if (k5_position > k4_position ) {                               //get direction
          if (stepper1->getCurrentPosition() >= (k5_position)) {                                    //if + direction
            Stp_1_active = 0;
          }
        } else {
          if (stepper1->getCurrentPosition() <= (k5_position)) {                                    //if - direction
            Stp_1_active = 0;
          }
        }
      } else {
        Stp_1_active = 0;
      }

      if (Stp_1_active == 0  ) {                               //Final test to see if all steppers have finished move
        Stp_active = 0;
      }
    }
  }

  //**********************************************************************************************************
  //****************************************************move5-6***********************************************
  //**********************************************************************************************************
  if (s6 != 0) {


    //************************Tlt move*****************************************************
    KeyB = k6_position;
    KeyA = k5_position;
    Find_KeyDist();
    if (KeyDist != 0 && s6 != 0) {
      Key_Crono_time = int(float(100 / s5) * Seq_Time);                                     //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s5_speed = abs(factor * 1000000);
      Stp_1_active = 1;
      Stp_active = 1;

      switch (a5) {
        case 0:                                                                                   //No ease
          K5A = (KeyDist / (Key_Crono_time));
          K5A = K5A * 3;
          break;
        case 3:
          K5A = (KeyDist / (Key_Crono_time));
          break;

        case 2:
          K5A = (KeyDist / (Key_Crono_time));
          K5A = K5A * 2;
          break;

        case 1:
          K5A = (KeyDist / (Key_Crono_time));
          K5A = K5A * 3;
          break;
      }

      if (stepper1->isRunning() && move4_Dest > 5 && move5_Rst != 1) {                                  //If the stepper is running on from last move
        stepper1->setAcceleration(K5A);
        s5_speed = s5_speed + ((s5_speed / 100) * 9);
        stepper1->setSpeedInUs(s5_speed);
        stepper1-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper1->setAcceleration(K5A);                                                  //If starting fresh Max accel to catch up on turnarround
        stepper1->setSpeedInUs(s5_speed);
        stepper1->moveTo(k6_position);
        delay(10);
      }
      Base_accel = K5A;
      Last_accel = K5A;
      Last_speed = s5_speed;
      ActiveMove = 5;
    } else {
      Stp_1_active = 0;
    }


    //**************************************move 5-6 Second ramp********************************************************************
    //If the running steppers current end point is ahead of this key ajust its speed to match the next keys speed for a smooth entry

    //********************************Test to chech for end of move**************************************
    while (Stp_active == 1) {

      if (stepper1->isRunning() ) {

        if (k6_position > k5_position ) {                               //get direction
          if (stepper1->getCurrentPosition() >= (k6_position)) {            //if + direction
            Stp_1_active = 0;
          }
        } else {
          if (stepper1->getCurrentPosition() <= (k6_position)) {            //if - direction
            Stp_1_active = 0;
          }
        }
      } else {
        Stp_1_active = 0;
      }
      if (Stp_1_active == 0) {        //Final test to see if all steppers have finished move
        Stp_active = 0;
      }

    }
  }
  //End Game for Sequencer moves
  if (Bounce >= 1) {
    BounceActive = 1;                                                         //Tell the system that bounce is active
    Bounce = Bounce - 1;

    But_Com = 6;
    SendNextionValues();
    But_Com = 0;

    if (BounceReturn == 1) {
      BounceReturn = 2;                                                        //Reverse move
      delay(Pause);
      Start_6();
    }
    if (BounceReturn == 2) {                                                   //Forward move
      BounceReturn = 1;
      delay(Pause);
      Start_6();
    }

  } else  {                                                                     //Bounce must = 0 Finish the move

    if (BounceReturn == 1 && BounceActive == 1) {                                  //Return for last bounce
      BounceReturn = 2;                                                            //Reverse move
      BounceActive = 0;
      delay(Pause);
      Start_6();
    } else {
      delay(Pause);
      if (PTZ_IDhld == 5) {
        But_Com = 5;                                                                //Stop the timer
        SendNextionValues();
      }
      But_Com = 0;
      delay(10);
      if (PTZ_IDhld == 5) {
        But_Com = 6;                                                                //Update Nextion bounce
        SendNextionValues();
      }                                                               //Update Nextion bounce

      But_Com = 0;
      BounceReturn = 1;                                                           //Forward move restored

      delay(10);
      if (PTZ_IDhld == 5) {
        digitalWrite(CAM, HIGH);                                                     //Stop camera
        delay (200);
        digitalWrite(CAM, LOW);
      }
      BounceActive = 0;
      Set_forward_values();

      Start_1();
    }
  }
}
//**********************************************************Sequencer bounce****************************************
void Start_6() {
  Start_5();
}



//******Set stepper speed based on time distance and amount od ease InOut***********
void SetSpeed() {

  //Serial.print("Enterring Setpeed");
  Travel_dist = abs(out_position - in_position);                           // Distance we have to go
  step_speed = (Travel_dist / Crono_time);                                  // fot the next three lines This formula should all be on one line but I cant get it to work!


  switch (ease_InOut) {
    case 0:                                                                //No ease
      Ease_Value = 4000;
      break;

    case 3:

      Ease_Value = (Travel_dist / (Crono_time * 12));
      break;
    case 2:


      Ease_Value = (Travel_dist / (Crono_time * 12));
      Ease_Value = (Ease_Value * 2);
      break;

    case 1:

      Ease_Value = (Travel_dist / (Crono_time * 12));
      Ease_Value = (Ease_Value * 3);
      break;
  }
  return;
}

void stepper1_plan() {
  k1_k2_Dir = 0;
  k2_k3_Dir = 0;
  k3_k4_Dir = 0;
  k4_k5_Dir = 0;
  k5_k6_Dir = 0;

  move2_Rst = 0;
  move3_Rst = 0;
  move4_Rst = 0;
  move5_Rst = 0;
  //********************* Move1 plan***********************************

  //***********k1-k2 move*************************
  move1_Dest = 1;                                            //The starting key
  if (s2 != 0) {
    KeyB = k2_position;                                      //check to see if there was a move
    KeyA = k1_position;
    Find_KeyDist();
    if (KeyDist != 0) {
      move1_Dest = 2;
    } else {                                                     //No move. Next possible move is from 2 so set it but tell move 2 it is a restart not moving on from 1
      move1_Dest = 2;
      move2_Rst = 1;
    }
  }

  // Test for continouse mov to k3
  if (move1_Dest == 2) {
    move1_Dest = 2;
    if (k2_position > k1_position) {                      //check direction if + or -
      k1_k2_Dir = 1;
    }
    if (k2_position < k1_position) {                      //If not it must be ether less - or 0
      k1_k2_Dir = -1;
    }
    if (k2_position == k1_position)  {
      k1_k2_Dir = 0;
    }
    //***********k2-k3 move*************************
    if (k3_position > k2_position) {                      //check direction if + or -
      k2_k3_Dir = 1;
    }
    if (k3_position < k2_position) {
      k2_k3_Dir = -1;
    }
    if (k3_position == k2_position) {
      k2_k3_Dir = 0;
    }

    //***************look for direction change**********************
    if ((k2_k3_Dir != 0) && (k1_k2_Dir == k2_k3_Dir)) {
      move1_Dest = 3;
    }
  }
  //***********k3-k4 move*************************
  if (move1_Dest == 3) {                                   //if the last key advanced check the next
    if (k4_position > k3_position) {
      k3_k4_Dir = 1;
    }
    if (k4_position < k3_position) {
      k3_k4_Dir = -1;
    }
    if (k4_position == k3_position) {
      k3_k4_Dir = 0;
    }

    if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {          //***************look for direction change**********************
      move1_Dest = 4;
    }
  }
  //***********k4-k5 move*************************
  if (move1_Dest == 4) {                                    //if the last key advanced check the next
    if (k5_position > k4_position) {                    //check direction if + or -
      k4_k5_Dir = 1;
    }
    if (k5_position < k4_position) {
      k4_k5_Dir = -1;
    }
    if (k5_position == k4_position) {
      k4_k5_Dir = 0;
    }

    if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {          //***************look for direction change**********************
      move1_Dest = 5;
    }
  }
  //***********k5-k6 move*************************
  if (move1_Dest == 5) {
    if (k6_position > k5_position) {                       //check direction if + or -
      k5_k6_Dir = 1;
    }
    if (k6_position < k5_position) {
      k5_k6_Dir = -1;
    }
    if (k6_position == k5_position) {
      k5_k6_Dir = 0;
    }
    if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {           //***************look for direction change**********************
      move1_Dest = 6;
    }
  }


  k1_k2_Dir = 0;
  k2_k3_Dir = 0;
  k3_k4_Dir = 0;
  k4_k5_Dir = 0;
  k5_k6_Dir = 0;

  //*********************Tlt motor Move2 plan***********************************

  switch (move1_Dest) {                                                 //Find the full move start if move one only took us to 2 then 2-3 must be a restart for a change in direction


    case 2:
      move2_Dest = 2;
      //*********************k1-k2**************************
      if (move2_Dest == 2 && s3 != 0) {
        if (k2_position > k1_position) {                                  //check direction if + or -
          k1_k2_Dir = 1;
        }
        if (k2_position < k1_position) {
          k1_k2_Dir = -1;
        }
        if (k2_position == k1_position) {
          k1_k2_Dir = 0;
        }

        //*********************k2-k3**************************
        if (k3_position > k2_position) {                                    //check direction if + or -
          k2_k3_Dir = 1;
        }
        if (k3_position < k2_position) {
          k2_k3_Dir = -1;
        }
        if (k3_position == k2_position) {
          k2_k3_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k2_k3_Dir != 0) && (k1_k2_Dir == k2_k3_Dir)) {                                                                 // if no direction change
          move2_Dest = 3;
        } else {
          move2_Dest = 3;
          move3_Rst = 1;
        }
      }
      //*********************k3-k4**************************
      if (move2_Dest == 3 && s4 != 0) {
        if (k4_position > k3_position) {                                    //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (k4_position < k3_position) {
          k3_k4_Dir = -1;
        }
        if (k4_position == k3_position) {
          k3_k4_Dir = 0;
        }

        //***************look for direction change**********************
        if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {                                                                 // if no direction change
          move2_Dest = 4;
        }
      }
      //***********k4-k5 move*************************
      if (move2_Dest == 4 && s5 != 0) {
        if (k5_position > k4_position) {                                    //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                  // if no direction change
          move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (move2_Dest == 5 && s6 != 0) {
        if (k6_position > k5_position) {                                    //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                // if no direction change
          move2_Dest = 6;
        }
      }
      break;
    case 3:
      move2_Dest = 3;


      //***********k2-k3 move*************************
      if (move2_Dest == 3 && s4 != 0) {
        if (k3_position > k2_position) {                                  //check direction if + or -
          k2_k3_Dir = 1;
        }
        if (k3_position < k2_position) {
          k2_k3_Dir = -1;
        }
        if (k3_position == k2_position) {
          k2_k3_Dir = 0;
        }

        //***********k3-k4 move*************************
        if (k4_position > k3_position) {                                  //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (k4_position < k3_position) {
          k3_k4_Dir = -1;
        }
        if (k4_position == k3_position) {
          k3_k4_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {                // if no direction change
          move2_Dest = 4;
        }
      }
      //***********k4-k5 move*************************
      if (move2_Dest == 4 && s5 != 0) {
        if (k5_position > k4_position) {                                  //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                // if no direction change
          move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (move2_Dest == 5 && s6 != 0) {

        if (k6_position > k5_position) {                                  //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                // if no direction change
          move2_Dest = 6;
        }
      }
      break;
    case 4:
      move2_Dest = 4;

      //***********k3-k4 move*************************

      if (move2_Dest == 4 && s5 != 0) {
        if (k4_position > k3_position) {                                  //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (k4_position < k3_position) {
          k3_k4_Dir = -1;
        }
        if (k4_position == k3_position) {
          k3_k4_Dir = 0;
        }
        //***********k4-k5 move*************************
        if (k5_position > k4_position) {                                  //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                // if no direction change
          move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (move2_Dest == 5 && s6 != 0) {
        if (k6_position > k5_position) {                                  //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                // if no direction change
          move2_Dest = 6;
        }
      }
      break;

    case 5:
      move2_Dest = 5;
      //***********k4-k5 move*************************

      if (move2_Dest == 5 && s6 != 0) {
        if (k5_position > k4_position) {                                  //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }

        //***********k5-k6 move*************************
        if (k6_position > k5_position) {                                    //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                 // if no direction change
          move2_Dest = 6;
        }
      }
      break;
  }

  k1_k2_Dir = 0;
  k2_k3_Dir = 0;
  k3_k4_Dir = 0;
  k4_k5_Dir = 0;
  k5_k6_Dir = 0;
  //*********************Tlt motor Move3 plan***********************************

  switch (move2_Dest) {                                                   //Find the full move start

    case 3:

      move3_Dest = 3;
      //***********k2-k3 move*************************
      if (move3_Dest == 3 && s4 != 0) {
        if (k3_position > k2_position) {                                  //check direction if + or -
          k2_k3_Dir = 1;
        }
        if (k3_position < k4_position) {
          k2_k3_Dir = -1;
        }
        if (k3_position == k4_position) {
          k2_k3_Dir = 0;
        }
        //***********k3-k4 move*************************

        if (k4_position > k3_position) {                                  //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (k4_position < k3_position) {
          k3_k4_Dir = -1;
        }
        if (k4_position == k3_position) {
          k3_k4_Dir = 0;
        }

        //***************look for direction change**********************
        if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {              // if no direction change
          move3_Dest = 4;
        } else {
          move3_Dest = 4;
          move4_Rst = 1;
        }
      }
      //***********k4-k5 move*************************
      if (move3_Dest == 4 && s5 != 0) {
        if (k5_position > k4_position) {                                //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {              // if no direction change
          move3_Dest = 5;
        }
      }
      //***********Test for moving on 3-6
      if (move3_Dest == 5 && s6 != 0) {
        if (k6_position > k5_position) {                                //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = 0;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {              // if no direction change
          move3_Dest = 6;
        }
      }
      break;
    case 4:
      move3_Dest = 4;

      //***********k3-k4 move*************************
      if (move3_Dest == 4 && s5 != 0) {
        if (k4_position > k3_position) {                                //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (k4_position < k3_position) {
          k3_k4_Dir = -1;
        }
        if (k4_position == k3_position) {
          k3_k4_Dir = 0;
        }

        //***********k4-k5 move*************************

        if (k5_position > k4_position) {                                //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {              // if no direction change
          move3_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (move3_Dest == 5 && s6 != 0) {
        if (k6_position > k5_position) {                                //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {              // if no direction change
          move3_Dest = 6;
        }
      }
      break;
    case 5:
      move3_Dest = 5;
      //***********k4-k5 move*************************
      if (move3_Dest == 5 && s6 != 0) {
        if (k5_position > k4_position) {                                //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************
        if (k6_position > k5_position) {                                //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {             // if no direction change
          move3_Dest = 6;
        }
      }
      break;
  }
  k1_k2_Dir = 0;
  k2_k3_Dir = 0;
  k3_k4_Dir = 0;
  k4_k5_Dir = 0;
  k5_k6_Dir = 0;
  //*********************Tlt motor Move4 plan***********************************

  switch (move3_Dest) {                                                 //Find the full move start


    case 4:

      move4_Dest = 4;


      //***********k3-k4 move*************************is it going on to 6
      if (move4_Dest == 4 && s5 != 0) {
        if (k4_position > k3_position) {                                //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (k4_position < k3_position) {
          k3_k4_Dir = -1;
        }
        if (k4_position == k3_position) {
          k3_k4_Dir = 0;
        }
        //***********k4-k5 move*************************is it going on to 6

        if (k5_position > k4_position) {                                //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {             // if no direction change
          move4_Dest = 5;
        } else {
          move4_Dest = 5;
          move5_Rst = 1;
        }
      }
      //***********k5-k6 move*************************
      if (move4_Dest == 5 && s6 != 0) {
        if (k6_position > k5_position) {                                //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {              // if no direction change
          move4_Dest = 6;
        }
      }
      break;
    case 5:
      move4_Rst = 1;
      move4_Dest = 5;
      //***********k4-k5 move*************************is it going on to 6
      if (move4_Dest == 5 && s6 != 0) {
        if (k5_position > k4_position) {                                //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************

        if (k6_position > k5_position) {                                  //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                // if no direction change
          move4_Dest = 6;
        }
      }
      break;
  }
  //********************* Move5 plan***********************************

  switch (move4_Dest) {                                                   //Find the full move start

    case 5:

      move4_Dest = 5;

      //***********k4-k5 move*************************is it going on to 6
      if (move5_Dest == 5 && s6 != 0) {
        if (k5_position > k4_position) {                                  //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************is it going on to 6

        if (k6_position > k5_position) {                                 //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {            // if no direction change
          move5_Dest = 6;
        } else {
          move5_Dest = 6;
          //move6_Rst = 1;
        }
      }
      break;
  }
  return;
}
//*****PTZ Control Function*****
void PTZ_Control() {
  if (JBpressent != 1) {                                              //Tell the system the Jib is online but only first time round
    SendNextionValues();
    JBpressent = 1;
  }
  //Record State
  if (Rec != lastRecState) {
    digitalWrite(CAM, HIGH);                                           //Start/stop Record on camera. Switch must be flipped
    delay (200);
    digitalWrite(CAM, LOW);
    lastRecState = Rec;
  }


  //Joystick comands
  //***Jib Tilt***
  if (abs(Joy_Tilt_Speed) > 150 ) {
    JTilt_is_moving = true;
    stepper1->setAcceleration(abs(Joy_Tilt_Accel / 16));                    //If starting fresh Max accel to catch up on turnarround
    stepper1->setSpeedInHz(abs(Joy_Tilt_Speed / 8));
    if (Joy_Tilt_Speed > 1) {
      stepper1->runForward();
    } else {
      stepper1->runBackward();
    }
  } else {
    if (JTilt_is_moving) {
      stepper1->setAcceleration(abs(Joy_Tilt_Accel / 16));                  //Breaking acceloration on the stick moving back to 0
      stepper1->applySpeedAcceleration();
      stepper1->stopMove();
      JTilt_is_moving = false;
    }
  }
}
void SetID() {
  SysMemory.begin("ID", false);
  SysMemory.putUInt("PTZ_ID", PTZ_ID_Nex);
  SysMemory.end();
  PTZ_ID = PTZ_ID_Nex;
}

void Load_SysMemory() {
  //SysMemory Recover last setup

  SysMemory.begin("In_positions", false);                              //Recover the last In_Positions from memory before shutdown
  in_position = SysMemory.getUInt("in_position", 0);
  SysMemory.end();

  SysMemory.begin("Out_positions", false);                              //Recover the last Out_Positions from memory before shutdown
  out_position =  SysMemory.getUInt("out_position", 0);
  SysMemory.end();

  SysMemory.begin("P1pos", false);                                      //Recover PTZ Cam1 poses
  P1_S = SysMemory.getUInt("P1_S", 10);
  SysMemory.end();

  SysMemory.begin("P2pos", false);
  P2_S = SysMemory.getUInt("P2_S", 10);
  SysMemory.end();

  SysMemory.begin("P3pos", false);
  P3_S = SysMemory.getUInt("P3_S", 10);
  SysMemory.end();

  SysMemory.begin("P4pos", false);
  P4_S = SysMemory.getUInt("P4_S", 10);
  SysMemory.end();

  SysMemory.begin("P5pos", false);
  P5_S = SysMemory.getUInt("P5_S", 10);
  SysMemory.end();

  SysMemory.begin("P6pos", false);
  P6_S = SysMemory.getUInt("P6_S", 10);
  SysMemory.end();

  SysMemory.begin("P7pos", false);
  P7_S = SysMemory.getUInt("P7_S", 10);
  SysMemory.end();

  SysMemory.begin("P8pos", false);
  P8_S = SysMemory.getUInt("P8_S", 10);
  SysMemory.end();

  SysMemory.begin("P9pos", false);
  P9_S = SysMemory.getUInt("P9_S", 10);
  SysMemory.end();

  SysMemory.begin("P10pos", false);
  P10_S = SysMemory.getUInt("P10_S", 10);
  SysMemory.end();

  SysMemory.begin("P11pos", false);
  P11_S = SysMemory.getUInt("P11_S", 10);
  SysMemory.end();

  SysMemory.begin("P12pos", false);
  P12_S = SysMemory.getUInt("P12_S", 10);
  SysMemory.end();

  SysMemory.begin("P13pos", false);
  P13_S = SysMemory.getUInt("P13_S", 10);
  SysMemory.end();

  SysMemory.begin("P14pos", false);
  P14_S = SysMemory.getUInt("P14_S", 10);
  SysMemory.end();

  SysMemory.begin("P15pos", false);
  P1_S = SysMemory.getUInt("P6_S", 10);
  SysMemory.end();

  SysMemory.begin("P16pos", false);
  P16_S = SysMemory.getUInt("P16_S", 10);
  SysMemory.end();


  SysMemory.begin("K1_pos", false);                               //Recover sequencer Key K1
  k1_position = SysMemory.getUInt("k1_pos", 0);
  SysMemory.end();

  SysMemory.begin("K2_pos", false);                               //Recover sequencer Key K1
  k2_position = SysMemory.getUInt("k2_pos", 0);
  SysMemory.end();

  SysMemory.begin("K3_pos", false);                               //Recover sequencer Key K1
  k3_position = SysMemory.getUInt("k3_pos", 0);
  SysMemory.end();

  SysMemory.begin("K4_pos", false);                               //Recover sequencer Key K1
  k4_position = SysMemory.getUInt("k4_pos", 0);
  SysMemory.end();

  SysMemory.begin("K5_pos", false);                               //Recover sequencer Key K1
  k5_position = SysMemory.getUInt("k5_pos", 0);
  SysMemory.end();

  SysMemory.begin("K6_pos", false);                               //Recover sequencer Key K1
  k6_position = SysMemory.getUInt("k6_pos", 0);
  SysMemory.end();
}


void ClearKeys() {                                                //Function to return system memmory of keys back to 0
  clrK = 0;
  nvs_flash_erase(); // erase the NVS partition and...
  nvs_flash_init(); // initialize the NVS partition.

  PTZ_ID_Nex = PTZ_ID;
  SetID();
  ESP.restart();
}



void Percent_100(void) {
  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(35, 16);            // Start at top-left corner
  display.cp437(true);                  // Use full 256 char 'Code Page 437' font
  if (PTZ_ID == 5) {
    OLED_ID = 0;
  } else {
    OLED_ID = PTZ_ID;
  }
  display.printf("ID:%d\n", OLED_ID);
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(35, 25);            // Start at top-left corner
  display.cp437(true);                  // Use full 256 char 'Code Page 437' font
  display.write("100%");
  display.drawBitmap(
    (display.width()  - 60 ),
    (display.height() - 10),
    logo_bmp4, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(100);
}

void Percent_75(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 16);
  display.cp437(true);
  if (PTZ_ID == 5) {
    OLED_ID = 0;
  } else {
    OLED_ID = PTZ_ID;
  }
  display.printf("ID:%d\n", OLED_ID);

  //display.write("ID:0");
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 25);
  display.cp437(true);
  display.write("75%");
  //display.println(BatV);
  display.drawBitmap(
    (display.width()  - 60 ),
    (display.height() - 10),
    logo_bmp3, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(100);
}

void Percent_50(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 16);
  display.cp437(true);
  if (PTZ_ID == 5) {
    OLED_ID = 0;
  } else {
    OLED_ID = PTZ_ID;
  }
  display.printf("ID:%d\n", OLED_ID);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 25);
  display.cp437(true);
  display.write("50%");
  display.drawBitmap(
    (display.width()  - 60 ),
    (display.height() - 10),
    logo_bmp2, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(100);
}

void Percent_25(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 16);
  display.cp437(true);
  if (PTZ_ID == 5) {
    OLED_ID = 0;
  } else {
    OLED_ID = PTZ_ID;
  }
  display.printf("ID:%d\n", OLED_ID);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 25);
  display.cp437(true);
  display.write("25%");
  display.drawBitmap(
    (display.width()  - 60 ),
    (display.height() - 10),
    logo_bmp1, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(100);
}

void Percent_0(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 16);
  display.cp437(true);
  if (PTZ_ID == 5) {
    OLED_ID = 0;
  } else {
    OLED_ID = PTZ_ID;
  }
  display.printf("ID:%d\n", OLED_ID);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 25);
  display.cp437(true);
  display.write("0%");
  display.drawBitmap(
    (display.width()  - 60 ),
    (display.height() - 10),
    logo_bmp1, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(100);
}
void BatCheck() {
  BatV = analogRead(BatPin);
  BatV = ((BatV / 4095) * 2) * 4.182;
  if (BatV > 8) {
    Percent_100();
  } else {
    if (BatV > 7.75) {
      Percent_75();
    } else {
      if (BatV > 7.5) {
        Percent_50();
      } else {
        if (BatV > 7.25) {
          Percent_25();
        } else {
          Percent_0();
        }
      }
    }
  }
}

void Oledmessage() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 16);
  display.cp437(true);
  display.printf("Sld:%d\n", Sld);
  display.display();
  delay(2000);
}
