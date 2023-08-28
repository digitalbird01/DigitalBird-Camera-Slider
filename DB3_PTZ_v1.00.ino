//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*Digital Bird DB3 VISCA PTZ HEAD part of the Digital Bird Motion Control System Created by Colin Henderson 2021
DB3_PTZ Version 1.0
 
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
  DB3 V: 1.0
*/////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <FastLED.h>
#include <nvs_flash.h>
#include <FastAccelStepper.h>
#include "esp_task_wdt.h"
#include <AS5600.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>
HardwareSerial UARTport(2); // use UART2 for VISCA comumication

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET 4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT   10
#define LOGO_WIDTH    25

#define NUM_LEDS 2
#define DATA_PIN 19
CRGB leds[NUM_LEDS];

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


// As in StepperDemo for Motor 1 on ESP32

#define step1_pinSTEP 26
#define step1_pinDIR  27
#define step2_pinSTEP 12
#define step2_pinDIR  14
#define step3_pinSTEP 25
#define step3_pinDIR  33
#define step4_pinSTEP 18
#define step4_pinDIR  5

#define CAM A10                               // Pin Gpio 4 Camera trigger
#define StepD 13                              // Pin GPio 13 Activate Deactivate steppers

//#define Mount_PIN A12                       //Switch Gpio 2 to mount the head
//#define DisMount_PIN A13                    //Switch Gpio 15 to dismount the head

#define Hall_Pan A12                          //Hall sensor for Pan axis
#define Hall_Tilt A13                        //Hall sensor for Pan axis



#define StepFOC 32                            //Focus motor enable pin for Stepper3
#define StepZOOM 32                            //Focus motor enable pin for Stepper3
const byte pan_PIN = A6;                      // pin 34 Joy Stick input
const byte tilt_PIN = A7;                     // Tilt 35 Joy Stick input
const int BatPin = 39;                        //Battery monitoring pin
float BatV;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;
FastAccelStepper *stepper4 = NULL;


//Variables
//int T_Rev = 0;
int Pan_Stop;
int Tilt_Stop;
int Forward_P_Out = 3800;                   //Pan & Tilt limits
int Forward_P_In = -3800;
int Forward_T_Out = 7000;
int Forward_T_In = -3500;

int Tiltposition;
int Panposition;
int Focusposition;
int Zoomposition;


int Snd;
int clrK;                                     //Clear all keys 1 or 0
int stopM_active;
int OLED_ID;
int lastP1;
int lastP2;
int lastP3;
int lastP4;
int lastP5;
int lastP6;

int lastInP;
int lastOutP;

unsigned long previousMillis = 0;
const long debouncedelay = 100;
float panspeed_current = 0;
float tiltspeed_current = 0;
float focusspeed_current = 0;
float zoomspeed_current = 0;

float speedfactor = 1;
float accelerationfactor = 1;
int pan_is_moving;
int tilt_is_moving;
int focus_is_moving;
int zoom_is_moving;

String DBFirmwareVersion = "frm V:5.00";      //Current firmware version

//*******************************************************************************************************************
//Change these valus depending on which pan tilt head you havebuilt over the top or balanced
int Tilt_J_Speed = 150;                       //Set this to 150 for the Balanced PT 200 for the Over the Top PT
int Pan_J_Speed = 4000;                        //Set this to 4000  for both the Balanced PT and Over the Top PT
//********************************************************************************************************************
//****************************PTZ Variables*************************************//
int Joy_Pan_Speed;                           //PTZ variables
int Joy_Pan_Accel;
int Joy_Tilt_Speed;
int Joy_Tilt_Accel = 1000;
int Joy_Focus_Speed;
int Joy_Focus_Accel;
int Joy_Zoom_Speed;
int Joy_Zoom_Accel;
int Focus_Stop;
int Zoom_Stop;

int Rec;                                      //Record request 0 -1
int lastRecState;                             //Last Record button state for camera
int PTZ_Cam = 0;                              //Current PTZ camera defaulted to 0
int PTZ_ID = 4;                               //PTZ Hardwae ID defaulted to 4 canot be used by PTZ control
int lastPTZ_ID = PTZ_ID;                      //Stores the last known PTZ_ID
int PTZ_ID_Nex;                               //incomming ID request from nection control
int PTZ_Pose;                                 //Curent PTZ Pose key
int PTZ_SaveP;                                //PTZ save position comand true or false
int lastPTZ_Pose;                             //Store last PTZ_position for future save too
int usejoy;                                   //Joystick button state


int LM;                                       //Focus/Zoom limits set
int lastLM;
int limitSetend;
int cam1_F_In;                                //Cam1 Focus/zoom inpoint limit
int cam1_F_Out;                               //Cam1 Focus/zoom inpoint limit
int cam2_F_In;                                //Cam2 Focus/zoom inpoint limit
int cam2_F_Out;                               //Cam2 Focus/zoom inpoint limit
int cam3_F_In;                                //Cam3 Focus/zoom inpoint limit
int cam3_F_Out;                               //Cam3 Focus/zoom inpoint limit

int cam_F_In;                                //Actual value used dependant on ID
int cam_F_Out;                                //Actual value used dependant on ID
int cam_Z_In;                                //Actual value used dependant on ID
int cam_Z_Out;                               //Actual value used dependant on ID

int Z_Stop;                                   //Used to prevent the focus motor crashing into the end over and over
int B_Stop;                                   //Used to prevent the focus motor crashing into the end over and over
int T_position;                               //Holding variable for PTZ positions
int P_position;
int F_position;
int Z_position;

int stopM_play;                               //Stopmotion Trigger SM
int stopM_cancel;                             //Stopmmotion cancel SC
int SM;
int SC;
int stopM_frames;                             //The Total number of frames required for stopmotion
int SMC;                                    //stopmotion counter
int But;

int P1_T = 10;                        //PTZ Tilt positions
int P2_T = 10;
int P3_T = 10;
int P4_T = 10;
int P5_T = 10;
int P6_T = 10;
int P7_T = 10;
int P8_T = 10;
int P9_T = 10;
int P10_T = 10;
int P11_T = 10;
int P12_T = 10;
int P13_T = 10;
int P14_T = 10;
int P15_T = 10;
int P16_T = 10;

int P1_P = 10;                          //PTZ Pan positions
int P2_P = 10;
int P3_P = 10;
int P4_P = 10;
int P5_P = 10;
int P6_P = 10;
int P7_P = 10;
int P8_P = 10;
int P9_P = 10;
int P10_P = 10;
int P11_P = 10;
int P12_P = 10;
int P13_P = 10;
int P14_P = 10;
int P15_P = 10;
int P16_P = 10;


int P1_F = 10;                          //PTZ Focus positions
int P2_F = 10;
int P3_F = 10;
int P4_F = 10;
int P5_F = 10;
int P6_F = 10;
int P7_F = 10;
int P8_F = 10;
int P9_F = 10;
int P10_F = 10;
int P11_F = 10;
int P12_F = 10;
int P13_F = 10;
int P14_F = 10;
int P15_F = 10;
int P16_F = 10;

int P1_Z = 10;                          //PTZ Zoom positions
int P2_Z = 10;
int P3_Z = 10;
int P4_Z = 10;
int P5_Z = 10;
int P6_Z = 10;
int P7_Z = 10;
int P8_Z = 10;
int P9_Z = 10;
int P10_Z = 10;
int P11_Z = 10;
int P12_Z = 10;
int P13_Z = 10;
int P14_Z = 10;
int P15_Z = 10;
int P16_Z = 10;

long F_revolutions = 0;                         // number of revolutions the encoder has made
long Z_revolutions = 0;                         // number of revolutions the encoder has made
long P_revolutions = 0;                         // number of revolutions the encoder has made
long T_revolutions = 0;                         // number of revolutions the encoder has made

long F_E_position = 0;                        // the calculated value the encoder is at
long Z_E_position = 0;
long P_E_position = 0;
long T_E_position = 0;

long F_output;                                // raw value from AS5600
long Z_output;
long P_output;
long T_output;

long F_lastOutput;                              // last output from AS5600
long Z_lastOutput;
long P_lastOutput;
long T_lastOutput;



long F_E_outputPos = 0;                         // Ajustment value
long Z_E_outputPos = 0;
long P_E_outputPos = 0;
long T_E_outputPos = 0;


long F_E_lastOutput;                            // last output from AS5600
long Z_E_lastOutput;
long P_E_lastOutput;
long T_E_lastOutput;

long F_S_position = 0;                          // Number of stepper steps at 16 microsteps/step interpolated by the driver to 256 microsteps/step (confusing!)
long Z_S_position = 0;
long P_S_position = 0;
long T_S_position = 0;

long F_E_Trim = 0;                              // A Trim value for the encoder effectively setting its 0 position to match Step 0 position
long Z_E_Trim = 0;
long P_E_Trim = 0;
long T_E_Trim = 0;

long F_E_Current = 0;
long Z_E_Current = 0;
long P_E_Current = 0;
long T_E_Current = 0;


long F_E_Turn = 0;
long Z_E_Turn = 0;
long P_E_Turn = 0;
long T_E_Turn = 0;

long F_E_outputTurn = 0;
long Z_E_outputTurn = 0;
long P_E_outputTurn = 0;
long T_E_outputTurn = 0;

long F_E_outputHold = 32728;
long Z_E_outputHold = 32728;
long P_E_outputHold = 32728;
long T_E_outputHold = 32728;

long F_loopcount = 0;
long Z_loopcount = 0;
long P_loopcount = 0;
long T_loopcount = 0;

long  F_S_lastPosition = 0;
long  Z_S_lastPosition = 0;
long  P_S_lastPosition = 0;
long  T_S_lastPosition = 0;

float factor;
int Mount = 0;                                 //Is the Mount function active 1 true 0 false

long tilt;
long pan;

long TiltJoySpeed;
long PanJoySpeed;
long tilt_AVG;
long pan_AVG;


long PANin_position = 0;                      // variable to hold IN positions for slider
long TLTin_position = 0;
long FOCin_position = 0;
long ZMin_position = 0;

long PANout_position = 0;                     // variable to hold OUT position for slider
long TLTout_position = 0;
long FOCout_position = 0;
long ZMout_position = 0;

long PANtravel_dist = 0;                        //Distance to travel
long TLTtravel_dist = 0;
long FOCtravel_dist = 0;
long ZOOMtravel_dist = 0;

float PANstep_speed = 0;                        // default travel speed between IN and OUT points
float TLTstep_speed = 0;
float FOCstep_speed = 0;
float ZOOMstep_speed = 0;

int PANfps_step_dist = 0;
int TLTfps_step_dist = 0;
int FOCfps_step_dist = 0;
int ZOOM_step_dist = 0;

int TLTease_Value = 0;                      // Variable to hold actual Ease value used for acceleration calculation
int PANease_Value = 0;
int FOCease_Value = 0;
int ZOOMease_Value = 0;

int  TLTTlps_step_dist;
int  PANTlps_step_dist;
int  FOCTlps_step_dist;
int  ZOOMTlps_step_dist;

long Max_Steps = 0;                         //Variable used to set the max number of steps for the given leangth of rail
long set_speed = 0;
long in_position = 0;                       // Not Used by Pan Tilt but required to keep everything the same incoming and outgoing
long out_position = 0;                      // Not Used by Pan Tilt but required to keep everything the same incoming and outgoing


int Crono_time = 10;
int crono_hours = 0;
int crono_minutes = 0;
int crono_seconds = 0;
int ElapsedTime;
int MoveTime;

int move_left = 0;                        // variable to detect move left on Nextion LCD
int move_right = 0;                       // variable to confirm move right on Nextion LCD
int start_cycle = 0;                      // variable to confirm start of travel between IN and OUT points
long Home_position = 0;                   // Variable to st home position
int ease_InOut = 0;                       // Variable to collect Ease_InOut from nextion screen value range 1-3
int LastMove;                             // Last key set
int ease_time = 0;                        // Extra time allowed for ease in out calculation
int move_finished = 1;                    // Used to check if move is completed
long initial_homing = 0;                  // Used to Home Stepper at startup
int nextion_Fnc = 0;
int Bounce = 0;
int Tps = 0;
int Tlps_countdown = 0;
int Tlps_step_dist = 0;
int TpsD = 2000;
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

int Nextion_Fnc;
int Nextion_play;
int But_Com;
int TpsM;

int F_ResetEncoder = 1;
int Z_ResetEncoder = 1;
int P_ResetEncoder = 1;
int T_ResetEncoder = 1;

int incomingEz;
int incomingBo;
int incomingCro;
int incomingFnc;
int incomingTps;
int incomingTpsD;
int incomingPlay;
long incomingIn;
long incomingOut;
int incomingBut;
int incommingInP;
int incommingOutP;
int incommingTpsM;

//Sequencer
int k1A;                                           //Actual interpolated Accel values First Ramp
int k2A;
int k3A;
int k4A;
int k5A;
int k6A;

int k1B;                                        //Actual interpolated Accel values second ramp
int k2B;
int k3B;
int k4B;
int k5B;
int k6B;



int a1 = 0;                                          //Nextion values for Accel 1-3 one for eack key
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

int k1AH;
int k2AH;
int k3AH;
int k4AH;
int k5AH;
int k6AH;

int Tlt_k1_position;
int Tlt_k2_position;
int Tlt_k3_position;
int Tlt_k4_position;
int Tlt_k5_position;
int Tlt_k6_position;

int Pan_k1_position;
int Pan_k2_position;
int Pan_k3_position;
int Pan_k4_position;
int Pan_k5_position;
int Pan_k6_position;

int Foc_k1_position;
int Foc_k2_position;
int Foc_k3_position;
int Foc_k4_position;
int Foc_k5_position;
int Foc_k6_position;

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

int Tlt_k1_positionH;
int Tlt_k2_positionH;
int Tlt_k3_positionH;
int Tlt_k4_positionH;
int Tlt_k5_positionH;
int Tlt_k6_positionH;

int Pan_k1_positionH;
int Pan_k2_positionH;
int Pan_k3_positionH;
int Pan_k4_positionH;
int Pan_k5_positionH;
int Pan_k6_positionH;

int Foc_k1_positionH;
int Foc_k2_positionH;
int Foc_k3_positionH;
int Foc_k4_positionH;
int Foc_k5_positionH;
int Foc_k6_positionH;


int Stp_1_active;
int Stp_2_active;
int Stp_3_active;
int Stp_active = 1;

int BounceActive;
float Time2Ramp2 = 0;
int Seq_Time = 5;
int Key_Crono_time;
int Tlt_TravelDist;
int Pan_TravelDist;
int Foc_TravelDist;

int Last_Tlt_accel;
int Last_Tlt_speed;
int Last_Pan_accel;
int Last_Pan_speed;
int Last_Foc_accel;
int Last_Foc_speed;

int Base_Tlt_accel = 0;
int Base_Pan_accel = 0;
int Base_Foc_accel = 0;

int k1_k2_Dir;
int k2_k3_Dir;
int k3_k4_Dir;
int k4_k5_Dir;
int k5_k6_Dir;

int Tlt_move1_Dest;
int Tlt_move2_Dest;
int Tlt_move3_Dest;
int Tlt_move4_Dest;
int Tlt_move5_Dest;

int Pan_move1_Dest;
int Pan_move2_Dest;
int Pan_move3_Dest;
int Pan_move4_Dest;
int Pan_move5_Dest;

int Foc_move1_Dest;
int Foc_move2_Dest;
int Foc_move3_Dest;
int Foc_move4_Dest;
int Foc_move5_Dest;


int Tlt_move2_Rst;
int Tlt_move3_Rst;
int Tlt_move4_Rst;
int Tlt_move5_Rst;

int Pan_move2_Rst;
int Pan_move3_Rst;
int Pan_move4_Rst;
int Pan_move5_Rst;

int Foc_move2_Rst;
int Foc_move3_Rst;
int Foc_move4_Rst;
int Foc_move5_Rst;



int Ramp_1_Dist;
//int Ramp_1_Time;
int Ramp_2_Dist;
//int Ramp_2_Time;

int KeyDist;
int KeyA;
int KeyB;

int Accel;
int PastTime;
int ActiveMove;
int SEQmin = 4000;                                   //Min slowest speed for sequencer larger number slower speed
int SEQmoves = 2;
int Tlt_Last_key;
int Pan_Last_key;
int Foc_Last_key;
int mess ;
int Sld;                                             //Is the Slider  present 1 or 0
int TT;                                              //Is the turntable present 1 or 0
int JB;                                              //Is the Jib present 1 or 0
int PT = 1;                                          //Is the Pan Tilt present 1 or 0
int SEQ = 0;


//*****************VISCA variables

int ViscaComand;
int number;
int IPR;
int IP1 = 192;
int IP2 = 168;
int IP3 = 137;
int IP4 = 80;
int IPGW = 1;        //IP gatway 4th position
long UDP = 1259;
int LastViscaComand;
int VPanSpeed;
int VTiltSpeed;
int VFocusSpeed;
int VZoomSpeed;
int VPoseNumber;
long PA = 0; //pan angle  = value=angle x 100
long TA = 0; //Tilt angle
long ZA = 0;  //Zoom angle
long FA = 0;  //Focus angle

String PanPPPPP = "05C26"; //Left100
String TiltTTTT = "1BA5"; //Up30
int TLY = 1;
int Tally;

const byte bufferSize = 32;
char serialBuffer[bufferSize];
byte bufferIndex = 0;
char EOL = '\n';
bool hasData = false;
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

String strs[4];
int StringCount = 0;
boolean newData = false;

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
  int a1;                                                //Key Accelorations
  int a2;
  int a3;
  int a4;
  int a5;
  int a6;
  int Cro;      //7                                       //Time in seconds of move
  //int Fnc;                                                //Nection Function
  int Tps;                                                //Timlapse frames required
  long Out;                                               //Slider Out position in steps

  int InP;                                                //Inpoint button press
  int clrK;                                               //Clear all keys 1 or 0
  int OutP;     //8                                       //Out point button press
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
  //int mess;
} struct_message;

struct_message NextionValues;                             //Define structure for sending values to remote Nextion Control
struct_message incomingValues;                            //Define structure for receiving values from remote Nextion control


AS5600 F_encoder;                                           //AMS AS5600 Encoder setups   The encoders keeps track of motor positiions even when powered down for positioning
AS5600 Z_encoder;
AS5600 P_encoder;
AS5600 T_encoder;
int WIFIOUT[8];                                           //set up 5 element array for sendinf values to pantilt
//#define RXD2 16                                           //Hardware Serial2 on ESP32 Dev (must also be a common earth between nextion and esp32)
//#define TXD2 17

TaskHandle_t C1;
TaskHandle_t C2;


// Select I2C BUS
void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  //Serial.print(bus);
}





void setup() {

  //Setup for Tally LED
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(90);

  //*************************************Setup OLED****************************
  TCA9548A(4);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  //*************************************Setup a core to run Encoder****************************
  //  xTaskCreatePinnedToCore(coreas1signments, "Core_1", 10000, NULL, 2, &C1, 0);
  //  delay(100);
  //  disableCore1WDT();

  engine.init();
  stepper1 = engine.stepperConnectToPin(step1_pinSTEP);
  if (stepper1) {
    stepper1->setDirectionPin(step1_pinDIR);
    stepper1->setEnablePin(StepD);
    stepper1->setAutoEnable(false);
  }
  stepper2 = engine.stepperConnectToPin(step2_pinSTEP);
  if (stepper2) {
    stepper2->setDirectionPin(step2_pinDIR);
    stepper2->setEnablePin(StepD);
    stepper2->setAutoEnable(false);
  }
  stepper3 = engine.stepperConnectToPin(step3_pinSTEP);
  if (stepper3) {
    stepper3->setDirectionPin(step3_pinDIR);
    stepper3->setEnablePin(StepFOC);
    stepper3->setAutoEnable(false);
  }
  stepper4 = engine.stepperConnectToPin(step4_pinSTEP);
  if (stepper4) {
    stepper4->setDirectionPin(step4_pinDIR);
    stepper4->setEnablePin(StepFOC);
    stepper4->setAutoEnable(false);
  }

  Serial.begin(115200);
  //  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Wire.begin();
  Serial.println("\nRunning DigitalBird DB3 Pan Tilt Head software version 1.0\n");
  UARTport.begin(9600, SERIAL_8N1, 16, 17);

  //********************Setup Radio ESP-NOW**********************************//
  WiFi.mode(WIFI_STA);

  Serial.println("\nmacAdress is:\n");
  Serial.print(WiFi.macAddress());
  WiFi.disconnect();

  //******************Setup ESP_now***************************
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Sucess");
    esp_now_register_recv_cb(receiveCallback);
    esp_now_register_send_cb(sentCallback);
  } else {
    Serial.println("ESPNow Init Failed");
    delay(3000);
    ESP.restart();
  }

  //  pin setups

  pinMode(BatPin, INPUT);
  //pinMode(Mount_PIN, INPUT_PULLUP);                             //pulled high Mount switch
  //pinMode(DisMount_PIN, INPUT_PULLUP);                          //pulled high DisMount switch
  pinMode(Hall_Pan, INPUT_PULLUP);
  pinMode(Hall_Tilt, INPUT_PULLUP);
  pinMode(tilt_PIN, INPUT);                                     //Joystick Tilt
  pinMode(pan_PIN, INPUT);                                      //Joystick pan
  pinMode (CAM, OUTPUT);                                        //Camera Shutter
  digitalWrite(StepD, LOW);                                     //Stepper Driver Activation
  digitalWrite(StepFOC, LOW);                                   //Enable Focus motor
  digitalWrite(CAM, LOW);                                       //Camera Shutter
  stepper1->setCurrentPosition(0);                              //Set all steppers to position 0
  stepper2->setCurrentPosition(0);
  stepper3->setCurrentPosition(0);
  stepper4->setCurrentPosition(0);

  //SysMemory Recover last setup
  SysMemory.begin("ID", false);                                 //Recover the last PTZ_ID from memory before shutdown
  PTZ_ID = SysMemory.getUInt("PTZ_ID", 4);
  SysMemory.end();
  lastPTZ_ID = PTZ_ID;                                          //set last PTZ_ID for comparison when changing ID
  Serial.printf("PTZ_ID is set to: %d \n", PTZ_ID);
  //saveIP();
  Load_SysMemory();
  Serial.printf("\ncam_F_Out: %d \n", cam_F_Out);
  Serial.printf("\ncam_F_In: %d \n", cam_F_In);
  Serial.printf("IP Adress: %d.%d.%d.%d %d\n", IP1, IP2, IP3, IP4, IPGW, UDP);
  digitalWrite(StepFOC, LOW);                                   //Power UP the Focus stepper

  InitialValues();                                              //onboard Joystick calobration
  SendNextionValues();
  homeStepper();
  //*************************************Setup a core to run Encoder****************************
  xTaskCreatePinnedToCore(coreas1signments, "Core_1", 10000, NULL, 2, &C1, 0);
  delay(100);
  disableCore1WDT();
}//end setup


void loop() {
  //TA=20;
  //PA=20;
  //FA=20;
  //ZA=20;

  if (IPR == 1) {               //If a request for current network address was received from the controller send back the values
    IPR = 2;
    SendNextionValues();         //Send back the address
    IPR = 0;
  }
  if (TLY == 1) {
    switch (Tally) {
      case 0:
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
        FastLED.show(); break;
      case 1:
        leds[0] = CRGB::Blue;
        leds[1] = CRGB::Blue;
        FastLED.show(); break;
      case 2:
        leds[0] = CRGB::Blue;
        leds[1] = CRGB::Blue;
        FastLED.show(); break;
      case 3:
        leds[0] = CRGB::Red;
        leds[1] = CRGB::Red;
        FastLED.show(); break;

      case 4:
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
        FastLED.show(); break;
    }
  }



  listenForVisca();

  //IF the Jib is present then make sure that the ease value is at least 1 for all moves.
  if (JB != 0 && ease_InOut < 1) {
    ease_InOut = 1;
  }

  if ((PTZ_ID_Nex != 0) && (PTZ_ID_Nex != lastPTZ_ID)) {
    SetID();
    lastPTZ_ID = PTZ_ID;
  }
  //Serial.printf("PTZ_CAM is set to: %d \n", PTZ_Cam);
  //Serial.printf("PTZ_ID is set to: %d \n", PTZ_ID);
  //Serial.printf("LM is set to: %d \n", LM);
  //delay(1000);
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

  //ADD Back IN******************Listen for mount/Dismount button comands*********************
  if (stepper1->isRunning() || stepper2->isRunning()) {
    // delay(1);
  } else {
    ReadAnalog(); //Joystick setup
    // ReadMount();                   //Call to read mount/dismount
  }


  //*************************************Action Nextion Comands*****************************************
  if (InP != 0 && (InP != lastInP) && (PTZ_ID == 4)) {
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
  if (OutP != 0 && (OutP != lastOutP) && (PTZ_ID == 4)) {

    switch (OutP) {
      case 1:                                                 //First Press: Request a new inpoint
        OUTpointSet();
        lastOutP = OutP;
        break;
      case 2:                                                 //Second Press: Record position
        OUTpointSet();
        SendNextionValues();
        OutP = 0;                                            //Reset Inpoint press back to 0
        lastOutP = 0;
        break;
    }
  }


  if  (Nextion_play == 1 && (PTZ_ID == 4)) {                  //Play comand receved
    if (stepper1->getCurrentPosition() != (TLTin_position) || stepper2->getCurrentPosition() != (PANin_position) || stepper3->getCurrentPosition() != (FOCin_position) || stepper4->getCurrentPosition() != (ZMin_position) ) {
      digitalWrite(StepFOC, LOW);
      delay(20);
      SetSpeed();
      delay(20);
      //BatCheck();
      Start_1();
    } else {                                                   //System is at the start and ready to play
      SetSpeed();                                              // This causes reboot?????????????
      delay(20);
      if (Tps == 0) {                                          //No timelapse
        digitalWrite(CAM, HIGH);                               //Start camera
        delay (200);
        digitalWrite(CAM, LOW);                                //Start camera
        if (TLY == 1) {
          Tally = 3;                                            //Turn off the tally light
          leds[0] = CRGB::Red;
          leds[1] = CRGB::Red;
          FastLED.show();
        }
        Start_2();
      } else {                                                 //Play Timelapse
        Start_4();
      }
    }

    Nextion_play = 0;                                          //Reset play to 0
    SendNextionValues();
  }


  //**************************************Listen for PTZ Comands**********************************
  if ((PTZ_Pose != 0) && (PTZ_Cam == PTZ_ID)) {                    //If a position is requested and this devise is paired with the current PTZ camera
    lastPTZ_Pose = (PTZ_Pose);                                     //Store last PTZ_position for future save to
    PTZ_MoveP();
    PTZ_Pose = 0;
  }
  if ((PTZ_SaveP != 0) && (PTZ_Cam == PTZ_ID)) {                   //If a position save is requested and this devise is paired with the current PTZ camera
    PTZ_Save();
    PTZ_SaveP = 0;
  }

  if (LM != 0) {

    //Focus & Zoom & Tilt limits
    digitalWrite(StepFOC, HIGH);                                    //Power down the Focus & Zoom steppers for manual positioning
    digitalWrite(StepD, HIGH);                                      //Power down the Pan Tilt motors
    delay(20);
    // FS_Limits();
    while (LM == 1) {
      cam_F_In = (F_S_position);                                      //Set Focus position IN
      stepper3->setCurrentPosition(F_S_position);

      cam_Z_In = (Z_S_position);                                      //Set zoom position IN
      stepper4->setCurrentPosition(Z_S_position);
      delay(10);

      Forward_T_In = (T_S_position);                                  //Set Tilt position IN
      stepper1->setCurrentPosition(T_S_position);

    }
    SysMemory.begin("FocLmts", false);                              //save to memory
    SysMemory.putUInt("cam_F_In", cam_F_In);
    SysMemory.end();

    SysMemory.begin("ZoomLmts", false);                              //save to memory
    SysMemory.putUInt("cam_Z_In", cam_Z_In);
    SysMemory.end();

    SysMemory.begin("TiltLmts", false);                              //save to memory
    SysMemory.putUInt("Forward_T_In", Forward_T_In);
    SysMemory.end();

    while (LM == 2) {
      cam_F_Out = (F_S_position);                                     //Set position OUT
      stepper3->setCurrentPosition(F_S_position);
      delay(10);
      cam_Z_Out = (Z_S_position);                                     //Set position OUT
      stepper4->setCurrentPosition(Z_S_position);
      delay(10);
      Forward_T_Out = (T_S_position);                                  //Set Tilt position IN
      stepper1->setCurrentPosition(T_S_position);

    }
    SysMemory.begin("FocLmts", false);                                //save to memory
    SysMemory.putUInt("cam_F_Out", cam_F_Out);
    SysMemory.end();

    SysMemory.begin("ZoomLmts", false);                               //save to memory
    SysMemory.putUInt("cam_Z_Out", cam_Z_Out);
    SysMemory.end();

    SysMemory.begin("TiltLmts", false);                              //save to memory
    SysMemory.putUInt("Forward_T_Out", Forward_T_Out);
    SysMemory.end();


    if (LM == 0) {


      digitalWrite(StepFOC, LOW);                                    //Power UP the Focus stepper only for manual positioning
      digitalWrite(StepZOOM, LOW);
      digitalWrite(StepD, LOW);
      delay(20);
      stepper3->setSpeedInHz(3000);
      stepper3->setAcceleration(1000);

      stepper4->setSpeedInHz(3000);
      stepper4->setAcceleration(1000);

      stepper1->setSpeedInHz(1500);
      stepper1->setAcceleration(500);

      stepper3->moveTo(cam_F_In);
      delay(10);
      stepper4->moveTo(cam_Z_In);
      delay(10);
      stepper1->moveTo(0);                                        //Return the Tilt to home
      delay(10);

      stepper2->setCurrentPosition(P_S_position);                 //Make sure we no where Pan is even though we are not setting its liits here
      stepper2->moveTo(0);                                        //Return the Tilt to home

      while (stepper3->isRunning() || stepper4->isRunning() || stepper1->isRunning() || stepper2->isRunning() ) {
        delay(10);
      }
    }
  }

  //**************************************Listen for Sequencer commands**********************************
  if (P1 != 0 && (P1 != lastP1) && (PTZ_ID == 4)) {
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
  if (P2 != 0 && (P2 != lastP2) && (PTZ_ID == 4)) {
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
  if (P3 != 0 && (P3 != lastP3) && (PTZ_ID == 4)) {
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
  if (P4 != 0 && (P4 != lastP4) && (PTZ_ID == 4)) {
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
  if (P5 != 0 && (P5 != lastP5) && (PTZ_ID == 4)) {
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
  if (P6 != 0 && (P6 != lastP6) && (PTZ_ID == 4)) {
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

  if  (Nextion_play == 2 && (PTZ_ID == 4)) {                 //Sequencer Play comand receved
    if (stepper1->getCurrentPosition() != Tlt_k1_position || stepper2->getCurrentPosition() != Pan_k1_position || stepper3->getCurrentPosition() != Foc_k1_position)  {  // Play Button action if not at atart)
      digitalWrite(StepFOC, LOW);
      delay(20);
      Start_1();                                             //Go to start
    } else {                                                 //System is at the start and ready to play
      delay(20);
      digitalWrite(CAM, HIGH);                               //Start camera
      delay (200);
      digitalWrite(CAM, LOW);                                //Start camera
      if (TLY == 1) {
        Tally = 3;                                            //Turn off the tally light
        leds[0] = CRGB::Red;
        leds[1] = CRGB::Red;
        FastLED.show();
      }
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
  if  (stopM_play == 1 && (PTZ_ID == 4)) {
    Start_7();
  }

  //Clear keys command received
  if (clrK == 1) {
    ClearKeys();
    clrK = 0;
  }

} //end loop




//*********************************************************Start Enoders*************************************
void coreas1signments( void * pvParameters ) {
  for (;;) {
    Start_F_Encoder();
    Start_Z_Encoder();
    Start_T_Encoder();
    Start_P_Encoder();
    vTaskDelay(10);
    BatCheck();
  };
}



//*****Focus Encoder
void Start_F_Encoder() {
  TCA9548A(0);
  if (F_ResetEncoder == 1) {
    F_encoder.setZero();
    F_lastOutput = 0;
    F_S_position = 0;
    F_E_position = 0;
    F_revolutions = 0;
    F_ResetEncoder = 0;

  }
  F_output = F_encoder.getPosition();           // get the raw value of the encoder

  if ((F_lastOutput - F_output) > 2047 ) {       // check if a full rotation has been made
    F_revolutions++;
  }
  if ((F_lastOutput - F_output) < -2047 ) {
    F_revolutions--;
  }
  F_E_position = F_revolutions * 4096 + F_output;   // calculate the position the the encoder is at based off of the number of revolutions

  //Serial.println("Encoder E_Position=");
  //Serial.println(E_position);

  F_lastOutput = F_output;                      // save the last raw value for the next loop
  F_E_outputPos = F_E_position;

  F_S_position = ((F_E_position / 2.56));       //Ajust encoder to stepper values the number of steps eqiv
  F_S_position = (F_S_position * 2);            //Ajust encoder to stepper values the number of steps eqiv
  //Serial.println(F_S_position);
}

//*****Zoom Encoder
void Start_Z_Encoder() {
  TCA9548A(2);
  if (Z_ResetEncoder == 1) {
    Z_encoder.setZero();
    Z_lastOutput = 0;
    Z_S_position = 0;
    Z_E_position = 0;
    Z_revolutions = 0;
    Z_ResetEncoder = 0;

  }
  Z_output = Z_encoder.getPosition();           // get the raw value of the encoder

  if ((Z_lastOutput - Z_output) > 2047 ) {       // check if a full rotation has been made
    Z_revolutions++;
  }
  if ((Z_lastOutput - Z_output) < -2047 ) {
    Z_revolutions--;
  }
  Z_E_position = Z_revolutions * 4096 + Z_output;   // calculate the position the the encoder is at based off of the number of revolutions

  //Serial.println("Encoder E_Position=");
  //Serial.println(E_position);

  Z_lastOutput = Z_output;                      // save the last raw value for the next loop
  Z_E_outputPos = Z_E_position;

  Z_S_position = ((Z_E_position / 2.56));       //Ajust encoder to stepper values the number of steps eqiv
  Z_S_position = (Z_S_position * 2);            //Ajust encoder to stepper values the number of steps eqiv
  //Serial.println(Z_S_position);
}
//*******Tilt Encoder
void Start_T_Encoder() {
  TCA9548A(1);
  if (T_ResetEncoder == 1) {
    T_encoder.setZero();
    T_lastOutput = 0;
    T_S_position = 0;
    T_E_position = 0;
    T_revolutions = 0;
    T_ResetEncoder = 0;

  }
  T_output = T_encoder.getPosition();           // get the raw value of the encoder

  if ((T_lastOutput - T_output) > 2047 ) {       // check if a full rotation has been made
    T_revolutions++;
  }
  if ((T_lastOutput - T_output) < -2047 ) {
    T_revolutions--;
  }
  T_E_position = T_revolutions * 4096 + T_output;   // calculate the position the the encoder is at based off of the number of revolutions

  //Serial.println("Encoder E_Position=");
  //Serial.println(E_position);

  T_lastOutput = T_output;                      // save the last raw value for the next loop
  T_E_outputPos = T_E_position;

  T_S_position = ((T_E_position / 2.56));       //Ajust encoder to stepper values the number of steps eqiv


  //Serial.println(P_S_position);
  T_S_position = (T_S_position * 5.18);            //Ajust for gear ratio 5.18:1 which is to fast for the encoder on the!
  //******This is required if the motor is geared and the encoder is NOT on the back of the motor. (The AS5600 canot keep up with the fast moving motor) *****************

  if (T_output < 0) {                               //Reverse the values for the encoder position
    T_S_position = abs(T_S_position);
  } else {
    T_S_position = T_S_position - (T_S_position * 2);

  }
  //***************************************************************************************************************

}

//*******Pan Encoder
void Start_P_Encoder() {
  TCA9548A(3);
  if (P_ResetEncoder == 1) {
    P_encoder.setZero();
    P_lastOutput = 0;
    P_S_position = 0;
    P_E_position = 0;
    P_revolutions = 0;
    P_ResetEncoder = 0;

  }
  P_output = P_encoder.getPosition();           // get the raw value of the encoder

  if ((P_lastOutput - P_output) > 2047 ) {       // check if a full rotation has been made
    P_revolutions++;
  }
  if ((P_lastOutput - P_output) < -2047 ) {
    P_revolutions--;
  }
  P_E_position = P_revolutions * 4096 + P_output;   // calculate the position the the encoder is at based off of the number of revolutions

  //Serial.println("Encoder E_Position=");
  //Serial.println(E_position);

  P_lastOutput = P_output;                      // save the last raw value for the next loop
  P_E_outputPos = P_E_position;

  P_S_position = ((P_E_position / 2.56));       //Ajust encoder to stepper values the number of steps eqiv
  // P_S_position = (P_S_position * 2);            //Ajust encoder to stepper values the number of steps eqiv
  //Serial.println(P_S_position);

}
//*********************************************************INpoint set*************************************
void INpointSet() {
  // Buttonset = 1;
  //digitalWrite(StepFOC, HIGH);
  delay(10);
  switch (InP) {
    case 1:                                                       //First Press: Request a new Inpoint
      if (stepper1->getCurrentPosition() != (TLTin_position) || stepper2->getCurrentPosition() != (PANin_position) || stepper3->getCurrentPosition() != (FOCin_position) || stepper4->getCurrentPosition() != (ZMin_position)) {  //Run to In position
        stepper1->setSpeedInHz(2000);
        stepper1->setAcceleration(3000);

        stepper2->setSpeedInHz(2000);
        stepper2->setAcceleration(3000);

        stepper3->setSpeedInUs(50);
        stepper3->setAcceleration(2000);

        stepper4->setSpeedInUs(50);
        stepper4->setAcceleration(2000);




        stepper1->moveTo(TLTin_position);
        delay(10);
        stepper2->moveTo(PANin_position);
        delay(10);
        stepper3->moveTo(FOCin_position);
        delay(10);
        stepper4->moveTo(ZMin_position);

        while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() || stepper4->isRunning() ) {                          //delay until move complete
          delay(10);
        }
      }
      if (usejoy == false) {
        digitalWrite(StepFOC, HIGH);                                    //Power down the Focus & Zoom steppers only for manual positioning
        digitalWrite(StepD, HIGH);                                      //Power down the Pan & Tilt steppers only for manual positioning
        delay(10);
      }




      if (usejoy) {
        PTZ_Control();
      }
      //InP = 0;
      break;

    case 2:                                                             //Second inpoint press take the encoder values and set as inpoint
      //TLTin_position = stepper1->getCurrentPosition();                //original code without Encoders
      //PANin_position = stepper2->getCurrentPosition();
      TLTin_position = (T_S_position);
      PANin_position = (P_S_position);
      FOCin_position = (F_S_position);                                  //Only the Focus motor has an encoder this is getting the value from the encoder
      ZMin_position = (Z_S_position);


      stepper1->setCurrentPosition(T_S_position);
      stepper2->setCurrentPosition(P_S_position);
      stepper3->setCurrentPosition(F_S_position);
      stepper4->setCurrentPosition(Z_S_position);

      InP = 0;
      delay(10);

      SysMemory.begin("In_positions", false);                           //save to memory
      SysMemory.putUInt("TLTin_position", TLTin_position);
      SysMemory.putUInt("PANin_position", PANin_position);
      SysMemory.putUInt("FOCin_position", FOCin_position);
      SysMemory.putUInt("ZMin_position", ZMin_position);
      SysMemory.end();


      Start_1();                                                      //Send the stepper back to In position
      //SetSpeed();
      break;
  }
  return;
}

//*********************************************************OUTpoint set*************************************
void OUTpointSet() {

  //Serial.printf("entering Outpointset %d \n");
  delay(10);
  switch (OutP) {
    case 1:                                                           //First Press: Request a new OUTpoint
      if (stepper1->getCurrentPosition() != (TLTout_position) || stepper2->getCurrentPosition() != (PANout_position) || stepper3->getCurrentPosition() != (FOCout_position) || stepper4->getCurrentPosition() != (ZMout_position)) {

        stepper1->setSpeedInUs(200);                                 //Setup stepper speed and acceloration
        stepper1->setAcceleration(3000);
        stepper2->setSpeedInUs(200);
        stepper2->setAcceleration(3000);
        stepper3->setSpeedInUs(50);
        stepper3->setAcceleration(2000);
        stepper4->setSpeedInUs(50);
        stepper4->setAcceleration(2000);

        stepper1->moveTo(TLTout_position);                            //Start moves
        delay(10);
        stepper2->moveTo(PANout_position);
        delay(10);
        stepper3->moveTo(FOCout_position);
        delay(10);
        stepper4->moveTo(ZMout_position);

        while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() || stepper4->isRunning() ) {                          //delay until move complete
          delay(10);
        }
      }
      if (usejoy == false) {
        digitalWrite(StepFOC, HIGH);                                    //Power down the Focus & Zoom steppers only for manual positioning
        digitalWrite(StepD, HIGH);                                      //Power down the Pan & Tilt steppers only for manual positioning
        delay(10);
      }
      if (usejoy) {
        PTZ_Control();
      }
      //Serial.println("leaving out press1");
      //OutP = 0;
      break;

    case 2:                                                           //Second outpoint press take the encoder values and set as inpoint
      //      TLTout_position = stepper1->getCurrentPosition();                //Original code withought encoders
      //      PANout_position = stepper2->getCurrentPosition();
      TLTout_position = (T_S_position);
      PANout_position = (P_S_position);
      FOCout_position = (F_S_position);
      ZMout_position = (Z_S_position);

      stepper1->setCurrentPosition(T_S_position);
      stepper2->setCurrentPosition(P_S_position);
      stepper3->setCurrentPosition(F_S_position);
      stepper4->setCurrentPosition(Z_S_position);
      OutP = 0;

      SysMemory.begin("Out_positions", false);                         //save to memory
      SysMemory.putUInt("TLTout_position", TLTout_position);
      SysMemory.putUInt("PANout_position", PANout_position);
      SysMemory.putUInt("FOCout_position", FOCout_position);
      SysMemory.putUInt("ZMout_position", ZMout_position);
      SysMemory.end();
      Serial.print("New Tilt Out= ");
      Serial.println(TLTout_position);
      Serial.print("New Tilt IN= ");
      Serial.println(TLTin_position);
      Start_1();                                                      //Send the stepper back to In position
      delay(10);
      //SendNextionValues();                                          //Update nextion with new inpoint values


      break;
  }
  return;
}


//*********************************************************Move to pre recorded PTZ pose*************************************
void PTZ_MoveP() {
  delay(10);

  switch (lastPTZ_Pose) {
    case 1:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P1_T);
        P_position = (P1_P);
        F_position = (P1_F);
        Z_position = (P1_Z);
      }
      break;
    case 2:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P2_T);
        P_position = (P2_P);
        F_position = (P2_F);
        Z_position = (P2_Z);
      }
      break;
    case 3:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P3_T);
        P_position = (P3_P);
        F_position = (P3_F);
        Z_position = (P3_Z);
      }
      break;
    case 4:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P4_T);
        P_position = (P4_P);
        F_position = (P4_F);
        Z_position = (P4_Z);
      }
      break;
    case 5:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P5_T);
        P_position = (P5_P);
        F_position = (P5_F);
        Z_position = (P5_Z);
      }
      break;
    case 6:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P6_T);
        P_position = (P6_P);
        F_position = (P6_F);
        Z_position = (P6_Z);
      }
      break;

    case 7:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P7_T);
        P_position = (P7_P);
        F_position = (P7_F);
        Z_position = (P7_Z);
      }
      break;
    case 8:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P8_T);
        P_position = (P8_P);
        F_position = (P8_F);
        Z_position = (P8_Z);
      }
      break;

    case 9:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P9_T);
        P_position = (P9_P);
        F_position = (P9_F);
        Z_position = (P9_Z);
      }
      break;
    case 10:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P10_T);
        P_position = (P10_P);
        F_position = (P10_F);
        Z_position = (P10_Z);
      }
      break;
    case 11:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P11_T);
        P_position = (P11_P);
        F_position = (P11_F);
        Z_position = (P11_Z);
      }
      break;

    case 12:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P12_T);
        P_position = (P12_P);
        F_position = (P12_F);
        Z_position = (P12_Z);
      }
      break;
    case 13:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P13_T);
        P_position = (P13_P);
        F_position = (P13_F);
        Z_position = (P13_Z);
      }
      break;

    case 14:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P14_T);
        P_position = (P14_P);
        F_position = (P14_F);
        Z_position = (P14_Z);
      }
      break;
    case 15:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P15_T);
        P_position = (P15_P);
        F_position = (P15_F);
        Z_position = (P15_Z);
      }
      break;

    case 16:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P16_T);
        P_position = (P16_P);
        F_position = (P16_F);
        Z_position = (P16_Z);
      }
      break;

  }


  //******************Stepper moves************************************************************
  if (stepper2->getCurrentPosition() != (P_position) && (P_position != 10)) {    //Run to  position

    stepper1->setSpeedInHz(5000);                           //Tilt
    stepper1->setAcceleration(2000);

    stepper2->setSpeedInHz(3000);                           //Pan
    stepper2->setAcceleration(1000);

    stepper3->setSpeedInHz(6000);                           //Focus
    stepper3->setAcceleration(2000);

    stepper4->setSpeedInHz(6000);                           //Zoom
    stepper4->setAcceleration(2000);


    stepper1->moveTo(T_position);
    delay(10);
    stepper2->moveTo(P_position);
    delay(10);
    stepper3->moveTo(F_position);
    delay(10);
    stepper4->moveTo(Z_position);

    while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() || stepper4->isRunning() ) {
      delay(10);
    }
  }

  return;
}




void PTZ_Save() {

  switch (lastPTZ_Pose) {
    case 1:
      if (PTZ_Cam == PTZ_ID) {

        P1_T = (stepper1->getCurrentPosition());
        P1_P = (stepper2->getCurrentPosition());
        P1_F = (stepper3->getCurrentPosition());
        P1_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P1pos", false);                         //save to memory
        SysMemory.putUInt("P1_T", P1_T);
        SysMemory.putUInt("P1_P", P1_P);
        SysMemory.putUInt("P1_F", P1_F);
        SysMemory.putUInt("P1_Z", P1_Z);
        SysMemory.end();
      }
      break;
    case 2:
      if (PTZ_Cam == PTZ_ID) {
        P2_T = (stepper1->getCurrentPosition());
        P2_P = (stepper2->getCurrentPosition());
        P2_F = (stepper3->getCurrentPosition());
        P2_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P2pos", false);                         //save to memory
        SysMemory.putUInt("P2_T", P2_T);
        SysMemory.putUInt("P2_P", P2_P);
        SysMemory.putUInt("P2_F", P2_F);
        SysMemory.putUInt("P2_Z", P2_Z);
        SysMemory.end();
      }
      break;
    case 3:
      if (PTZ_Cam == PTZ_ID) {
        P3_T = (stepper1->getCurrentPosition());
        P3_P = (stepper2->getCurrentPosition());
        P3_F = (stepper3->getCurrentPosition());
        P3_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P3pos", false);                         //save to memory
        SysMemory.putUInt("P3_T", P3_T);
        SysMemory.putUInt("P3_P", P3_P);
        SysMemory.putUInt("P3_F", P3_F);
        SysMemory.putUInt("P3_Z", P3_Z);
        SysMemory.end();
      }
      break;
    case 4:
      if (PTZ_Cam == PTZ_ID) {
        P4_T = (stepper1->getCurrentPosition());
        P4_P = (stepper2->getCurrentPosition());
        P4_F = (stepper3->getCurrentPosition());
        P4_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P4pos", false);                         //save to memory
        SysMemory.putUInt("P4_T", P4_T);
        SysMemory.putUInt("P4_P", P4_P);
        SysMemory.putUInt("P4_F", P4_F);
        SysMemory.putUInt("P4_Z", P4_Z);
        SysMemory.end();
      }
      break;
    case 5:
      if (PTZ_Cam == PTZ_ID) {
        P5_T = (stepper1->getCurrentPosition());
        P5_P = (stepper2->getCurrentPosition());
        P5_F = (stepper3->getCurrentPosition());
        P5_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P5pos", false);                         //save to memory
        SysMemory.putUInt("P5_T", P5_T);
        SysMemory.putUInt("P5_P", P5_P);
        SysMemory.putUInt("P5_F", P5_F);
        SysMemory.putUInt("P5_Z", P5_Z);
        SysMemory.end();
      }
      break;
    case 6:
      if (PTZ_Cam == PTZ_ID) {
        P6_T = (stepper1->getCurrentPosition());
        P6_P = (stepper2->getCurrentPosition());
        P6_F = (stepper3->getCurrentPosition());
        P6_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P6pos", false);                         //save to memory
        SysMemory.putUInt("P6_T", P6_T);
        SysMemory.putUInt("P6_P", P6_P);
        SysMemory.putUInt("P6_F", P6_F);
        SysMemory.putUInt("P6_Z", P6_Z);
        SysMemory.end();
      }
      break;
    case 7:
      if (PTZ_Cam == PTZ_ID) {
        P7_T = (stepper1->getCurrentPosition());
        P7_P = (stepper2->getCurrentPosition());
        P7_F = (stepper3->getCurrentPosition());
        P7_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P7pos", false);                         //save to memory
        SysMemory.putUInt("P7_T", P7_T);
        SysMemory.putUInt("P7_P", P7_P);
        SysMemory.putUInt("P7_F", P7_F);
        SysMemory.putUInt("P7_Z", P7_Z);
        SysMemory.end();
      }
      break;
    case 8:
      if (PTZ_Cam == PTZ_ID) {
        P8_T = (stepper1->getCurrentPosition());
        P8_P = (stepper2->getCurrentPosition());
        P8_F = (stepper3->getCurrentPosition());
        P8_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P8pos", false);                         //save to memory
        SysMemory.putUInt("P8_T", P8_T);
        SysMemory.putUInt("P8_P", P8_P);
        SysMemory.putUInt("P8_F", P8_F);
        SysMemory.putUInt("P8_Z", P8_Z);
        SysMemory.end();
      }
      break;
    case 9:
      if (PTZ_Cam == PTZ_ID) {
        P9_T = (stepper1->getCurrentPosition());
        P9_P = (stepper2->getCurrentPosition());
        P9_F = (stepper3->getCurrentPosition());
        P9_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P9pos", false);                         //save to memory
        SysMemory.putUInt("P9_T", P9_T);
        SysMemory.putUInt("P9_P", P9_P);
        SysMemory.putUInt("P9_F", P9_F);
        SysMemory.putUInt("P9_Z", P9_Z);
        SysMemory.end();
      }
      break;
    case 10:
      if (PTZ_Cam == PTZ_ID) {
        P10_T = (stepper1->getCurrentPosition());
        P10_P = (stepper2->getCurrentPosition());
        P10_F = (stepper3->getCurrentPosition());
        P10_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P10pos", false);                         //save to memory
        SysMemory.putUInt("P10_T", P10_T);
        SysMemory.putUInt("P10_P", P10_P);
        SysMemory.putUInt("P10_F", P10_F);
        SysMemory.putUInt("P10_Z", P10_Z);
        SysMemory.end();
      }
      break;
    case 11:
      if (PTZ_Cam == PTZ_ID) {
        P11_T = (stepper1->getCurrentPosition());
        P11_P = (stepper2->getCurrentPosition());
        P11_F = (stepper3->getCurrentPosition());
        P11_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P11pos", false);                         //save to memory
        SysMemory.putUInt("P11_T", P11_T);
        SysMemory.putUInt("P11_P", P11_P);
        SysMemory.putUInt("P11_F", P11_F);
        SysMemory.putUInt("P11_Z", P11_Z);
        SysMemory.end();
      }
      break;
    case 12:
      if (PTZ_Cam == PTZ_ID) {
        P12_T = (stepper1->getCurrentPosition());
        P12_P = (stepper2->getCurrentPosition());
        P12_F = (stepper3->getCurrentPosition());
        P12_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P12pos", false);                         //save to memory
        SysMemory.putUInt("P12_T", P12_T);
        SysMemory.putUInt("P12_P", P12_P);
        SysMemory.putUInt("P12_F", P12_F);
        SysMemory.putUInt("P12_Z", P12_Z);
        SysMemory.end();
      }
      break;
    case 13:
      if (PTZ_Cam == PTZ_ID) {
        P13_T = (stepper1->getCurrentPosition());
        P13_P = (stepper2->getCurrentPosition());
        P13_F = (stepper3->getCurrentPosition());
        P13_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P13pos", false);                         //save to memory
        SysMemory.putUInt("P13_T", P13_T);
        SysMemory.putUInt("P13_P", P13_P);
        SysMemory.putUInt("P13_F", P13_F);
        SysMemory.putUInt("P13_Z", P13_Z);
        SysMemory.end();
      }
      break;
    case 14:
      if (PTZ_Cam == PTZ_ID) {
        P14_T = (stepper1->getCurrentPosition());
        P14_P = (stepper2->getCurrentPosition());
        P14_F = (stepper3->getCurrentPosition());
        P14_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P14pos", false);                         //save to memory
        SysMemory.putUInt("P14_T", P14_T);
        SysMemory.putUInt("P14_P", P14_P);
        SysMemory.putUInt("P14_F", P14_F);
        SysMemory.putUInt("P14_Z", P14_Z);
        SysMemory.end();
      }
      break;
    case 15:
      if (PTZ_Cam == PTZ_ID) {
        P15_T = (stepper1->getCurrentPosition());
        P15_P = (stepper2->getCurrentPosition());
        P15_F = (stepper3->getCurrentPosition());
        P15_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P15pos", false);                         //save to memory
        SysMemory.putUInt("P15_T", P15_T);
        SysMemory.putUInt("P15_P", P15_P);
        SysMemory.putUInt("P15_F", P15_F);
        SysMemory.putUInt("P15_Z", P15_Z);
        SysMemory.end();
      }
      break;
    case 16:
      if (PTZ_Cam == PTZ_ID) {
        P16_T = (stepper1->getCurrentPosition());
        P16_P = (stepper2->getCurrentPosition());
        P16_F = (stepper3->getCurrentPosition());
        P16_Z = (stepper4->getCurrentPosition());

        SysMemory.begin("P16pos", false);                         //save to memory
        SysMemory.putUInt("P16_T", P16_T);
        SysMemory.putUInt("P16_P", P16_P);
        SysMemory.putUInt("P16_F", P16_F);
        SysMemory.putUInt("P16_Z", P16_Z);
        SysMemory.end();
      }
      break;
  }
}



//*********************************************************k1set*************************************
void k1set() {


  switch (P1) {
    case 1:                                                      //First Press: Request a new position
      if (stepper1->getCurrentPosition() != (Tlt_k1_position) || stepper2->getCurrentPosition() != (Pan_k1_position) || stepper3->getCurrentPosition() != (Foc_k1_position)) { //Run to  position

        stepper1->setSpeedInUs(200);
        stepper1->setAcceleration(3000);
        stepper1->moveTo(Tlt_k1_position);
        delay(10);
        stepper2->setSpeedInUs(200);
        stepper2->setAcceleration(3000);
        stepper2->moveTo(Pan_k1_position);
        delay(10);
        stepper3->setSpeedInUs(50);
        stepper3->setAcceleration(2000);
        stepper3->moveTo(Foc_k1_position);

        while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning()) {                      //delay until move complete
          delay(10);
        }

      }
      delay(50);
      if (usejoy == false) {
        digitalWrite(StepFOC, HIGH);                                     //Power down the Focusstepper for manual positioning
      }
      if (usejoy) {
        PTZ_Control();
      }
      // P1 = 0;
      break;

    case 2:                                                            //Second press take the encoder values and set as new position
      Tlt_k1_position = stepper1->getCurrentPosition();                //Burn in positions
      Pan_k1_position = stepper2->getCurrentPosition();
      Foc_k1_position = (F_S_position);
      //      Zoom_k1_position= (F_S_position);

      SysMemory.begin("k1_pos", false);                               //save to memory
      SysMemory.putUInt("Tlt_k1_pos", Tlt_k1_position);
      SysMemory.putUInt("Pan_k1_pos", Pan_k1_position);
      SysMemory.putUInt("Foc_k1_pos", Foc_k1_position);
      //SysMemory.putUInt("Foc_k1_pos", Foc_k1_position);
      SysMemory.end();

      stepper3->setCurrentPosition(F_S_position);
      P1 = 0;
      digitalWrite(StepFOC, LOW);                                     //Power UP the stepper
      delay(100);
      SendNextionValues();                                            //Update nextion with new inpoint values
      break;
  }
  return;
}
//*********************************************************K2set*************************************
void k2set() {

  delay(10);
  switch (P2) {
    case 1:                                                                                                 //First press send steppers to key
      if (stepper1->getCurrentPosition() != (Tlt_k2_position) || stepper2->getCurrentPosition() != (Pan_k2_position) || stepper3->getCurrentPosition() != (Foc_k2_position)) { //Run to  position
        if (Tlt_k2_position == 0 && Pan_k2_position == 0 && Foc_k2_position == 0) {                                 //Do not move
        } else {

          stepper1->setSpeedInUs(200);
          stepper1->setAcceleration(3000);
          stepper1->moveTo(Tlt_k2_position);
          delay(10);
          stepper2->setSpeedInUs(200);
          stepper2->setAcceleration(3000);
          stepper2->moveTo(Pan_k2_position);
          delay(10);
          stepper3->setSpeedInUs(50);
          stepper3->setAcceleration(2000);
          stepper3->moveTo(Foc_k2_position);
          while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning()) {                      //delay until move complete
            delay(10);
          }
        }
      }
      delay(50);
      if (usejoy == false) {
        digitalWrite(StepFOC, HIGH);                                     //Power down the Focusstepper for manual positioning
      }
      if (usejoy) {
        PTZ_Control();
      }
      //P2 = 0;
      break;

    case 2:                                                            //Second press take the encoder values and set as new position
      Tlt_k2_position = stepper1->getCurrentPosition();                //Burn in positions
      Pan_k2_position = stepper2->getCurrentPosition();
      Foc_k2_position = (F_S_position);
      //Zoom_k2_position = (F_S_position);

      SysMemory.begin("k2_pos", false);                               //save to memory
      SysMemory.putUInt("Tlt_k2_pos", Tlt_k2_position);
      SysMemory.putUInt("Pan_k2_pos", Pan_k2_position);
      SysMemory.putUInt("Foc_k2_pos", Foc_k2_position);
      //SysMemory.putUInt("Foc_k2_pos", Foc_k2_position);
      SysMemory.end();

      stepper3->setCurrentPosition(F_S_position);
      P2 = 0;
      digitalWrite(StepFOC, LOW);                                     //Power UP the Focus stepper
      delay(100);
      SendNextionValues();                                            //Update nextion with new inpoint values
      break;
  }
  return;
}
//*********************************************************K3set*************************************
void k3set() {

  delay(10);
  switch (P3) {
    case 1:
      if (stepper1->getCurrentPosition() != (Tlt_k3_position) || stepper2->getCurrentPosition() != (Pan_k3_position) || stepper3->getCurrentPosition() != (Foc_k3_position)) { //Run to  position
        if (Tlt_k3_position == 0 && Pan_k3_position == 0 && Foc_k3_position == 0) { //Do not move
        } else {
          stepper1->setSpeedInUs(200);
          stepper1->setAcceleration(3000);
          stepper1->moveTo(Tlt_k3_position);
          delay(10);
          stepper2->setSpeedInUs(200);
          stepper2->setAcceleration(3000);
          stepper2->moveTo(Pan_k3_position);
          delay(10);
          stepper3->setSpeedInUs(50);
          stepper3->setAcceleration(2000);
          stepper3->moveTo(Foc_k3_position);


          while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning()) {                      //delay until move complete
            delay(10);
          }
        }
      }
      delay(50);
      if (usejoy == false) {
        digitalWrite(StepFOC, HIGH);                                     //Power down the Focusstepper for manual positioning
      }
      if (usejoy) {
        PTZ_Control();
      }
      //P3 = 0;
      break;

    case 2:                                                            //Second press take the encoder values and set as new position
      Tlt_k3_position = stepper1->getCurrentPosition();                //Burn in positions
      Pan_k3_position = stepper2->getCurrentPosition();
      Foc_k3_position = (F_S_position);
      //Foc_k3_position = (F_S_position);

      SysMemory.begin("k3_pos", false);                               //save to memory
      SysMemory.putUInt("Tlt_k3_pos", Tlt_k3_position);
      SysMemory.putUInt("Pan_k3_pos", Pan_k3_position);
      SysMemory.putUInt("Foc_k3_pos", Foc_k3_position);
      //SysMemory.putUInt("Foc_k3_pos", Foc_k3_position);
      SysMemory.end();
      stepper3->setCurrentPosition(F_S_position);
      P3 = 0;
      digitalWrite(StepFOC, LOW);                                     //Power UP the Focus stepper
      delay(100);
      SendNextionValues();                                            //Update nextion with new inpoint values
      break;
  }
  return;
}
//*********************************************************K4set*************************************
void k4set() {

  delay(10);
  switch (P4) {
    case 1:
      if (stepper1->getCurrentPosition() != (Tlt_k4_position) || stepper2->getCurrentPosition() != (Pan_k4_position) || stepper3->getCurrentPosition() != (Foc_k4_position)) {
        if (Tlt_k4_position == 0 && Pan_k4_position == 0 && Foc_k4_position == 0) {           //Do not move
        } else {                                                                               //Run to  position
          stepper1->setSpeedInUs(200);
          stepper1->setAcceleration(3000);
          stepper1->moveTo(Tlt_k4_position);
          delay(10);
          stepper2->setSpeedInUs(200);
          stepper2->setAcceleration(3000);
          stepper2->moveTo(Pan_k4_position);
          delay(10);
          stepper3->setSpeedInUs(50);
          stepper3->setAcceleration(2000);
          stepper3->moveTo(Foc_k4_position);

          while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning()) {                      //delay until move complete
            delay(10);
          }
        }
      }
      delay(50);
      if (usejoy == false) {
        digitalWrite(StepFOC, HIGH);                                     //Power down the Focusstepper for manual positioning
      }
      if (usejoy) {
        PTZ_Control();
      }
      //P4 = 0;
      break;

    case 2:                                                            //Second press take the encoder values and set as new position
      Tlt_k4_position = stepper1->getCurrentPosition();                //Burn in positions
      Pan_k4_position = stepper2->getCurrentPosition();
      Foc_k4_position = (F_S_position);
      //Zoom_k4_position = (F_S_position);
      SysMemory.begin("k4_pos", false);                               //save to memory
      SysMemory.putUInt("Tlt_k4_pos", Tlt_k4_position);
      SysMemory.putUInt("Pan_k4_pos", Pan_k4_position);
      SysMemory.putUInt("Foc_k4_pos", Foc_k4_position);
      //SysMemory.putUInt("Zoom_k4_pos", Zoom_k4_position);
      SysMemory.end();

      stepper3->setCurrentPosition(F_S_position);
      P4 = 0;

      digitalWrite(StepFOC, LOW);                                     //Power UP the Focus stepper
      delay(100);
      SendNextionValues();                                            //Update nextion with new inpoint values

      break;
  }
  return;
}
//*********************************************************K5set*************************************
void k5set() {

  delay(10);
  switch (P5) {
    case 1:

      if (stepper1->getCurrentPosition() != (Tlt_k5_position) || stepper2->getCurrentPosition() != (Pan_k5_position) || stepper3->getCurrentPosition() != (Foc_k5_position)) {
        if (Tlt_k5_position == 0 && Pan_k5_position == 0 && Foc_k5_position == 0) {           //Do not move
        } else {                                                                               //Run to  position
          stepper1->setSpeedInUs(200);
          stepper1->setAcceleration(3000);
          stepper1->moveTo(Tlt_k5_position);
          delay(10);
          stepper2->setSpeedInUs(200);
          stepper2->setAcceleration(3000);
          stepper2->moveTo(Pan_k5_position);
          delay(10);
          stepper3->setSpeedInUs(50);
          stepper3->setAcceleration(2000);
          stepper3->moveTo(Foc_k5_position);

          while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning()) {                      //delay until move complete
            delay(10);
          }
        }
      }
      delay(100);
      if (usejoy == false) {
        digitalWrite(StepFOC, HIGH);                                     //Power down the Focusstepper for manual positioning
      }
      if (usejoy) {
        PTZ_Control();
      }
      //P5 = 0;
      break;

    case 2:                                                            //Second press take the encoder values and set as new position
      Tlt_k5_position = stepper1->getCurrentPosition();                //Burn in positions
      Pan_k5_position = stepper2->getCurrentPosition();
      Foc_k5_position = (F_S_position);
      //Zoom_k5_position = (F_S_position);

      SysMemory.begin("k5_pos", false);                               //save to memory
      SysMemory.putUInt("Tlt_k5_pos", Tlt_k5_position);
      SysMemory.putUInt("Pan_k5_pos", Pan_k5_position);
      SysMemory.putUInt("Foc_k5_pos", Foc_k5_position);
      //SysMemory.putUInt("Foc_k5_pos", Foc_k5_position);
      SysMemory.end();


      stepper3->setCurrentPosition(F_S_position);
      P5 = 0;


      digitalWrite(StepFOC, LOW);                                     //Power UP the Focus stepper
      delay(50);
      SendNextionValues();                                            //Update nextion with new inpoint values
      delay(50);
      break;
  }
  return;
}
//*********************************************************K6set*************************************
void k6set() {

  delay(10);
  switch (P6) {
    case 1:
      if (stepper1->getCurrentPosition() != (Tlt_k6_position) || stepper2->getCurrentPosition() != (Pan_k6_position) || stepper3->getCurrentPosition() != (Foc_k6_position)) {
        if (Tlt_k6_position == 0 && Pan_k6_position == 0 && Foc_k6_position == 0) {           //Do not move
        } else {                                                                                //Run to  position
          stepper1->setSpeedInUs(200);
          stepper1->setAcceleration(3000);
          stepper1->moveTo(Tlt_k6_position);
          delay(10);
          stepper2->setSpeedInUs(200);
          stepper2->setAcceleration(3000);
          stepper2->moveTo(Pan_k6_position);
          delay(10);
          stepper3->setSpeedInUs(50);
          stepper3->setAcceleration(2000);
          stepper3->moveTo(Foc_k6_position);

          while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning()) {                      //delay until move complete
            delay(10);
          }
        }
      }
      delay(50);
      if (usejoy == false) {
        digitalWrite(StepFOC, HIGH);                                     //Power down the Focusstepper for manual positioning
      }
      if (usejoy) {
        PTZ_Control();
      }
      // P6 = 0;
      break;

    case 2:                                                            //Second press take the encoder values and set as new position
      Tlt_k6_position = stepper1->getCurrentPosition();                //Burn in positions
      Pan_k6_position = stepper2->getCurrentPosition();
      Foc_k6_position = (F_S_position);
      //Zoom_k6_position = (F_S_position);
      SysMemory.begin("k6_pos", false);                               //save to memory
      SysMemory.putUInt("Tlt_k6_pos", Tlt_k6_position);
      SysMemory.putUInt("Pan_k6_pos", Pan_k6_position);
      SysMemory.putUInt("Foc_k6_pos", Foc_k6_position);
      //SysMemory.putUInt("Foc_k6_pos", Foc_k6_position);
      SysMemory.end();

      stepper3->setCurrentPosition(F_S_position);
      P6 = 0;


      digitalWrite(StepFOC, LOW);                                     //Power UP the Focus stepper
      delay(100);
      SendNextionValues();                                            //Update nextion with new inpoint values
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

  //************************Read in all current Nextion values************************************************
  LM = int(incomingValues.LM);                    //Focus/zoom limit set 1-6
  Rec = int(incomingValues.Rc);                  //Record button state for PTZ
  a1 = int(incomingValues.a1);
  a2 = int(incomingValues.a2);
  a3 = int(incomingValues.a3);
  a4 = int(incomingValues.a4);
  a5 = int(incomingValues.a5);
  a6 = int(incomingValues.a6);
  P1 = int(incomingValues.P1);
  P2 = int(incomingValues.P2);
  P3 = int(incomingValues.P3);
  P4 = int(incomingValues.P4);
  P5 = int(incomingValues.P5);
  P6 = int(incomingValues.P6);
  s1 = int(incomingValues.s1);                   //Sequencer keys % speed
  if (s1 == 100) {
    s1 = 99;
  }
  s2 = int(incomingValues.s2);
  if (s2 == 100) {
    s2 = 99;
  }
  s3 = int(incomingValues.s3);
  if (s3 == 100) {
    s3 = 99;
  }
  s4 = int(incomingValues.s4);
  if (s4 == 100) {
    s4 = 99;
  }
  if (s4 == 0) {                                  //If a key was deleted reset the key positions to 0
    Tlt_k4_position = 0;
    Pan_k4_position = 0;
    Foc_k4_position = 0;
  }
  s5 = int(incomingValues.s5);
  if (s5 == 100) {
    s5 = 99;
  }
  if (s5 == 0) {
    Tlt_k5_position = 0;
    Pan_k5_position = 0;
    Foc_k5_position = 0;
  }
  s6 = int(incomingValues.s6);
  if (s6 == 100) {
    s6 = 99;
  }
  if (s6 == 0) {
    Tlt_k6_position = 0;
    Pan_k6_position = 0;
    Foc_k6_position = 0;
  }

  Snd = int(incomingValues.Sld);                //Senders Ident 1=Controller 2=Slider 3=PanTilt 4=TurnT 5=Jib
  switch (Snd) {                                //Only Change the Device state if the message was sent by the device
    case 2:
      Sld = int(incomingValues.Sld);            //Slider available 1 or 0
      break;
    case 4:
      TT = int(incomingValues.TT);               //Turntable available 1 or 0
      break;
    case 5:
      JB = int(incomingValues.JB);                 //Jib available 1 or 0
      break;
  }

  ease_InOut = int(incomingValues.Ez);          //Acceleration control
  Bounce = int(incomingValues.Bo);              //Number of bounces
  Crono_time = int(incomingValues.Cro);         //Time in sec of system move
  //Nextion_Fnc = int(incomingValues.Fnc);      //Current Func of nextion display
  Tps = int(incomingValues.Tps);                //Timlapse frames
  TpsD = int(incomingValues.TpsD);              //Timplapse delay or shutter open time in Bulb mode
  TpsM = int(incomingValues.TpsM);              //Timelapse move comand used to trigger next move from nextion
  Nextion_play = int(incomingValues.Play);      //playbutton pressed on nextion
  But_Com = int(incomingValues.But);           //playbutton pressed on Slave
  InP = int(incomingValues.InP);               //Inpoint button state 0-2
  OutP = int(incomingValues.OutP);             //Outpoint button state 0-2
  Joy_Pan_Speed = int(incomingValues.Ps);      //Joystick Pan PTZ Controler speed
  Joy_Pan_Accel = int(incomingValues.Ma)/10 ;   //Joystick Pan PTZ Controler Acceloration

  Joy_Tilt_Speed = int(incomingValues.Ts);      //Joystick Pan PTZ Controler speed
  Joy_Tilt_Accel = int(incomingValues.Ma);      //Joystick Pan PTZ Controler Acceloration

  Joy_Focus_Speed = int(incomingValues.Fs);      //Joystick Pan PTZ Controler speed
  Joy_Focus_Accel = int(incomingValues.Ma);      //Joystick Pan PTZ Controler Acceloration

  Joy_Zoom_Speed = int(incomingValues.Zs);      //Joystick Pan PTZ Controler speed
  Joy_Zoom_Accel = int(incomingValues.Ma);      //Joystick Pan PTZ Controler Acceloration

  PTZ_ID_Nex = int(incomingValues.ID);           //PTZ messagefor ID
  PTZ_Cam = int(incomingValues.CA);              //Current PTZ camera
  PTZ_SaveP = int(incomingValues.SP);            //PTZ save position 1 or 0
  PTZ_Pose = int(incomingValues.Pz);             //Current PTZ pose 1-6

  stopM_play = int(incomingValues.SM);           //Stopmotion Trigger SM
  stopM_frames = int(incomingValues.Tps);
  stopM_cancel = int(incomingValues.SC);         //Stopmmotion cancel SC
  usejoy = int(incomingValues.Jb);               //Joystick button state 1 or 0
  clrK = int(incomingValues.clrK);               //Command to clear all keys 1 or 0
  TLY = int(incomingValues.TLY);                //Command to clear all keys 1 or 0
  IPR = int(incomingValues.IPR);
  if (int(incomingValues.IP1) > 0 && IPR == 3) { //If the incomming IP is from the controler and is not Zero then adopt the new address
    IP1 = int(incomingValues.IP1);
    IP2 = int(incomingValues.IP2);
    IP3 = int(incomingValues.IP3);
    IP4 = int(incomingValues.IP4);
    IPGW = int(incomingValues.IPGW);
    UDP = int(incomingValues.UDP);
    saveIP();
    Serial.printf("IP Adress: %d.%d.%d.%d %d\n", IP1, IP2, IP3, IP4, IPGW, UDP);

    IPR = 0;                                     //Set IPR back to 0
  }

  //if (abs(Joy_Tilt_Speed)>150){                     //If Joystick commands are comming from the WIFI controler make sure the directions are not reversed
  //  T_Rev=0;
  //}

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
  // Serial.print("Last Packet Sent to: ");
  // Serial.println(macStr);
  // Serial.print("Last Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
  /*uint8_t peerAddress[] = {0x3C, 0x81, 0xBF, 0x47, 0xA5, 0xC0};
    esp_now_peer_info_t peerInfo = {};
    memcpy(&peerInfo.peer_addr, peerAddress, 6);
    if (!esp_now_is_peer_exist(peerAddress))
    {
    esp_now_add_peer(&peerInfo);
    }
    esp_err_t result = esp_now_send(peerAddress, (const uint8_t *)message.c_str(), message.length());*/
  if (result == ESP_OK)
  {
    //Serial.println("Broadcast message success");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_INIT)
  {
    //Serial.println("ESPNOW not Init.");
  }
  else if (result == ESP_ERR_ESPNOW_ARG)
  {
    //Serial.println("Invalid Argument");
  }
  else if (result == ESP_ERR_ESPNOW_INTERNAL)
  {
    //Serial.println("Internal Error");
  }
  else if (result == ESP_ERR_ESPNOW_NO_MEM)
  {
    //Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    // Serial.println("Peer not found.");
  }
  else
  {
    //Serial.println("Unknown error");
  }
}

void SendNextionValues() {
  broadcast ("");
  NextionValues.Snd = 3;                    //Tells the controller who sent the message 1=Controller 2=Slider 3=PanTilt 4=Turntable 5=Jib
  NextionValues.Ez = ease_InOut;            // Acceloration control
  NextionValues.Bo = Bounce;                // Number of bounces
  NextionValues.Cro = Crono_time;           // Time in sec of system move
  //NextionValues.Fnc = Nextion_Fnc;        // Current Func of nextion display
  NextionValues.Tps = Tps;                  // Timlapse frames
  //NextionValues.TpsD = TpsD;              // Timplapse delay or shutter open time in Bulb mode
  //NextionValues.TpsM = TpsM;              // Timelapse move command
  //NextionValues.In = PANin_position;      // InPoint step position value
  //NextionValues.Out = PANout_position;    // OutPoint Step position value
  NextionValues.Play = Nextion_play;        // play comand
  NextionValues.But = But_Com;              // Button reply
  NextionValues.InP = InP;                  // Inpoint button state 0-2
  NextionValues.OutP = OutP;                // Outpoint button state 0-2
  //NextionValues.Sld = Sld;                // Slider present 0-1
  //NextionValues.TT = TT;                  // Turntable present 0-1
  NextionValues.PT = PT;                    // Pan Tilt present 0-1 or 2 which Prevents cumulative error in bouncespeed and controls interconnective moves

  NextionValues.s1 = s1;                    // Sequencer key speeds
  NextionValues.s2 = s2;
  NextionValues.s3 = s3;
  NextionValues.s4 = s4;
  NextionValues.s5 = s5;
  NextionValues.s6 = s6;
  NextionValues.P1 = P1;                    // Sequencer key button press 0-2
  NextionValues.P2 = P2;
  NextionValues.P3 = P3;
  NextionValues.P4 = P4;
  NextionValues.P5 = P5;
  NextionValues.P6 = P6;
  NextionValues.a1 = a1;                    // Sequencer key Acceloration values
  NextionValues.a2 = a2;
  NextionValues.a3 = a3;
  NextionValues.a4 = a4;
  NextionValues.a5 = a5;
  NextionValues.a6 = a6;
  NextionValues.SMC = SMC;                  //Stop motion counter

  //If a request has been made from the controller for a network adress update
  NextionValues.IP1 = IP1;
  NextionValues.IP2 = IP2;
  NextionValues.IP3 = IP3;
  NextionValues.IP4 = IP4;
  NextionValues.IPGW = IPGW;
  NextionValues.UDP = UDP;







  //NextionValues.mess = mess;                  //message window
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(broadcastAddress, (uint8_t *) &NextionValues, sizeof(NextionValues));
}


//*********************************************ARM The Head READY TO START**************************************************
void Start_1() {

  //  Arm the slider to start point ready for second play press
  //Serial.print("Entering Start1 ");
  digitalWrite(StepFOC, LOW);                                                                              //Engage the focus motor
  digitalWrite(StepD, LOW);
  delay(100);

  stepper1->setSpeedInHz(2000);
  stepper1->setAcceleration(1500);
  stepper2->setSpeedInUs(2500);
  stepper2->setAcceleration(1000);
  stepper3->setSpeedInUs(50);
  stepper3->setAcceleration(2000);
  stepper4->setSpeedInUs(50);
  stepper4->setAcceleration(2000);
  if (SEQ == 0) {
    Serial.print("Stepper1 In before move =: ");
    Serial.println(TLTin_position);


    stepper1->moveTo(TLTin_position);
    delay(10);
    stepper2->moveTo(PANin_position);
    delay(10);
    stepper3->moveTo(FOCin_position);
    delay(10);
    stepper4->moveTo(ZMin_position);
  }
  if (SEQ == 1) {
    stepper1->moveTo(Tlt_k1_position);
    delay(10);
    stepper2->moveTo(Pan_k1_position);
    delay(10);
    stepper3->moveTo(Foc_k1_position);
    //delay(10);
    //stepper4->moveTo(Zoom_k1_position);
  }
  while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() || stepper4->isRunning() ) {                             //delay until move complete (block)
    delay(10);
  }

  Serial.print("Stepper1 current position: ");
  Serial.println(stepper1->getCurrentPosition());
  But_Com = 5;                                                                                                   //Stop the nextion timer

  if (PTZ_Cam == 0) {
    SendNextionValues();
    delay(100);
  }
  But_Com = 0;
}

//************************************************RUN THE SEQUENCE BASIC**************************************************************
void Start_2() {

  digitalWrite(StepFOC, LOW);                                       //Engage the steppers
  digitalWrite(StepD, LOW);
  if (Bounce >= 1) {
    delay(1300);

  }

  But_Com = 4;                                                    //Start the Nextion timer
  PT = 2;                                                         //Tell other devices to start move
  SendNextionValues();
  delay(10);
  But_Com = 0;
  PT = 1;

  stepper1->setSpeedInHz(TLTstep_speed);                           //Setup speed and acceloration values
  stepper1->setAcceleration(TLTease_Value);
  stepper2->setSpeedInHz(PANstep_speed);
  stepper2->setAcceleration(PANease_Value);
  stepper3->setSpeedInHz(FOCstep_speed);
  stepper3->setAcceleration(FOCease_Value);
  stepper4->setSpeedInHz(ZOOMstep_speed);
  stepper4->setAcceleration(ZOOMease_Value);

  if (TLTtravel_dist != 0) {
    stepper1->moveTo(TLTout_position);                               //Move the steppers
    delay(10);
  }
  if (PANtravel_dist != 0) {
    stepper2->moveTo(PANout_position);
    delay(10);
  }
  if (FOCtravel_dist != 0) {
    stepper3->moveTo(FOCout_position);
    delay(10);
  }
  if (ZOOMtravel_dist != 0) {
    stepper4->moveTo(ZMout_position);
  }
  while (stepper2->isRunning() || stepper1->isRunning() || stepper3->isRunning() || stepper4->isRunning()) {          //delay until move complete (block)
    delay(20);
  }
  //******************************************RUN THE SEQUENCE with Bounce************************************************
  //*************************Formally start_3 which ran into watchdog problems looping between two functions********************

  if (Bounce >= 1) {
    delay(1300);

    Bounce = Bounce - 1;
    But_Com = 6;                                                                                //Send current bounce counter back to nextion
    PT = 2;                                                                                     //Tell other devices to start
    SendNextionValues();
    delay(10);
    But_Com = 0;
    PT = 1;

    stepper1->moveTo(TLTin_position);                                                          //Move the steppers
    delay(2);
    stepper2->moveTo(PANin_position);
    delay(2);
    stepper3->moveTo(FOCin_position);
    delay(2);
    stepper4->moveTo(ZMin_position);




    while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() || stepper4->isRunning()  ) {         //delay until move complete (block)
      delay(10);
    }



    if (Bounce > 0) {
      Start_2();                                            //If no other systems pressent the just start bounce return
    } else {
      digitalWrite(CAM, HIGH);                              //Stop camera
      delay (200);
      digitalWrite(CAM, LOW);
      Tally = 1;                                            //Turn off the tally light
      leds[0] = CRGB::Black;
      leds[1] = CRGB::Black;
      FastLED.show();
      But_Com = 5;                                          //Tell the nextion to stop the timer
      PT = 1;
      SendNextionValues();
      delay(10);
      But_Com = 0;
      Start_1();
    }



  } else  {
    digitalWrite(CAM, HIGH);                                 //Stop camera
    delay (200);
    digitalWrite(CAM, LOW);
    Tally = 0;                                            //Turn off the tally light
    leds[0] = CRGB::Black;
    leds[1] = CRGB::Black;
    FastLED.show();
    But_Com = 5;                                             //Tell the nextion to stop the timer
    PT = 1;
    SendNextionValues();
    delay(10);
    But_Com = 0;
    Start_1();
  }

}



//**********************************************Timelapse*******************************************
void Start_4() {
  digitalWrite(StepFOC, LOW);
  TLTtravel_dist = (TLTout_position - TLTin_position);                     // Distance we have to go
  PANtravel_dist = (PANout_position - PANin_position);
  FOCtravel_dist = (FOCout_position - FOCin_position);
  ZOOMtravel_dist = (ZMout_position - ZMin_position);
  // TpsD = (crono_seconds * 1000);                                        //use the timer seconds as delay time for timelapse

  TLTTlps_step_dist = TLTtravel_dist / Tps;
  PANTlps_step_dist = PANtravel_dist / Tps;
  FOCTlps_step_dist = FOCtravel_dist / Tps;
  ZOOMTlps_step_dist = ZOOMtravel_dist / Tps;

  TLTstep_speed = (TLTtravel_dist / 5);
  PANstep_speed = (PANtravel_dist / 5);
  FOCstep_speed = (FOCtravel_dist / 5);
  ZOOMstep_speed = (ZOOMtravel_dist / 5);



  while (Tps >= 1) {


    //**********If Jib  is conected give slider or jib  control over timing**********
    if (JB > 0 ) {
      JB = 1;
      stepper1->setSpeedInHz(300);
      stepper1->setAcceleration(500);
      stepper2->setSpeedInHz(300);
      stepper2->setAcceleration(500);
      stepper3->setSpeedInHz(600);
      stepper3->setAcceleration(800);
      stepper4->setSpeedInHz(600);
      stepper4->setAcceleration(800);

      stepper1->moveTo(stepper1->getCurrentPosition() + TLTTlps_step_dist);    //Current position plus one fps move
      delay(5);
      stepper2->moveTo(stepper2->getCurrentPosition() + PANTlps_step_dist);    //Current position plus one fps move
      delay(5);
      stepper3->moveTo(stepper3->getCurrentPosition() + FOCTlps_step_dist);    //Current position plus one fps move
      delay(5);
      stepper4->moveTo(stepper4->getCurrentPosition() + FOCTlps_step_dist);    //Current position plus one fps move

      while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() || stepper4->isRunning() ) {                      //delay until move complete
        delay(5);
      }

      while (JB != 2) {
        delay(20);
      }

      delay(1000);
      digitalWrite(CAM, HIGH);                                               //Fire shutter
      if (TpsD <= 1000) {
        TpsD = 1000;
      }                                                          //Time the shutter is open for
      delay(TpsD);
      digitalWrite(CAM, LOW);                                                //Cose the shutter and move on

      if (Tps != 0) {
        Tps = Tps - 1;
      }
      if (Tps == 0) {
        JB = 1;
      }
    }

    //**********If Slider is conected give slider control over timing**********
    if (Sld > 0 ) {                                                             //If Slider or Jib  is conected give slider or jib  control over timing
      Sld = 1;
      stepper1->setSpeedInHz(300);
      stepper1->setAcceleration(500);
      stepper2->setSpeedInHz(300);
      stepper2->setAcceleration(500);
      stepper3->setSpeedInHz(600);
      stepper3->setAcceleration(1000);
      stepper4->setSpeedInHz(600);
      stepper4->setAcceleration(1000);

      stepper1->moveTo(stepper1->getCurrentPosition() + TLTTlps_step_dist);    //Current position plus one fps move
      delay(5);
      stepper2->moveTo(stepper2->getCurrentPosition() + PANTlps_step_dist);    //Current position plus one fps move
      delay(5);
      stepper3->moveTo(stepper3->getCurrentPosition() + FOCTlps_step_dist);    //Current position plus one fps move
      delay(5);
      stepper4->moveTo(stepper4->getCurrentPosition() + ZOOMTlps_step_dist);    //Current position plus one fps move
      while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() || stepper3->isRunning()) {                       //delay until move complete
        delay(5);
      }
      while (Sld != 2) {
        delay(20);
      }
      Sld = 1;

      delay(1000);
      digitalWrite(CAM, HIGH);                                                 //Fire shutter
      if (TpsD <= 1000) {
        TpsD = 1000;
      }                                                                       //Time the shutter is open for
      delay(TpsD);
      digitalWrite(CAM, LOW);                                                 //Cose the shutter and move on

      if (Tps != 0) {
        Tps = Tps - 1;

      }
      if (Tps == 0) {
        Sld = 1;
      }

    }

    //**********If Slider or Jib is not with us take control**********
    if (Sld == 0 && JB == 0) {
      //Serial.println("Tlps Trigger from PT");
      //Trigger pantilt head timelapse move
      SendNextionValues();                                                      //Trigger pantilt head timelapse move
      stepper1->setSpeedInHz(300);
      stepper1->setAcceleration(500);
      stepper2->setSpeedInHz(300);
      stepper2->setAcceleration(500);
      stepper3->setSpeedInUs(600);
      stepper3->setAcceleration(800);
      stepper4->setSpeedInUs(600);
      stepper4->setAcceleration(800);

      stepper1->moveTo(stepper1->getCurrentPosition() + TLTTlps_step_dist);     //Current position plus one fps move
      delay(10);
      stepper2->moveTo(stepper2->getCurrentPosition() + PANTlps_step_dist);     //Current position plus one fps move
      delay(10);
      stepper3->moveTo(stepper3->getCurrentPosition() + FOCTlps_step_dist);     //Current position plus one fps move
      delay(10);
      stepper4->moveTo(stepper4->getCurrentPosition() + ZOOMTlps_step_dist);     //Current position plus one fps move

      if (Tps != 0) {
        Tps = Tps - 1;
      }
      But_Com = 7;                                                              //Tell the nextion to update the frame counter
      SendNextionValues();
      But_Com = 0;
      while (stepper1->isRunning()) {                                           //delay until move complete (blocking)
        delay(10);
      }
      PT = 2;                                                                   //Tell any other parts of the system the PT is ready to take the shot
      SendNextionValues();
      delay(1000);                                                   //Take a second to ensure camera is still

      digitalWrite(CAM, HIGH);                                                  //Fire shutter
      if (TpsD <= 1000) {
        TpsD = 1000;
      }
      delay(TpsD);                                                            //Time the shutter is open for
      digitalWrite(CAM, LOW);                                                   //Cose the shutter and move on
      delay (500);
      PT = 1;
      SendNextionValues();
    }
  }



  But_Com = 5;                                                                  //Tell the nextion to stop the timer
  SendNextionValues();
  But_Com = 0;

  Start_1();                                                                    //send the camera back to the start
}



//**********************************************Stop Motion*******************************************
void Start_7() {
  if (Sld == 0 && JB == 0) {                                                     //If no slider or jib take charge of the count
    if (stopM_active = 1) {                                                     // Is stopM active?
      SMC = SMC - 1;                                                             //Subtract frame
    } else {
      SMC = Tps;
      SMC = SMC - 1;
    }
    But_Com = 8;
    SendNextionValues();                                                    //Tell the nextion to update the stopmotion frame counter
  } else {
    SMC = SMC - 1;
  }
  TLTtravel_dist = (TLTout_position - TLTin_position);                           // Distance we have to go
  PANtravel_dist = (PANout_position - PANin_position);
  FOCtravel_dist = (FOCout_position - FOCin_position);
  ZOOMtravel_dist = (ZMout_position - ZMin_position);                      //Ready for 4th axis

  TLTTlps_step_dist = (TLTtravel_dist / stopM_frames);
  PANTlps_step_dist = (PANtravel_dist / stopM_frames);
  FOCTlps_step_dist = (FOCtravel_dist / stopM_frames);
  ZOOMTlps_step_dist = (FOCtravel_dist / stopM_frames);                        //Ready for 4th axis


  digitalWrite(StepD, LOW);


  TLTstep_speed = (TLTtravel_dist / 5);
  PANstep_speed = (PANtravel_dist / 5);
  FOCstep_speed = (FOCtravel_dist / 5);
  ZOOMstep_speed = (ZOOMtravel_dist / 5);

  stepper1->setSpeedInUs(500);
  stepper1->setAcceleration(500);

  stepper2->setSpeedInUs(500);
  stepper2->setAcceleration(500);

  stepper3->setSpeedInUs(500);
  stepper3->setAcceleration(500);

  stepper4->setSpeedInUs(500);
  stepper4->setAcceleration(500);

  if (SMC >= 0  && stopM_cancel != 1) {                                           //Run the steppers

    stepper1->moveTo(stepper1->getCurrentPosition() + TLTTlps_step_dist);           //Current position plus one fps move
    delay(10);
    stepper2->moveTo(stepper2->getCurrentPosition() + PANTlps_step_dist);
    delay(10);
    stepper3->moveTo(stepper3->getCurrentPosition() + FOCTlps_step_dist);
    delay(10);
    stepper4-> moveTo(stepper4->getCurrentPosition() + ZOOMTlps_step_dist);          //Ready for forth axis

    //delay until move complete. **Add stepper 4 if present
    while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() || stepper4->isRunning()) {
      delay(10);
    }
    delay (1000);                                                                     //Take a second to ensure camera is still
    digitalWrite(CAM, HIGH);                                                          //Fire shutter
    if (TpsD <= 1000) {
      TpsD = 1000;
    }                                                                                 //Time the shutter is open for if on ball. If not on ball camera has control of shutter time and user must set this higher than shutter speed
    delay(TpsD);
    digitalWrite(CAM, LOW);                                                           //Cose the shutter and move on

    if (Sld == 0 && JB == 0) {
      But_Com = 5;
      SendNextionValues();
      But_Com = 0;
    }
  }
  stopM_play = 0;

  if (SMC == 0) {                                                                  // If the Stopmotion has finished or received a cancel. send the camera back to the start                                                                      // No longer need the total number of frames so 0
    SMC = Tps;
    stopM_active = 0;                                                              // StopM finished inactive
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
  Pan_k1_position = Pan_k1_positionH;
  Tlt_k1_position = Tlt_k1_positionH;
  Foc_k1_position = Foc_k1_positionH;

  Pan_k2_position = Pan_k2_positionH;
  Tlt_k2_position = Tlt_k2_positionH;
  Foc_k2_position = Foc_k2_positionH;

  Pan_k3_position = Pan_k3_positionH;
  Tlt_k3_position = Tlt_k3_positionH;
  Foc_k3_position = Foc_k3_positionH;

  Pan_k4_position = Pan_k4_positionH;
  Tlt_k4_position = Tlt_k4_positionH;
  Foc_k4_position = Foc_k4_positionH;

  Pan_k5_position = Pan_k5_positionH;
  Tlt_k5_position = Tlt_k5_positionH;
  Foc_k5_position = Foc_k5_positionH;

  Pan_k6_position = Pan_k6_positionH;
  Tlt_k6_position = Tlt_k6_positionH;
  Foc_k6_position = Foc_k6_positionH;

  s1 = s1H;
  s2 = s2H;
  s3 = s3H;
  s4 = s4H;
  s5 = s5H;
  s6 = s6H;

  k1A = k1AH;
  k2A = k2AH;
  k3A = k3AH;
  k4A = k4AH;
  k5A = k5AH;
  k6A = k6AH;
}
void Set_reverse_values() {
  switch (LastMove) {
    case 3:
      Pan_k1_position = Pan_k3_positionH;
      Tlt_k1_position = Tlt_k3_positionH;
      Foc_k1_position = Foc_k3_positionH;

      Pan_k2_position = Pan_k2_positionH;
      Tlt_k2_position = Tlt_k2_positionH;
      Foc_k2_position = Foc_k2_positionH;

      Pan_k3_position = Pan_k1_positionH;
      Tlt_k3_position = Tlt_k1_positionH;
      Foc_k3_position = Foc_k1_positionH;

      s1 = s3H;
      s2 = s2H;
      s3 = s1H;

      k1A = k3AH;
      k2A = k2AH;
      k3A = k1AH;
      break;

    case 4:
      Pan_k1_position = Pan_k4_positionH;
      Tlt_k1_position = Tlt_k4_positionH;
      Foc_k1_position = Foc_k4_positionH;

      Pan_k2_position = Pan_k3_positionH;
      Tlt_k2_position = Tlt_k3_positionH;
      Foc_k2_position = Foc_k3_positionH;

      Pan_k3_position = Pan_k2_positionH;
      Tlt_k3_position = Tlt_k2_positionH;
      Foc_k3_position = Foc_k2_positionH;

      Pan_k4_position = Pan_k1_positionH;
      Tlt_k4_position = Tlt_k1_positionH;
      Foc_k4_position = Foc_k1_positionH;

      s1 = s4H;
      s2 = s3H;
      s3 = s2H;
      s4 = s1H;

      k1A = k4AH;
      k2A = k3AH;
      k3A = k2AH;
      k4A = k1AH;
      break;

    case 5:
      Pan_k1_position = Pan_k5_positionH;
      Tlt_k1_position = Tlt_k5_positionH;
      Foc_k1_position = Foc_k5_positionH;

      Pan_k2_position = Pan_k4_positionH;
      Tlt_k2_position = Tlt_k4_positionH;
      Foc_k2_position = Foc_k4_positionH;

      Pan_k3_position = Pan_k3_positionH;
      Tlt_k3_position = Tlt_k3_positionH;
      Foc_k3_position = Foc_k3_positionH;

      Pan_k4_position = Pan_k2_positionH;
      Tlt_k4_position = Tlt_k2_positionH;
      Foc_k4_position = Foc_k2_positionH;

      Pan_k5_position = Pan_k1_positionH;
      Tlt_k5_position = Tlt_k1_positionH;
      Foc_k5_position = Foc_k1_positionH;

      s1 = s5H;
      s2 = s4H;
      s3 = s3H;
      s4 = s2H;
      s5 = s1H;

      k1A = k5AH;
      k2A = k4AH;
      k3A = k3AH;
      k4A = k2AH;
      k5A = k1AH;
      break;
    case 6:
      Pan_k1_position = Pan_k6_positionH;
      Tlt_k1_position = Tlt_k6_positionH;
      Foc_k1_position = Foc_k6_positionH;

      Pan_k2_position = Pan_k5_positionH;
      Tlt_k2_position = Tlt_k5_positionH;
      Foc_k2_position = Foc_k5_positionH;

      Pan_k3_position = Pan_k4_positionH;
      Tlt_k3_position = Tlt_k4_positionH;
      Foc_k3_position = Foc_k4_positionH;

      Pan_k4_position = Pan_k3_positionH;
      Tlt_k4_position = Tlt_k3_positionH;
      Foc_k4_position = Foc_k3_positionH;

      Pan_k5_position = Pan_k2_positionH;
      Tlt_k5_position = Tlt_k2_positionH;
      Foc_k5_position = Foc_k2_positionH;

      Pan_k6_position = Pan_k1_positionH;
      Tlt_k6_position = Tlt_k1_positionH;
      Foc_k6_position = Foc_k1_positionH;

      s1 = s6H;
      s2 = s5H;
      s3 = s4H;
      s4 = s3H;
      s5 = s2H;
      s6 = s1H;

      k1A = k6AH;
      k2A = k5AH;
      k3A = k4AH;
      k4A = k3AH;
      k5A = k2AH;
      k6A = k1AH;
      break;
  }
}
void StorePositions() {
  //Save original forward values in holding variables ready for Bounce
  Pan_k1_positionH = Pan_k1_position;
  Tlt_k1_positionH = Tlt_k1_position;
  Foc_k1_positionH = Foc_k1_position;

  Pan_k2_positionH = Pan_k2_position;
  Tlt_k2_positionH = Tlt_k2_position;
  Foc_k2_positionH = Foc_k2_position;

  Pan_k3_positionH = Pan_k3_position;
  Tlt_k3_positionH = Tlt_k3_position;
  Foc_k3_positionH = Foc_k3_position;

  Pan_k4_positionH = Pan_k4_position;
  Tlt_k4_positionH = Tlt_k4_position;
  Foc_k4_positionH = Foc_k4_position;

  Pan_k5_positionH = Pan_k5_position;
  Tlt_k5_positionH = Tlt_k5_position;
  Foc_k5_positionH = Foc_k5_position;

  Pan_k6_positionH = Pan_k6_position;
  Tlt_k6_positionH = Tlt_k6_position;
  Foc_k6_positionH = Foc_k6_position;

  s1H = s1;
  s2H = s2;
  s3H = s3;
  s4H = s4;
  s5H = s5;
  s6H = s6;

  k1AH = k1A;
  k2AH = k2A;
  k3AH = k3A;
  k4AH = k4A;
  k5AH = k5A;
  k6AH = k6A;
}


void Start_5() {
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
  if (Sld < 1) {
    SendNextionValues();
  }
  But_Com = 0;
  delay(20);
  int StartTime;

  Last_Tlt_accel = 0;
  Last_Tlt_speed = 0;
  Last_Pan_accel = 0;
  Last_Pan_speed = 0;
  Last_Foc_accel = 0;
  Last_Foc_speed = 0;

  Base_Tlt_accel = 0;
  Base_Pan_accel = 0;
  Base_Foc_accel = 0;

  Tlt_move1_Dest = 0;
  Tlt_move2_Dest = 0;
  Tlt_move3_Dest = 0;
  Tlt_move4_Dest = 0;
  Tlt_move5_Dest = 0;

  Pan_move1_Dest = 0;
  Pan_move2_Dest = 0;
  Pan_move3_Dest = 0;
  Pan_move4_Dest = 0;
  Pan_move5_Dest = 0;

  Foc_move1_Dest = 0;
  Foc_move2_Dest = 0;
  Foc_move3_Dest = 0;
  Foc_move4_Dest = 0;
  Foc_move5_Dest = 0;

  s1_speed = 0;
  s2_speed = 0;
  s3_speed = 0;
  s4_speed = 0;
  s5_speed = 0;
  s6_speed = 0;

  int Foc_AccelLock;
  int Tlt_AccelLock;
  int Pan_AccelLock;

  Stp_1_active = 0;
  Stp_2_active = 0;
  Stp_3_active = 0;
  Stp_active = 0;
  float Tlt_Ramp_Time = 0;
  float Pan_Ramp_Time = 0;
  float Foc_Ramp_Time = 0;

  stepper1_plan();
  delay(10);
  stepper2_plan();
  delay(10);
  stepper3_plan();
  delay(10);
  //  Serial.print("Tlt_move1_Dest:   ");
  //  Serial.print(Tlt_move1_Dest);
  //  Serial.print("Pan_move1_Dest:   ");
  //  Serial.print(Pan_move1_Dest);
  //  Serial.print("Foc_move1_Dest:   ");
  //  Serial.print(Foc_move1_Dest);
  //  Serial.println("**********************");
  //  Serial.print("Tlt_move2_Dest:   ");
  //  Serial.print(Tlt_move2_Dest);
  //  Serial.print("Pan_move2_Dest:   ");
  //  Serial.print(Pan_move2_Dest);
  //  Serial.print("Foc_move2_Dest:   ");
  //  Serial.print(Foc_move2_Dest);
  //  Serial.println("**********************");
  //  Serial.print("Tlt_move3_Dest:   ");
  //  Serial.print(Tlt_move3_Dest);
  //  mess = Tlt_move3_Dest;
  //  SendNextionValues();
  //  Serial.print("Pan_move3_Dest:   ");
  //  Serial.print(Pan_move3_Dest);
  //  Serial.print("Foc_move3_Dest:   ");
  //  Serial.print(Foc_move3_Dest);
  //****************************************************move1-2*****************************************

  //**********************Pan move****************************
  if (s2 != 0) {
    KeyB = Pan_k2_position;
    KeyA = Pan_k1_position;
    Find_KeyDist();
    if (KeyDist != 0 && s2 != 0) {
      Key_Crono_time = int(float(100 / s1) * Seq_Time);                                         //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s1_speed = abs(factor * 1000000);
      Stp_2_active = 1;
      Stp_active = 1;                                                                             //For end of move check

      switch (a1) {
        case 0:                                                                                   //No ease
          k1A = (KeyDist / (Key_Crono_time));
          k1A = k1A * 3;
          break;
        case 3:

          k1A = (KeyDist / (Key_Crono_time));
          break;

        case 2:
          k1A = (KeyDist / (Key_Crono_time));
          k1A = k1A * 1;
          break;

        case 1:
          k1A = (KeyDist / (Key_Crono_time));
          k1A = k1A * 3;
          break;
      }

      stepper2->setAcceleration(k1A);
      stepper2->setSpeedInUs(s1_speed);


      switch (Pan_move1_Dest) {                                                                     //Pick up how far the motor should move before it either stopes or changes direction
        case 2:
          stepper2->moveTo(Pan_k2_position);
          break;
        case 3:
          s1_speed = s1_speed + ((s1_speed / 100) * 8);
          stepper2->setSpeedInUs(s1_speed);
          stepper2->moveTo(Pan_k3_position);
          break;
        case 4:
          s1_speed = s1_speed + ((s1_speed / 100) * 8);
          stepper2->setSpeedInUs(s1_speed);
          stepper2->moveTo(Pan_k4_position);
          break;
        case 5:
          s1_speed = s1_speed + ((s1_speed / 100) * 8);
          stepper2->setSpeedInUs(s1_speed);
          stepper2->moveTo(Pan_k5_position);
          break;
        case 6:
          s1_speed = s1_speed + ((s1_speed / 100) * 8);
          stepper2->setSpeedInUs(s1_speed);
          stepper2->moveTo(Pan_k6_position);
          break;
      }
      Base_Pan_accel =  k1A;
      Last_Pan_accel = k1A;
      Last_Pan_speed = s1_speed;
      delay(10);
      ActiveMove = 1;
    } else {
      Stp_2_active = 0;
    }

    //************************Tlt move*****************************************************
    KeyB = Tlt_k2_position;
    KeyA = Tlt_k1_position;
    Find_KeyDist();
    if (KeyDist != 0 && s2 != 0) {
      Key_Crono_time = int(float(100 / s1) * Seq_Time);                                          //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s1_speed = abs(factor * 1000000);
      Stp_1_active = 1;
      Stp_active = 1;

      switch (a1) {
        case 0:                                                                                   //No ease
          k1A = (KeyDist / (Key_Crono_time));
          k1A = k1A * 3;
          break;
        case 3:
          k1A = (KeyDist / (Key_Crono_time ));
          break;

        case 2:
          k1A = (KeyDist / (Key_Crono_time ));
          k1A = k1A * 2;
          break;

        case 1:
          k1A = (KeyDist / (Key_Crono_time ));
          k1A = k1A * 3;
          break;
      }

      stepper1->setAcceleration(k1A);
      stepper1->setSpeedInUs(s1_speed);


      switch (Tlt_move1_Dest) {                                                      //Pick up how far the motor should move before it either stopes or changes direction
        case 2:
          stepper1->moveTo(Tlt_k2_position);
          break;
        case 3:
          s1_speed = s1_speed + ((s1_speed / 100) * 8);
          stepper1->setSpeedInUs(s1_speed);
          stepper1->moveTo(Tlt_k3_position);
          break;
        case 4:
          s1_speed = s1_speed + ((s1_speed / 100) * 8);
          stepper1->setSpeedInUs(s1_speed);
          stepper1->moveTo(Tlt_k4_position);
          break;
        case 5:
          s1_speed = s1_speed + ((s1_speed / 100) * 8);
          stepper1->setSpeedInUs(s1_speed);
          stepper1->moveTo(Tlt_k5_position);
          break;
        case 6:
          s1_speed = s1_speed + ((s1_speed / 100) * 8);
          stepper1->setSpeedInUs(s1_speed);
          stepper1->moveTo(Tlt_k6_position);
          break;
      }
      Base_Tlt_accel =  k1A;
      Last_Tlt_accel = k1A;
      Last_Tlt_speed = s1_speed;

      delay(10);
      ActiveMove = 1;
    } else {
      Stp_1_active = 0;
    }

    //************************Foc move*****************************************************
    KeyB = Foc_k2_position;
    KeyA = Foc_k1_position;
    Find_KeyDist();
    if (KeyDist != 0 && s2 != 0) {
      Key_Crono_time = int(float(100 / s1) * Seq_Time);                                         //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s1_speed = abs(factor * 1000000);
      Stp_3_active = 1;
      Stp_active = 1;

      switch (a1) {
        case 0:                                                                                   //No ease
          k1A = (KeyDist / (Key_Crono_time ));
          k1A = k1A * 3;
          break;
        case 3:
          k1A = (KeyDist / (Key_Crono_time ));
          break;

        case 2:
          k1A = (KeyDist / (Key_Crono_time));
          k1A = k1A * 2;
          break;

        case 1:
          k1A = (KeyDist / (Key_Crono_time));
          k1A = k1A * 3;
          break;
      }

      stepper3->setAcceleration(k1A);
      stepper3->setSpeedInUs(s1_speed);
      Base_Foc_accel =  k1A;
      Last_Foc_accel = k1A;
      Last_Foc_speed = s1_speed;

      switch (Foc_move1_Dest) {                                                            //Pick up how far the motor should move before it either stopes or changes direction
        case 2:
          stepper3->moveTo(Foc_k2_position);
          break;
        case 3:
          s1_speed = s1_speed + ((s1_speed / 100) * 8);
          stepper3->setSpeedInUs(s1_speed);
          stepper3->moveTo(Foc_k3_position);
          break;
        case 4:
          s1_speed = s1_speed + ((s1_speed / 100) * 8);
          stepper3->setSpeedInUs(s1_speed);
          stepper3->moveTo(Foc_k4_position);
          break;
        case 5:
          s1_speed = s1_speed + ((s1_speed / 100) * 8);
          stepper3->setSpeedInUs(s1_speed);
          stepper3->moveTo(Foc_k5_position);
          break;
        case 6:
          s1_speed = s1_speed + ((s1_speed / 100) * 8);
          stepper3->setSpeedInUs(s1_speed);
          stepper3->moveTo(Foc_k6_position);
          break;
      }

      delay(10);
      ActiveMove = 1;
    } else {
      Stp_3_active = 0;
    }
    if ((Stp_1_active != 0) || (Stp_2_active = !0) || (Stp_3_active != 0)) {              //If any of the steppers are moving tell the slider
      if (Bounce != 0) {
        delay(1000);
      }
      PT = 2;
      SendNextionValues();
    }
    //************************Slider move*****************************************************
    if (Sld == 2 ) {
      ActiveMove = 1;
      Stp_active = 1;
    }
    //  Time2Ramp2 = (Key_Crono_time / 2) * 1000;
    //  delay(Time2Ramp2);
    //**************************************move 1 mid way check***********************************


    //***************************Test to chech for end of move**********************************************
    while (Stp_active == 1) {
      if (stepper3->isRunning()) {

        if (Foc_k2_position > Foc_k1_position && stepper3->isRunning()) {                   //get direction
          if (stepper3->getCurrentPosition() >= (Foc_k2_position)) {                        //if + direction
            Stp_3_active = 0;
          }
        } else {
          if (stepper3->getCurrentPosition() <= (Foc_k2_position)) {                        //if - direction
            Stp_3_active = 0;
          }
        }
      } else {
        Stp_3_active = 0;
      }

      if (stepper1->isRunning()) {

        if (Tlt_k2_position > Tlt_k1_position && stepper1->isRunning()) {                   //get direction
          if (stepper1->getCurrentPosition() >= (Tlt_k2_position)) {                        //if + direction
            Stp_1_active = 0;
          }
        } else {
          if (stepper1->getCurrentPosition() <= (Tlt_k2_position)) {                        //if - direction
            Stp_1_active = 0;
          }
        }
      } else {
        Stp_1_active = 0;

      }

      if (stepper2->isRunning()) {
        if (Pan_k2_position > Pan_k1_position && stepper2->isRunning()) {                   //get direction
          if (stepper2->getCurrentPosition() >= (Pan_k2_position)) {                        //if + direction
            Stp_2_active = 0;
          }
        } else {
          if (stepper2->getCurrentPosition() <= (Pan_k2_position)) {                        //if - direction
            Stp_2_active = 0;
          }
        }
      } else {
        Stp_2_active = 0;
      }

      if (Stp_1_active == 0 && Stp_2_active == 0 && Stp_3_active == 0 ) {                              //Test to see if The PT steppers have finished move
        PT = 1;                                                                                         //Tell the Slider the PT has finished its move
        SendNextionValues();
        delay(100);
      }
      if (Stp_1_active == 0 && Stp_2_active == 0 && Stp_3_active == 0 && Sld != 2 ) {                    //Final test to see if all steppers have finished move
        PT = 1;                                                                                         //Tell the Slider the PT has finished its move
        SendNextionValues();
        Stp_active = 0;
      }
    }
  }
  //**********************************************************************************************************
  //****************************************************move2-3***********************************************
  //**********************************************************************************************************


  //**********************Pan move****************************
  if (s3 != 0) {
    KeyB = Pan_k3_position;
    KeyA = Pan_k2_position;
    Find_KeyDist();
    if (KeyDist != 0 && s3 != 0) {
      Key_Crono_time = int(float(100 / s2) * Seq_Time);                                         //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s2_speed = abs(factor * 1000000);
      Stp_2_active = 1;
      Stp_active = 1;

      switch (a2) {
        case 0:                                                                                   //No ease
          k2A = (KeyDist / (Key_Crono_time ));
          k2A = k2A * 3;
          break;
        case 3:
          k2A = (KeyDist / (Key_Crono_time ));
          break;

        case 2:
          k2A = (KeyDist / (Key_Crono_time));
          k2A = k2A * 2;
          break;

        case 1:
          k2A = (KeyDist / (Key_Crono_time ));
          k2A = k2A * 3;
          break;
      }
      if (stepper2->isRunning() && Pan_move1_Dest > 2 && Pan_move2_Rst != 1 ) {                                   //If the stepper is running on from last move
        stepper2->setAcceleration(k2A);
        s2_speed = s2_speed + ((s2_speed / 100) * 8);
        stepper2->setSpeedInUs(s2_speed);
        stepper2-> applySpeedAcceleration();
        delay(10);
      } else {

        stepper2->setAcceleration(k2A);                                                              //If starting fresh Max accel to catch up on turnarround
        stepper2->setSpeedInUs(s2_speed);


        switch (Pan_move2_Dest) {                                  //Pick up how far the motor should move before it either stopes or changes direction
          case 3:
            stepper2->moveTo(Pan_k3_position);
            break;
          case 4:
            s2_speed = s2_speed + ((s2_speed / 100) * 8);
            stepper2->setSpeedInUs(s2_speed);
            stepper2->moveTo(Pan_k4_position);
            break;
          case 5:
            s1_speed = s1_speed + ((s1_speed / 100) * 8);
            stepper2->setSpeedInUs(s2_speed);
            stepper2->moveTo(Pan_k5_position);
            break;
          case 6:
            s1_speed = s1_speed + ((s1_speed / 100) * 8);
            stepper2->setSpeedInUs(s2_speed);
            stepper2->moveTo(Pan_k6_position);
            break;
        }
        delay(10);
      }
      Base_Pan_accel = k2A;
      Last_Pan_accel = k2A;
      Last_Pan_speed = s2_speed;

      ActiveMove = 1;
    } else {
      Stp_2_active = 0;
    }

    //************************Tlt move*****************************************************
    KeyB = Tlt_k3_position;
    KeyA = Tlt_k2_position;
    Find_KeyDist();
    if (KeyDist != 0 && s3 != 0) {

      Key_Crono_time = int(float(100 / s2) * Seq_Time);                                         //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s2_speed = abs(factor * 1000000);
      Stp_1_active = 1;
      Stp_active = 1;

      switch (a2) {
        case 0:                                                                                   //No ease
          k2A = abs(KeyDist / (Key_Crono_time ));
          k2A = k2A * 3;
          break;
        case 3:
          k2A = (KeyDist / (Key_Crono_time ));
          break;

        case 2:
          k2A = (KeyDist / (Key_Crono_time ));
          k2A = k2A * 2;
          break;

        case 1:
          k2A = (KeyDist / (Key_Crono_time ));
          k2A = k2A * 3;
          break;
      }

      if (stepper1->isRunning() == true && Tlt_move1_Dest > 2 && Tlt_move2_Rst != 1 ) {                                   //If the stepper is running on from last move
        stepper1->setAcceleration(k2A);
        stepper1->setSpeedInUs(s2_speed);
        stepper1-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper1->setAcceleration(k2A);                                                              //If starting fresh Max accel to catch up on turnarround
        stepper1->setSpeedInUs(s2_speed);

        switch (Tlt_move2_Dest) {                                  //Pick up how far the motor should move before it either stopes or changes direction
          case 3:
            stepper1->moveTo(Tlt_k3_position);
            break;
          case 4:
            s2_speed = s2_speed + ((s2_speed / 100) * 8);
            stepper1->setSpeedInUs(s2_speed);
            stepper1->moveTo(Tlt_k4_position);
            break;
          case 5:
            s2_speed = s2_speed + ((s2_speed / 100) * 8);
            stepper1->setSpeedInUs(s2_speed);
            stepper1->moveTo(Tlt_k5_position);
            break;
          case 6:
            s2_speed = s2_speed + ((s2_speed / 100) * 8);
            stepper1->setSpeedInUs(s2_speed);
            stepper1->moveTo(Tlt_k6_position);
            break;
        }
        delay(10);
      }
      Base_Tlt_accel = k2A;
      Last_Tlt_accel = k2A;
      Last_Tlt_speed = s2_speed;

      ActiveMove = 2;
    } else {
      Stp_1_active = 0;
    }

    //************************Foc move*****************************************************
    KeyB = Foc_k3_position;
    KeyA = Foc_k2_position;
    Find_KeyDist();
    if (KeyDist != 0 && s3 != 0) {
      Key_Crono_time = int(float(100 / s2) * Seq_Time);                                         //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s2_speed = abs(factor * 1000000);
      Stp_3_active = 1;
      Stp_active = 1;

      switch (a2) {
        case 0:                                                                                   //No ease
          k2A = (KeyDist / (Key_Crono_time));
          k2A = k2A * 3;
          break;
        case 3:
          k2A = (KeyDist / (Key_Crono_time));
          break;

        case 2:
          k2A = (KeyDist / (Key_Crono_time));
          k2A = k2A * 2;
          break;

        case 1:
          k2A = (KeyDist / (Key_Crono_time));
          k2A = k2A * 3;
          break;
      }

      if (stepper3->isRunning() && Foc_move1_Dest > 2 && Foc_move2_Rst != 1 ) {                                   //If the stepper is running on from last move
        stepper3->setAcceleration(k2A);
        stepper3->setSpeedInUs(s2_speed);
        stepper3-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper3->setAcceleration( k2A);                                                              //If starting fresh Max accel to catch up on turnarround
        stepper3->setSpeedInUs(s2_speed);

        switch (Foc_move2_Dest) {                                                                    //Pick up how far the motor should move before it either stopes or changes direction
          case 3:
            stepper3->moveTo(Foc_k3_position);
            break;
          case 4:
            s2_speed = s2_speed + ((s2_speed / 100) * 8);
            stepper3->setSpeedInUs(s2_speed);
            stepper3->moveTo(Foc_k4_position);
            break;
          case 5:
            s2_speed = s2_speed + ((s2_speed / 100) * 8);
            stepper3->setSpeedInUs(s2_speed);
            stepper3->moveTo(Foc_k5_position);
            break;
          case 6:
            s2_speed = s2_speed + ((s2_speed / 100) * 8);
            stepper3->setSpeedInUs(s2_speed);
            stepper3->moveTo(Foc_k6_position);
            break;
        }
        delay(10);
      }
      Base_Foc_accel =  k2A;
      Last_Foc_accel =  k2A;
      Last_Foc_speed = s2_speed;

      ActiveMove = 1;
    } else {
      Stp_3_active = 0;
    }
    if ((Stp_1_active != 0) || (Stp_2_active = !0) || (Stp_3_active != 0)) {              //If any of the steppers are moving tell the slider
      delay(1000);
      PT = 2;
      SendNextionValues();
    }
    //************************Slider move*****************************************************
    if (Sld == 2 ) {
      ActiveMove = 2;
      Stp_active = 1;
    }
    //  Time2Ramp2 = (Key_Crono_time / 2) * 1000;
    //  delay(Time2Ramp2);
    //**************************************move 2-3 Second ramp********************************************************************


    //********************************Test to chech for end of move**************************************
    while (Stp_active == 1) {
      if (stepper3->isRunning()) {

        if (Foc_k3_position > Foc_k2_position ) {                                //get direction
          if (stepper3->getCurrentPosition() >= (Foc_k3_position)) {                                     //if + direction
            Stp_3_active = 0;
          }
        } else {
          if (stepper3->getCurrentPosition() <= (Foc_k3_position)) {                                      //if - direction
            Stp_3_active = 0;
          }
        }
      } else {
        Stp_3_active = 0;
      }
      if (stepper1->isRunning() ) {

        if (Tlt_k3_position > Tlt_k2_position ) {                               //get direction
          if (stepper1->getCurrentPosition() >= (Tlt_k3_position)) {                                    //if + direction
            Stp_1_active = 0;
          }
        } else {
          if (stepper1->getCurrentPosition() <= (Tlt_k3_position)) {                                    //if - direction
            Stp_1_active = 0;
          }
        }
      } else {
        Stp_1_active = 0;
      }

      if (stepper2->isRunning()) {
        if (Pan_k3_position > Pan_k2_position ) {                               //get direction
          if (stepper2->getCurrentPosition() >= (Pan_k3_position)) {                                    //if + direction
            Stp_2_active = 0;
          }
        } else {
          if (stepper2->getCurrentPosition() <= (Pan_k3_position)) {                                    //if - direction
            Stp_2_active = 0;
          }
        }
      } else {
        Stp_2_active = 0;
      }

      if (Stp_1_active == 0 && Stp_2_active == 0 && Stp_3_active == 0 ) {                              //Test to see if The PT steppers have finished move
        PT = 1;                                                                                         //Tell the Slider the PT has finished its move
        SendNextionValues();
      }
      if (Stp_1_active == 0 && Stp_2_active == 0 && Stp_3_active == 0 && Sld != 2 ) {                //Final test to see if all steppers have finished move including slider
        PT = 1;                                                                                         //Tell the Slider the PT has finished its move
        SendNextionValues();
        Stp_active = 0;
      }
    }
  }
  //**********************************************************************************************************
  //****************************************************move3-4***********************************************
  //**********************************************************************************************************
  if (s4 != 0) {
    //    Stp_1_active = 1;                                                                     //Used by check when move is complete on all steppers
    //    Stp_2_active = 1;
    //    Stp_3_active = 1;
    //    Stp_active = 1;

    //**********************Pan move****************************
    KeyB = Pan_k4_position;
    KeyA = Pan_k3_position;

    Find_KeyDist();

    if (KeyDist != 0 && s4 != 0) {
      Key_Crono_time = int(float(100 / s3) * Seq_Time);                                     //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s3_speed = abs(factor * 1000000);
      Stp_2_active = 1;
      Stp_active = 1;

      switch (a3) {
        case 0:                                                                                   //No ease
          k3A = (KeyDist / (Key_Crono_time));
          k3A = k3A * 3;
          break;
        case 3:
          k3A = (KeyDist / (Key_Crono_time));
          break;

        case 2:
          k3A = (KeyDist / (Key_Crono_time));
          k3A = k3A * 2;
          break;

        case 1:
          k3A = (KeyDist / (Key_Crono_time));
          k3A = k3A * 3;
          break;
      }
      if (stepper2->isRunning() && Pan_move2_Dest > 3 && Pan_move3_Rst != 1 ) {                                   //If the stepper is running on from last move
        stepper2->setAcceleration(k3A);
        s3_speed = s3_speed + ((s3_speed / 100) * 8);
        stepper2->setSpeedInUs(s3_speed);
        stepper2-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper2->setAcceleration(k3A);                                                      //If starting fresh Max accel to catch up on turnarround
        stepper2->setSpeedInUs(s3_speed);


        switch (Pan_move3_Dest) {                                                            //Pick up how far the motor should move before it either stopes or changes direction
          case 3:
            stepper2->moveTo(Pan_k4_position);
            break;
          case 4:
            stepper2->moveTo(Pan_k4_position);
            break;
          case 5:
            s3_speed = s3_speed + ((s3_speed / 100) * 8);
            stepper2->setSpeedInUs(s3_speed);
            stepper2->moveTo(Pan_k5_position);
            break;
          case 6:
            s3_speed = s3_speed + ((s3_speed / 100) * 8);                                 //5% ajustement on pan if running through keys this slows pan down by 5%
            stepper2->setSpeedInUs(s3_speed);
            stepper2->moveTo(Pan_k6_position);
            break;
        }
        delay(10);
      }
      Base_Pan_accel = k3A;
      Last_Pan_accel = k3A;
      Last_Pan_speed = s3_speed;

      ActiveMove = 3;
    } else {
      Stp_2_active = 0;
    }

    //************************Tlt move*****************************************************
    KeyB = Tlt_k4_position;
    KeyA = Tlt_k3_position;
    Find_KeyDist();
    if (KeyDist != 0 && s4 != 0) {
      Key_Crono_time = int(float(100 / s3) * Seq_Time);                                         //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s3_speed = abs(factor * 1000000);
      Stp_1_active = 1;
      Stp_active = 1;


      switch (a3) {
        case 0:                                                                                   //No ease
          k3A = (KeyDist / (Key_Crono_time));
          k3A = k3A * 3;
          break;
        case 3:
          k3A = (KeyDist / (Key_Crono_time));
          break;

        case 2:
          k3A = (KeyDist / (Key_Crono_time));
          k3A = k3A * 2;
          break;

        case 1:
          k3A = (KeyDist / (Key_Crono_time));
          k3A = k3A * 3;
          break;
      }

      if (stepper1->isRunning() && Tlt_move2_Dest > 3 && Tlt_move3_Rst != 1 ) {                                           //If the stepper is running on from last move
        stepper1->setAcceleration(k3A);
        s3_speed = s3_speed + ((s3_speed / 100) * 8);
        stepper1->setSpeedInUs(s3_speed);
        stepper1-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper1->setAcceleration(k3A);                                                              //If starting fresh Max accel to catch up on turnarround
        stepper1->setSpeedInUs(s3_speed);

        switch (Tlt_move3_Dest) {                                                                    //Pick up how far the motor should move before it either stopes or changes direction

          case 4:
            //s3_speed = s3_speed + ((s3_speed / 100) * 8);
            stepper1->setSpeedInUs(s3_speed);
            stepper1->moveTo(Tlt_k4_position);
            break;
          case 5:
            s3_speed = s3_speed + ((s3_speed / 100) * 8);
            stepper1->setSpeedInUs(s3_speed);
            stepper1->moveTo(Tlt_k5_position);
            break;
          case 6:
            s3_speed = s3_speed + ((s3_speed / 100) * 8);
            stepper1->setSpeedInUs(s3_speed);
            stepper1->moveTo(Tlt_k6_position);
            break;
        }
        delay(10);
      }
      Base_Tlt_accel = k3A;
      Last_Tlt_accel = k3A;
      Last_Tlt_speed = s3_speed;

      ActiveMove = 3;
    } else {
      Stp_1_active = 0;
    }

    //************************Foc move*****************************************************
    KeyB = Foc_k4_position;
    KeyA = Foc_k3_position;
    Find_KeyDist();
    if (KeyDist != 0 && s4 != 0) {
      Key_Crono_time = int(float(100 / s3) * Seq_Time);                                         //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                             //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s3_speed = abs(factor * 1000000);
      Stp_3_active = 1;
      Stp_active = 1;

      switch (a3) {
        case 0:                                                                                   //No ease
          k3A = (KeyDist / (Key_Crono_time));
          k3A = k3A * 3;
          break;

        case 3:
          k3A = (KeyDist / (Key_Crono_time));
          break;

        case 2:
          k3A = (KeyDist / (Key_Crono_time));
          k3A = k3A * 2;
          break;

        case 1:
          k3A = (KeyDist / (Key_Crono_time));
          k3A = k3A * 3;
          break;
      }

      if (stepper3->isRunning() && Foc_move2_Dest > 3 && Foc_move3_Rst != 1 ) {                                  //If the stepper is running on from last move
        stepper3->setAcceleration(k3A);
        s3_speed = s3_speed + ((s3_speed / 100) * 8);
        stepper3->setSpeedInUs(s3_speed);
        stepper3-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper3->setAcceleration(k3A);                                                              //If starting fresh Max accel to catch up on turnarround
        stepper3->setSpeedInUs(s3_speed);

        switch (Foc_move3_Dest) {                                                                    //Pick up how far the motor should move before it either stopes or changes direction
          case 4:
            stepper3->moveTo(Foc_k4_position);
            break;
          case 5:
            s3_speed = s3_speed + ((s3_speed / 100) * 8);
            stepper3->setSpeedInUs(s3_speed);
            stepper3->moveTo(Foc_k5_position);
            break;
          case 6:
            s3_speed = s3_speed + ((s3_speed / 100) * 8);
            stepper3->setSpeedInUs(s3_speed);
            stepper3->moveTo(Foc_k6_position);
            break;
        }
        delay(10);
      }
      Base_Foc_accel = k3A;
      Last_Foc_accel = k3A;
      Last_Foc_speed = s3_speed;

      ActiveMove = 3;
    } else {
      Stp_3_active = 0;
    }
    if ((Stp_1_active != 0) || (Stp_2_active = !0) || (Stp_3_active != 0)) {              //If any of the steppers are moving tell the slider
      delay(1000);
      PT = 2;
      SendNextionValues();
    }
    //************************Slider move*****************************************************
    if (Sld == 2 ) {
      ActiveMove = 1;
      Stp_active = 1;
    }
    //    Time2Ramp2 = (Key_Crono_time / 2) * 1000;
    //    delay(Time2Ramp2);
    //**************************************move 3-4 Second ramp********************************************************************
    //If the running steppers current end point is ahead of this key ajust its speed to match the next keys speed for a smooth entry
    //*******************************************TLT Ramp2**************************************************************************



    //********************************Test to chech for end of move**************************************
    while (Stp_active == 1) {
      if (stepper3->isRunning()) {

        if (Foc_k4_position > Foc_k3_position ) {                                //get direction
          if (stepper3->getCurrentPosition() >= (Foc_k4_position)) {                                     //if + direction
            Stp_3_active = 0;
          }
        } else {
          if (stepper3->getCurrentPosition() <= (Foc_k4_position)) {                                      //if - direction
            Stp_3_active = 0;
          }
        }
      } else {
        Stp_3_active = 0;
      }
      if (stepper1->isRunning() ) {

        if (Tlt_k4_position > Tlt_k3_position ) {                               //get direction
          if (stepper1->getCurrentPosition() >= (Tlt_k4_position)) {                                    //if + direction
            Stp_1_active = 0;
          }
        } else {
          if (stepper1->getCurrentPosition() <= (Tlt_k4_position)) {                                    //if - direction
            Stp_1_active = 0;
          }
        }
      } else {
        Stp_1_active = 0;
      }

      if (stepper2->isRunning()) {
        if (Pan_k4_position > Pan_k3_position ) {                               //get direction
          if (stepper2->getCurrentPosition() >= (Pan_k4_position)) {                                    //if + direction
            Stp_2_active = 0;
          }
        } else {
          if (stepper2->getCurrentPosition() <= (Pan_k4_position)) {                                    //if - direction
            Stp_2_active = 0;
          }
        }
      } else {
        Stp_2_active = 0;
      }

      if (Stp_1_active == 0 && Stp_2_active == 0 && Stp_3_active == 0 ) {                              //Test to see if The PT steppers have finished move
        PT = 1;                                                                                  //Tell the Slider the PT has finished its move
        SendNextionValues();
      }
      if (Stp_1_active == 0 && Stp_2_active == 0 && Stp_3_active == 0 && Sld != 2 ) {        //Final test to see if all steppers have finished move
        PT = 1;                                                                                         //Tell the Slider the PT has finished its move
        SendNextionValues();
        Stp_active = 0;
      }

    }

  }

  //**********************************************************************************************************
  //****************************************************move4-5***********************************************
  //**********************************************************************************************************
  if (s5 != 0) {
    //    Stp_1_active = 1;                                               //Used by check when move is complete on all steppers
    //    Stp_2_active = 1;
    //    Stp_3_active = 1;
    //    Stp_active = 1;

    //**********************Pan move****************************
    KeyB = Pan_k5_position;
    KeyA = Pan_k4_position;
    Find_KeyDist();
    if (KeyDist != 0 && s5 != 0) {
      Key_Crono_time = int(float(100 / s4) * Seq_Time);                                         //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s4_speed = abs(factor * 1000000);
      Stp_2_active = 1;
      Stp_active = 1;

      switch (a4) {
        case 0:                                                                                   //No ease
          k4A = (KeyDist / (Key_Crono_time));
          k4A = k4A * 3;
          break;
        case 3:
          k4A = (KeyDist / (Key_Crono_time));
          break;

        case 2:
          k4A = (KeyDist / (Key_Crono_time));
          k4A = k4A * 2;
          break;

        case 1:
          k4A = (KeyDist / (Key_Crono_time));
          k4A = k4A * 3;
          break;
      }
      if (stepper2->isRunning() && Pan_move3_Dest > 4 && Pan_move4_Rst != 1 ) {                                   //If the stepper is running on from last move
        stepper2->setAcceleration(k4A);
        s4_speed = s4_speed + ((s4_speed / 100) * 8);
        stepper2->setSpeedInUs(s4_speed);
        stepper2-> applySpeedAcceleration();
        delay(10);
      } else {

        stepper2->setAcceleration(k4A);                                                    //If starting fresh Max accel to catch up on turnarround
        stepper2->setSpeedInUs(s4_speed);

        switch (Pan_move4_Dest) {                                                          //Pick up how far the motor should move before it either stopes or changes direction
          case 5:
            stepper2->moveTo(Pan_k5_position);
            break;
          case 6:
            s4_speed = s4_speed + ((s4_speed / 100) * 8);
            stepper2->setSpeedInUs(s4_speed);
            stepper2->moveTo(Pan_k6_position);
            break;
        }
        delay(10);
      }
      Base_Pan_accel = k4A;
      Last_Pan_accel = k4A;
      Last_Pan_speed = s4_speed;

      ActiveMove = 4;
    } else {
      Stp_2_active = 0;
    }

    //************************Tlt move*****************************************************
    KeyB = Tlt_k5_position;
    KeyA = Tlt_k4_position;
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
          k4A = (KeyDist / (Key_Crono_time));
          k4A = k4A * 3;
          break;
        case 3:
          k4A = (KeyDist / (Key_Crono_time));
          break;

        case 2:
          k4A = (KeyDist / (Key_Crono_time));
          k4A = k4A * 2;
          break;

        case 1:
          k4A = (KeyDist / (Key_Crono_time));
          k4A = k4A * 3;
          break;
      }

      if (stepper1->isRunning() && Tlt_move3_Dest > 4 && Tlt_move4_Rst != 1) {                                           //If the stepper is running on from last move
        stepper1->setAcceleration(k4A);
        s4_speed = s4_speed + ((s4_speed / 100) * 8);
        stepper1->setSpeedInUs(s4_speed);
        stepper1-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper1->setAcceleration(k4A);                                                              //If starting fresh Max accel to catch up on turnarround
        stepper1->setSpeedInUs(s4_speed);

        switch (Tlt_move4_Dest) {                                                                     //Pick up how far the motor should move before it either stopes or changes direction
          case 5:
            stepper1->moveTo(Tlt_k5_position);
            break;
          case 6:
            s4_speed = s4_speed + ((s4_speed / 100) * 8);
            stepper1->setSpeedInUs(s4_speed);
            stepper1->moveTo(Tlt_k6_position);
            break;
        }
        delay(10);
      }
      Base_Tlt_accel = k4A;
      Last_Tlt_accel = k4A;
      Last_Tlt_speed = s4_speed;

      ActiveMove = 4;
    } else {
      Stp_1_active = 0;
    }

    //************************Foc move*****************************************************
    KeyB = Foc_k5_position;
    KeyA = Foc_k4_position;
    Find_KeyDist();
    if (KeyDist != 0 && s5 != 0) {
      Key_Crono_time = int(float(100 / s4) * Seq_Time);                                         //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s4_speed = abs(factor * 1000000);
      Stp_3_active = 1;
      Stp_active = 1;

      switch (a4) {
        case 0:                                                                                   //No ease
          k4A = (KeyDist / (Key_Crono_time));
          k4A = k4A * 3;
          break;
        case 3:
          k4A = (KeyDist / (Key_Crono_time));
          break;

        case 2:
          k4A = (KeyDist / (Key_Crono_time));
          k4A = k4A * 2;
          break;

        case 1:
          k4A = (KeyDist / (Key_Crono_time));
          k4A = k4A * 3;
          break;
      }

      if (stepper3->isRunning() && Foc_move3_Dest > 4 && Foc_move4_Rst != 1) {                                  //If the stepper is running on from last move
        stepper3->setAcceleration(k4A);
        s4_speed = s4_speed + ((s4_speed / 100) * 8);
        stepper3->setSpeedInUs(s4_speed);
        stepper3-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper3->setAcceleration(k4A);                                                              //If starting fresh Max accel to catch up on turnarround
        stepper3->setSpeedInUs(s4_speed);

        switch (Foc_move4_Dest) {                                                                    //Pick up how far the motor should move before it either stopes or changes direction

          case 5:
            stepper3->moveTo(Foc_k5_position);
            break;
          case 6:
            s4_speed = s4_speed + ((s4_speed / 100) * 8);
            stepper1->setSpeedInUs(s4_speed);
            stepper3->moveTo(Foc_k6_position);
            break;
        }
        delay(10);
      }
      Base_Foc_accel = k4A;
      Last_Foc_accel = k4A;
      Last_Foc_speed = s4_speed;

      ActiveMove = 3;
    } else {
      Stp_3_active = 0;
    }
    if ((Stp_1_active != 0) || (Stp_2_active = !0) || (Stp_3_active != 0)) {              //If any of the steppers are moving tell the slider
      delay(1000);
      PT = 2;
      SendNextionValues();
    }
    //************************Slider move*****************************************************
    if (Sld == 2 ) {
      ActiveMove = 1;
      Stp_active = 1;
    }
    //    Time2Ramp2 = (Key_Crono_time / 2) * 1000;
    //    delay(Time2Ramp2);
    //**************************************move 4-5 Second ramp********************************************************************
    //If the running steppers current end point is ahead of this key ajust its speed to match the next keys speed for a smooth entry
    //*******************************************TLT Ramp2**************************************************************************


    //********************************Test to chech for end of move**************************************
    while (Stp_active == 1) {
      if (stepper3->isRunning()) {

        if (Foc_k5_position > Foc_k4_position ) {                                //get direction
          if (stepper3->getCurrentPosition() >= (Foc_k5_position)) {                                     //if + direction
            Stp_3_active = 0;
          }
        } else {
          if (stepper3->getCurrentPosition() <= (Foc_k5_position)) {                                      //if - direction
            Stp_3_active = 0;
          }
        }
      } else {
        Stp_3_active = 0;
      }
      if (stepper1->isRunning() ) {

        if (Tlt_k5_position > Tlt_k4_position ) {                               //get direction
          if (stepper1->getCurrentPosition() >= (Tlt_k5_position)) {                                    //if + direction
            Stp_1_active = 0;
          }
        } else {
          if (stepper1->getCurrentPosition() <= (Tlt_k5_position)) {                                    //if - direction
            Stp_1_active = 0;
          }
        }
      } else {
        Stp_1_active = 0;
      }

      if (stepper2->isRunning()) {
        if (Pan_k5_position > Pan_k4_position ) {                               //get direction
          if (stepper2->getCurrentPosition() >= (Pan_k5_position)) {                                    //if + direction
            Stp_2_active = 0;
          }
        } else {
          if (stepper2->getCurrentPosition() <= (Pan_k5_position)) {                                    //if - direction
            Stp_2_active = 0;
          }
        }
      } else {
        Stp_2_active = 0;
      }

      if (Stp_1_active == 0 && Stp_2_active == 0 && Stp_3_active == 0 ) {                              //Test to see if The PT steppers have finished move
        PT = 1;                                                                                         //Tell the Slider the PT has finished its move
        SendNextionValues();
      }
      if (Stp_1_active == 0 && Stp_2_active == 0 && Stp_3_active == 0 && Sld != 2 ) {                               //Final test to see if all steppers have finished move
        PT = 1;                                                                                         //Tell the Slider the PT has finished its move
        SendNextionValues();
        Stp_active = 0;
      }
    }
  }



  //**********************************************************************************************************
  //****************************************************move5-6***********************************************
  //**********************************************************************************************************
  if (s6 != 0) {
    //    Stp_1_active = 1;                                               //Used by check when move is complete on all steppers
    //    Stp_2_active = 1;
    //    Stp_3_active = 1;
    //    Stp_active = 1;

    //**********************Pan move****************************
    KeyB = Pan_k6_position;
    KeyA = Pan_k5_position;
    Find_KeyDist();
    if (KeyDist != 0 && s6 != 0) {
      Key_Crono_time = int(float(100 / s5) * Seq_Time);                                         //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s5_speed = abs(factor * 1000000);
      Stp_2_active = 1;
      Stp_active = 1;

      switch (a5) {
        case 0:                                                                                   //No ease
          k5A = (KeyDist / (Key_Crono_time));
          k5A = k5A * 3;
          break;
        case 3:
          k5A = (KeyDist / (Key_Crono_time));
          break;

        case 2:
          k5A = (KeyDist / (Key_Crono_time));
          k5A = k5A * 2;
          break;

        case 1:
          k5A = (KeyDist / (Key_Crono_time));
          k5A = k5A * 3;
          break;
      }
      if (stepper2->isRunning() && Pan_move4_Dest > 5 && Pan_move5_Rst != 1 ) {                                   //If the stepper is running on from last move
        stepper2->setAcceleration(k5A);
        s5_speed = s5_speed + ((s5_speed / 100) * 8);
        stepper2->setSpeedInUs(s5_speed);
        stepper2-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper2->setAcceleration(k5A);                                                                           //If starting fresh Max accel to catch up on turnarround
        stepper2->setSpeedInUs(s5_speed);
        stepper2->moveTo(Pan_k6_position);
        delay(10);
      }


      Base_Pan_accel = k5A;
      Last_Pan_accel = k5A;
      Last_Pan_speed = s5_speed;

      ActiveMove = 5;
    } else {
      Stp_2_active = 0;
    }

    //************************Tlt move*****************************************************
    KeyB = Tlt_k6_position;
    KeyA = Tlt_k5_position;
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
          k5A = (KeyDist / (Key_Crono_time));
          k5A = k5A * 3;
          break;
        case 3:
          k5A = (KeyDist / (Key_Crono_time));
          break;

        case 2:
          k5A = (KeyDist / (Key_Crono_time));
          k5A = k5A * 2;
          break;

        case 1:
          k5A = (KeyDist / (Key_Crono_time));
          k5A = k5A * 3;
          break;
      }

      if (stepper1->isRunning() && Tlt_move4_Dest > 5 && Tlt_move5_Rst != 1) {                                  //If the stepper is running on from last move
        stepper1->setAcceleration(k5A);
        s5_speed = s5_speed + ((s5_speed / 100) * 8);
        stepper1->setSpeedInUs(s5_speed);
        stepper1-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper1->setAcceleration(k5A);                                                  //If starting fresh Max accel to catch up on turnarround
        stepper1->setSpeedInUs(s5_speed);
        stepper1->moveTo(Tlt_k6_position);
        delay(10);
      }
      Base_Tlt_accel = k5A;
      Last_Tlt_accel = k5A;
      Last_Tlt_speed = s5_speed;

      ActiveMove = 5;
    } else {
      Stp_1_active = 0;
    }

    //************************Foc move*****************************************************
    KeyB = Foc_k6_position;
    KeyA = Foc_k5_position;
    Find_KeyDist();
    if (KeyDist != 0 && s6 != 0) {
      Key_Crono_time = int(float(100 / s5) * Seq_Time);                                     //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      s5_speed = abs(factor * 1000000);
      Stp_3_active = 1;
      Stp_active = 1;

      switch (a5) {
        case 0:                                                                                //No ease
          k5A = (KeyDist / (Key_Crono_time));
          k5A = k5A * 3;
          break;
        case 3:
          k5A = (KeyDist / (Key_Crono_time));
          break;

        case 2:
          k5A = (KeyDist / (Key_Crono_time));
          k5A = k5A * 2;
          break;

        case 1:
          k5A = (KeyDist / (Key_Crono_time));
          k5A = k5A * 3;
          break;
      }

      if (stepper3->isRunning() && Foc_move4_Dest > 5 && Foc_move5_Rst != 1 ) {                                   //If the stepper is running on from last move
        stepper3->setAcceleration(k5A);
        s5_speed = s5_speed + ((s5_speed / 100) * 8);
        stepper3->setSpeedInUs(s5_speed);
        stepper3-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper3->setAcceleration(k5A);                                                  //If starting fresh Max accel to catch up on turnarround
        stepper3->setSpeedInUs(s5_speed);
        stepper3->moveTo(Foc_k6_position);
        delay(10);
      }
      Base_Foc_accel = k5A;
      Last_Foc_accel = k5A;
      Last_Foc_speed = s5_speed;

      ActiveMove = 5;
    } else {
      Stp_3_active = 0;
    }
    if ((Stp_1_active != 0) || (Stp_2_active = !0) || (Stp_3_active != 0)) {              //If any of the steppers are moving tell the slider
      delay(1000);
      PT = 2;
      SendNextionValues();
    }
    //************************Slider move*****************************************************
    if (Sld == 2 ) {
      ActiveMove = 1;
      Stp_active = 1;
    }
    //    Time2Ramp2 = (Key_Crono_time / 2) * 1000;
    //    delay(Time2Ramp2);
    //**************************************move 5-6 Second ramp********************************************************************
    //If the running steppers current end point is ahead of this key ajust its speed to match the next keys speed for a smooth entry

    //********************************Test to chech for end of move**************************************
    while (Stp_active == 1) {
      if (stepper3->isRunning()) {

        if (Foc_k6_position > Foc_k5_position ) {                                //get direction
          if (stepper3->getCurrentPosition() >= (Foc_k6_position)) {             //if + direction
            Stp_3_active = 0;
          }
        } else {
          if (stepper3->getCurrentPosition() <= (Foc_k6_position)) {              //if - direction
            Stp_3_active = 0;
          }
        }
      } else {
        Stp_3_active = 0;
      }
      if (stepper1->isRunning() ) {

        if (Tlt_k6_position > Tlt_k5_position ) {                               //get direction
          if (stepper1->getCurrentPosition() >= (Tlt_k6_position)) {            //if + direction
            Stp_1_active = 0;
          }
        } else {
          if (stepper1->getCurrentPosition() <= (Tlt_k6_position)) {            //if - direction
            Stp_1_active = 0;
          }
        }
      } else {
        Stp_1_active = 0;
      }

      if (stepper2->isRunning()) {
        if (Pan_k6_position > Pan_k5_position ) {                               //get direction
          if (stepper2->getCurrentPosition() >= (Pan_k6_position)) {            //if + direction
            Stp_2_active = 0;
          }
        } else {
          if (stepper2->getCurrentPosition() <= (Pan_k6_position)) {            //if - direction
            Stp_2_active = 0;
          }
        }
      } else {
        Stp_2_active = 0;
      }

      if (Stp_1_active == 0 && Stp_2_active == 0 && Stp_3_active == 0 ) {                              //Test to see if The PT steppers have finished move
        PT = 1;                                                                                         //Tell the Slider the PT has finished its move
        SendNextionValues();
      }
      if (Stp_1_active == 0 && Stp_2_active == 0 && Stp_3_active == 0 && Sld != 2 ) {        //Final test to see if all steppers have finished move
        PT = 1;                                                                                         //Tell the Slider the PT has finished its move
        SendNextionValues();
        Stp_active = 0;
      }

    }
  }
  //End Game for Sequencer moves

  if (Bounce >= 1)  {
    BounceActive = 1;                                                         //Tell the system that bounce is active

    Bounce = Bounce - 1;

    But_Com = 6;
    if (Sld == 0) {
      SendNextionValues();
    }
    But_Com = 0;

    if (BounceReturn == 1) {
      BounceReturn = 2;                                                        //Reverse move
      Start_6();
    }
    if (BounceReturn == 2) {                                                   //Forward move
      BounceReturn = 1;
      Start_6();
    }

  } else  {                                                                     //Bounce must = 0 Finish the move
    if (BounceReturn == 1 && BounceActive == 1) {                                  //Return for last bounce
      BounceReturn = 2;                                                            //Reverse move
      //BounceActive = 0;
      Start_6();
    } else {
      But_Com = 5;                                                                //Stop the timer
      if (Sld == 0) {
        SendNextionValues();
      }
      But_Com = 0;
      delay(10);
      But_Com = 6;                                                                //Update Nextion bounce
      if (Sld == 0) {
        SendNextionValues();
      }
      But_Com = 0;
      BounceReturn = 1;                                                            //Forward move restored
      delay(10);
      digitalWrite(CAM, HIGH);                                                     //Stop camera
      delay (200);
      digitalWrite(CAM, LOW);
      if (TLY == 1) {
        Tally = 0;                                            //Turn off/On the tally light
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
        FastLED.show();
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
  TLTtravel_dist = abs(TLTout_position - TLTin_position);                     // Distance we have to go
  TLTstep_speed = (TLTtravel_dist / Crono_time);


  PANtravel_dist = abs(PANout_position - PANin_position);                     // Distance we have to go
  PANstep_speed = (PANtravel_dist / Crono_time);


  FOCtravel_dist = abs(FOCout_position - FOCin_position);                     // Distance we have to go
  FOCstep_speed = (FOCtravel_dist / Crono_time);

  ZOOMtravel_dist = abs(ZMout_position - ZMin_position);                  // Distance we have to go
  ZOOMstep_speed = (ZOOMtravel_dist / Crono_time);

  switch (ease_InOut) {
    case 0:                                                                    //No ease
      TLTease_Value = 4000;
      PANease_Value = 4000;
      FOCease_Value = 4000;
      ZOOMease_Value = 4000;
      break;

    case 3:

      PANtravel_dist = abs(PANout_position - PANin_position);                   // Distance we have to go in steps *3 for ramp at start and end
      if (PANtravel_dist != 0) {

        PANease_Value = (PANtravel_dist / (Crono_time * 12));                      //Calculate the Acceloration value for the ramps total distance/ time *12
      } else {
        PANstep_speed = 10;                                                       // If the axis is not in use 0 move you must give it some realistic values or the esp32 will crash!
        PANease_Value = 10;
      }

      TLTtravel_dist = abs(TLTout_position - TLTin_position);
      if (TLTtravel_dist != 0) {

        TLTease_Value = (TLTtravel_dist / (Crono_time * 12));
      } else {
        TLTstep_speed = 10;
        TLTease_Value = 10;
      }

      FOCtravel_dist = abs(FOCout_position - FOCin_position);
      if (FOCtravel_dist != 0) {

        FOCease_Value = (FOCtravel_dist / (Crono_time * 12));
      } else {
        FOCstep_speed = 10;
        FOCease_Value = 10;
      }
      ZOOMtravel_dist = abs(ZMout_position - ZMin_position);
      if (ZOOMtravel_dist != 0) {

        ZOOMease_Value = (ZOOMtravel_dist / (Crono_time * 12));
      } else {
        ZOOMstep_speed = 10;
        ZOOMease_Value = 10;
      }

      break;

    case 2:
      PANtravel_dist = abs(PANout_position - PANin_position);                   // Distance we have to go in steps *3 for ramp at start and end
      if (PANtravel_dist != 0) {

        PANease_Value = (PANtravel_dist / (Crono_time * 12));                      //Calculate the Acceloration value for the ramps total distance/ time *12
        PANease_Value = ( PANease_Value * 2);
      } else {
        PANstep_speed = 10;                                                       // If the axis is not in use 0 move you must give it some realistic values or the esp32 will crash!
        PANease_Value = 10;
      }

      TLTtravel_dist = abs(TLTout_position - TLTin_position);
      if (TLTtravel_dist != 0) {

        TLTease_Value = (TLTtravel_dist / (Crono_time * 12));
        TLTease_Value = (TLTease_Value * 2);
      } else {
        TLTstep_speed = 10;
        TLTease_Value = 10;
      }

      FOCtravel_dist = abs(FOCout_position - FOCin_position);
      if (FOCtravel_dist != 0) {

        FOCease_Value = (FOCtravel_dist / (Crono_time * 12));
        FOCease_Value = (FOCease_Value * 2);
      } else {
        FOCstep_speed = 10;
        FOCease_Value = 10;
      }
      ZOOMtravel_dist = abs(ZMout_position - ZMin_position);
      if (ZOOMtravel_dist != 0) {

        ZOOMease_Value = (ZOOMtravel_dist / (Crono_time * 12));
        ZOOMease_Value = (ZOOMease_Value * 2);
      } else {
        ZOOMstep_speed = 10;
        ZOOMease_Value = 10;
      }
      break;

    case 1:
      PANtravel_dist = abs(PANout_position - PANin_position);                   // Distance we have to go in steps *3 for ramp at start and end
      if (PANtravel_dist != 0) {

        PANease_Value = (PANtravel_dist / (Crono_time * 12));                      //Calculate the Acceloration value for the ramps total distance/ time *12
        PANease_Value = ( PANease_Value * 3);
      } else {
        PANstep_speed = 10;                                                       // If the axis is not in use 0 move you must give it some realistic values or the esp32 will crash!
        PANease_Value = 10;
      }

      TLTtravel_dist = abs(TLTout_position - TLTin_position);
      if (TLTtravel_dist != 0) {

        TLTease_Value = (TLTtravel_dist / (Crono_time * 12));
        TLTease_Value = (TLTease_Value * 3);
      } else {
        TLTstep_speed = 10;
        TLTease_Value = 10;
      }

      FOCtravel_dist = abs(FOCout_position - FOCin_position);
      if (FOCtravel_dist != 0) {

        FOCease_Value = (FOCtravel_dist / (Crono_time * 12));
        FOCease_Value = (FOCease_Value * 3);
      } else {
        FOCstep_speed = 10;
        FOCease_Value = 10;
      }
      ZOOMtravel_dist = abs(ZMout_position - ZMin_position);
      if (ZOOMtravel_dist != 0) {

        ZOOMease_Value = (ZOOMtravel_dist / (Crono_time * 12));
        ZOOMease_Value = (ZOOMease_Value * 3);
      } else {
        ZOOMstep_speed = 10;
        ZOOMease_Value = 10;
      }


      break;
  }

  return;
}



//*************************************** Joystick control********************************************
void ReadAnalog() {                                  //read joystick
  tilt = analogRead(tilt_PIN);                       //Reading the Tilt & Pan potentiometers from joystick
  pan = analogRead(pan_PIN);

  TiltJoySpeed = (tilt - tilt_AVG);
  PanJoySpeed = (pan - pan_AVG);


  if (abs( TiltJoySpeed) > 250) {
    stepper1->setSpeedInUs(Tilt_J_Speed);             //Larger value is slower! Ajust this to ajust the joystick tilt speed
    stepper1->setAcceleration (abs(TiltJoySpeed) * 2);

    if (TiltJoySpeed > 0) {
      stepper1->runBackward();
      //Serial.println("TiltStepper is Backwords");
      while (stepper1->isRunning()) {                          //delay until move complete
        tilt = analogRead(tilt_PIN);
        stepper1->setAcceleration (abs(TiltJoySpeed) * 2);
        stepper1->applySpeedAcceleration();
        if (abs(tilt - tilt_AVG) < 250) {
          //Serial.println("TiltStepper has been stoped");
          stepper1->forceStopAndNewPosition(stepper1->getCurrentPosition());
          return;
        }
      }

    } else {
      stepper1->runForward();
      //Serial.println("TiltStepper is Forwards");
      while (stepper1->isRunning()) {                          //delay until move complete
        tilt = analogRead(tilt_PIN);
        stepper1->setAcceleration (abs(TiltJoySpeed) * 2);
        stepper1->applySpeedAcceleration();
        if (abs(tilt - tilt_AVG) < 250) {
          //Serial.println("TiltStepper has been stoped");
          stepper1->forceStopAndNewPosition(stepper1->getCurrentPosition());
          //stepper1->stopMove();
          return;
        }
      }
    }
  }

  if (abs( PanJoySpeed) > 350) {
    stepper2->setSpeedInUs(Pan_J_Speed);                             //Larger value is slower! Ajust this to ajust the joystick Pan speed 4000 for Gearless drive 200 or less for geared
    stepper2->setAcceleration (abs(PanJoySpeed) / 6);       //  /6 for no gear *2 for geared
    if (PanJoySpeed < 0) {
      stepper2->runBackward();
      while (stepper2->isRunning()) {                           //delay until move complete
        pan = analogRead(pan_PIN);
        PanJoySpeed = ((pan - pan_AVG));
        stepper2->setAcceleration(abs(PanJoySpeed) / 6);       //  /6 for no gear *2 for geared
        stepper2->applySpeedAcceleration();
        if (abs(pan - pan_AVG) < 250) {
          stepper2->forceStopAndNewPosition(stepper2->getCurrentPosition());
          return;
        }
      }
    } else {
      stepper2->runForward();
      while (stepper2->isRunning()) {                           //delay until move complete
        pan = analogRead(pan_PIN);
        PanJoySpeed = ((pan - pan_AVG));
        stepper2->setAcceleration(abs(PanJoySpeed) / 6);
        stepper2->applySpeedAcceleration();
        if (abs(pan - pan_AVG) < 250) {
          stepper2->forceStopAndNewPosition(stepper2->getCurrentPosition());
          return;
        }
      }
    }
  }

}

void InitialValues() {

  //Set the values to zero before averaging
  float temp_tilt = 0;
  float temp_pan = 0;
  for (int i = 0; i < 50; i++)
  {

    temp_tilt += analogRead(tilt_PIN);
    delay(10); //allowing a little time between two readings
    temp_pan += analogRead(pan_PIN);
    delay(10);
  }
  tilt_AVG = temp_tilt / 50;
  pan_AVG = temp_pan / 50;
  stepper3->setCurrentPosition(0);
  F_ResetEncoder = 1;
  stepper4->setCurrentPosition(0);
  Z_ResetEncoder = 1;

  stepper1->setCurrentPosition(0);
  T_ResetEncoder = 1;
  stepper2->setCurrentPosition(0);
  P_ResetEncoder = 1;

  return;
}

//Not used on DB3 head
//void ReadMount() {                                         //Listen for PanTilt mount buttons and execute
//  stepper2->setSpeedInUs(250);                             //Larger value is slower!
//  stepper2->setAcceleration(3000);
//
//  if (digitalRead(Mount_PIN) == LOW) {
//    delay(100); // Debounce the button
//    while  (digitalRead(Mount_PIN) == LOW) {
//      Mount = 1;
//      stepper2-> runForward();
//    }
//  }
//  if (digitalRead(Mount_PIN) == HIGH && Mount == 1) {
//    stepper2->forceStopAndNewPosition(0);                  //Stops dead
//    Mount = 0;
//  }
//
//  if (digitalRead(DisMount_PIN) == LOW) {
//    delay(100); // Debounce the button
//    while  (digitalRead(DisMount_PIN) == LOW) {
//      Mount = 1;
//      stepper2-> runBackward();
//    }
//  }
//
//  if (digitalRead(DisMount_PIN) == HIGH && Mount == 1) {
//    stepper2->forceStopAndNewPosition(0);                  //Stops dead
//    Mount = 0;
//  }
//}

void stepper1_plan() {
  k1_k2_Dir = 0;
  k2_k3_Dir = 0;
  k3_k4_Dir = 0;
  k4_k5_Dir = 0;
  k5_k6_Dir = 0;

  Tlt_move2_Rst = 0;
  Tlt_move3_Rst = 0;
  Tlt_move4_Rst = 0;
  Tlt_move5_Rst = 0;
  //*********************Tlt motor Move1 plan***********************************

  //***********k1-k2 move*************************
  Tlt_move1_Dest = 1;                                            //The starting key
  if (s2 != 0) {
    KeyB = Tlt_k2_position;                                      //check to see if there was a move
    KeyA = Tlt_k1_position;
    Find_KeyDist();
    if (KeyDist != 0) {
      Tlt_move1_Dest = 2;
    } else {                                                     //No move. Next possible move is from 2 so set it but tell move 2 it is a restart not moving on from 1
      Tlt_move1_Dest = 2;
      Tlt_move2_Rst = 1;
    }
  }

  // Test for continouse mov to k3
  if (Tlt_move1_Dest == 2) {
    Tlt_move1_Dest = 2;
    if (Tlt_k2_position > Tlt_k1_position) {                      //check direction if + or -
      k1_k2_Dir = 1;
    }
    if (Tlt_k2_position < Tlt_k1_position) {                      //If not it must be ether less - or 0
      k1_k2_Dir = -1;
    }
    if (Tlt_k2_position == Tlt_k1_position)  {
      k1_k2_Dir = 0;
    }
    //***********k2-k3 move*************************
    if (Tlt_k3_position > Tlt_k2_position) {                      //check direction if + or -
      k2_k3_Dir = 1;
    }
    if (Tlt_k3_position < Tlt_k2_position) {
      k2_k3_Dir = -1;
    }
    if (Tlt_k3_position == Tlt_k2_position) {
      k2_k3_Dir = 0;
    }

    //***************look for direction change**********************
    if ((k2_k3_Dir != 0) && (k1_k2_Dir == k2_k3_Dir)) {
      Tlt_move1_Dest = 3;
    }
  }
  //***********k3-k4 move*************************
  if (Tlt_move1_Dest == 3) {                                   //if the last key advanced check the next
    if (Tlt_k4_position > Tlt_k3_position) {
      k3_k4_Dir = 1;
    }
    if (Tlt_k4_position < Tlt_k3_position) {
      k3_k4_Dir = -1;
    }
    if (Tlt_k4_position == Tlt_k3_position) {
      k3_k4_Dir = 0;
    }

    if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {          //***************look for direction change**********************
      Tlt_move1_Dest = 4;
    }
  }
  //***********k4-k5 move*************************
  if (Tlt_move1_Dest == 4) {                                    //if the last key advanced check the next
    if (Tlt_k5_position > Tlt_k4_position) {                    //check direction if + or -
      k4_k5_Dir = 1;
    }
    if (Tlt_k5_position < Tlt_k4_position) {
      k4_k5_Dir = -1;
    }
    if (Tlt_k5_position == Tlt_k4_position) {
      k4_k5_Dir = 0;
    }

    if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {          //***************look for direction change**********************
      Tlt_move1_Dest = 5;
    }
  }
  //***********k5-k6 move*************************
  if (Tlt_move1_Dest == 5) {
    if (Tlt_k6_position > Tlt_k5_position) {                       //check direction if + or -
      k5_k6_Dir = 1;
    }
    if (Tlt_k6_position < Tlt_k5_position) {
      k5_k6_Dir = -1;
    }
    if (Tlt_k6_position == Tlt_k5_position) {
      k5_k6_Dir = 0;
    }
    if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {           //***************look for direction change**********************
      Tlt_move1_Dest = 6;
    }
  }


  k1_k2_Dir = 0;
  k2_k3_Dir = 0;
  k3_k4_Dir = 0;
  k4_k5_Dir = 0;
  k5_k6_Dir = 0;

  //*********************Tlt motor Move2 plan***********************************

  switch (Tlt_move1_Dest) {                                //Find the full move start if move one only took us to 2 then 2-3 must be a restart for a change in direction


    case 2:
      Tlt_move2_Dest = 2;
      //*********************k1-k2**************************
      if (Tlt_move2_Dest == 2 && s3 != 0) {
        if (Tlt_k2_position > Tlt_k1_position) {                      //check direction if + or -
          k1_k2_Dir = 1;
        }
        if (Tlt_k2_position < Tlt_k1_position) {
          k1_k2_Dir = -1;
        }
        if (Tlt_k2_position == Tlt_k1_position) {
          k1_k2_Dir = 0;
        }

        //*********************k2-k3**************************
        if (Tlt_k3_position > Tlt_k2_position) {                      //check direction if + or -
          k2_k3_Dir = 1;
        }
        if (Tlt_k3_position < Tlt_k2_position) {
          k2_k3_Dir = -1;
        }
        if (Tlt_k3_position == Tlt_k2_position) {
          k2_k3_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k2_k3_Dir != 0) && (k1_k2_Dir == k2_k3_Dir)) {                                                                 // if no direction change
          Tlt_move2_Dest = 3;
        } else {
          Tlt_move2_Dest = 3;
          Tlt_move3_Rst = 1;
        }
      }
      //*********************k3-k4**************************
      if (Tlt_move2_Dest == 3 && s4 != 0) {
        if (Tlt_k4_position > Tlt_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Tlt_k4_position < Tlt_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Tlt_k4_position == Tlt_k3_position) {
          k3_k4_Dir = 0;
        }

        //***************look for direction change**********************
        if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {                                                                 // if no direction change
          Tlt_move2_Dest = 4;
        }
      }
      //***********k4-k5 move*************************
      if (Tlt_move2_Dest == 4 && s5 != 0) {
        if (Tlt_k5_position > Tlt_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Tlt_k5_position < Tlt_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Tlt_k5_position == Tlt_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                        // if no direction change
          Tlt_move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (Tlt_move2_Dest == 5 && s6 != 0) {
        if (Tlt_k6_position > Tlt_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Tlt_k6_position < Tlt_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Tlt_k6_position == Tlt_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Tlt_move2_Dest = 6;
        }
      }
      break;
    case 3:
      Tlt_move2_Dest = 3;


      //***********k2-k3 move*************************
      if (Tlt_move2_Dest == 3 && s4 != 0) {
        if (Tlt_k3_position > Tlt_k2_position) {                      //check direction if + or -
          k2_k3_Dir = 1;
        }
        if (Tlt_k3_position < Tlt_k2_position) {
          k2_k3_Dir = -1;
        }
        if (Tlt_k3_position == Tlt_k2_position) {
          k2_k3_Dir = 0;
        }

        //***********k3-k4 move*************************
        if (Tlt_k4_position > Tlt_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Tlt_k4_position < Tlt_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Tlt_k4_position == Tlt_k3_position) {
          k3_k4_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {                        // if no direction change
          Tlt_move2_Dest = 4;
        }
      }
      //***********k4-k5 move*************************
      if (Tlt_move2_Dest == 4 && s5 != 0) {
        if (Tlt_k5_position > Tlt_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Tlt_k5_position < Tlt_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Tlt_k5_position == Tlt_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                        // if no direction change
          Tlt_move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (Tlt_move2_Dest == 5 && s6 != 0) {

        if (Tlt_k6_position > Tlt_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Tlt_k6_position < Tlt_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Tlt_k6_position == Tlt_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Tlt_move2_Dest = 6;
        }
      }
      break;
    case 4:
      Tlt_move2_Dest = 4;

      //***********k3-k4 move*************************

      if (Tlt_move2_Dest == 4 && s5 != 0) {
        if (Tlt_k4_position > Tlt_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Tlt_k4_position < Tlt_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Tlt_k4_position == Tlt_k3_position) {
          k3_k4_Dir = 0;
        }
        //***********k4-k5 move*************************
        if (Tlt_k5_position > Tlt_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Tlt_k5_position < Tlt_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Tlt_k5_position == Tlt_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          Tlt_move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (Tlt_move2_Dest == 5 && s6 != 0) {
        if (Tlt_k6_position > Tlt_k5_position) {                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Tlt_k6_position < Tlt_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Tlt_k6_position == Tlt_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Tlt_move2_Dest = 6;
        }
      }
      break;

    case 5:
      Tlt_move2_Dest = 5;
      //***********k4-k5 move*************************

      if (Tlt_move2_Dest == 5 && s6 != 0) {
        if (Tlt_k5_position > Tlt_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Tlt_k5_position < Tlt_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Tlt_k5_position == Tlt_k4_position) {
          k4_k5_Dir = 0;
        }

        //***********k5-k6 move*************************
        if (Tlt_k6_position > Tlt_k5_position) {                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Tlt_k6_position < Tlt_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Tlt_k6_position == Tlt_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Tlt_move2_Dest = 6;
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

  switch (Tlt_move2_Dest) {                                                        //Find the full move start

    case 3:

      Tlt_move3_Dest = 3;
      //***********k2-k3 move*************************
      if (Tlt_move3_Dest == 3 && s4 != 0) {
        if (Tlt_k3_position > Tlt_k2_position) {                                  //check direction if + or -
          k2_k3_Dir = 1;
        }
        if (Tlt_k3_position < Tlt_k4_position) {
          k2_k3_Dir = -1;
        }
        if (Tlt_k3_position == Tlt_k4_position) {
          k2_k3_Dir = 0;
        }
        //***********k3-k4 move*************************

        if (Tlt_k4_position > Tlt_k3_position) {                                  //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Tlt_k4_position < Tlt_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Tlt_k4_position == Tlt_k3_position) {
          k3_k4_Dir = 0;
        }

        //***************look for direction change**********************
        if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {                          // if no direction change
          Tlt_move3_Dest = 4;
        } else {
          Tlt_move3_Dest = 4;
          Tlt_move4_Rst = 1;
        }
      }
      //***********k4-k5 move*************************
      if (Tlt_move3_Dest == 4 && s5 != 0) {
        if (Tlt_k5_position > Tlt_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Tlt_k5_position < Tlt_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Tlt_k5_position == Tlt_k4_position) {
          k4_k5_Dir = 0;
        }
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          Tlt_move3_Dest = 5;
        }
      }
      //***********Test for moving on 3-6
      if (Tlt_move3_Dest == 5 && s6 != 0) {
        if (Tlt_k6_position > Tlt_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Tlt_k6_position < Tlt_k5_position) {
          k5_k6_Dir = 0;
        }
        if (Tlt_k6_position == Tlt_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Tlt_move3_Dest = 6;
        }
      }
      break;
    case 4:
      Tlt_move3_Dest = 4;

      //***********k3-k4 move*************************
      if (Tlt_move3_Dest == 4 && s5 != 0) {
        if (Tlt_k4_position > Tlt_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Tlt_k4_position < Tlt_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Tlt_k4_position == Tlt_k3_position) {
          k3_k4_Dir = 0;
        }

        //***********k4-k5 move*************************

        if (Tlt_k5_position > Tlt_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Tlt_k5_position < Tlt_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Tlt_k5_position == Tlt_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          Tlt_move3_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (Tlt_move3_Dest == 5 && s6 != 0) {
        if (Tlt_k6_position > Tlt_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Tlt_k6_position < Tlt_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Tlt_k6_position == Tlt_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Tlt_move3_Dest = 6;
        }
      }
      break;
    case 5:
      Tlt_move3_Dest = 5;
      //***********k4-k5 move*************************
      if (Tlt_move3_Dest == 5 && s6 != 0) {
        if (Tlt_k5_position > Tlt_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Tlt_k5_position < Tlt_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Tlt_k5_position == Tlt_k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************
        if (Tlt_k6_position > Tlt_k5_position) {                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Tlt_k6_position < Tlt_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Tlt_k6_position == Tlt_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Tlt_move3_Dest = 6;
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

  switch (Tlt_move3_Dest) {                                //Find the full move start


    case 4:

      Tlt_move4_Dest = 4;


      //***********k3-k4 move*************************is it going on to 6
      if (Tlt_move4_Dest == 4 && s5 != 0) {
        if (Tlt_k4_position > Tlt_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Tlt_k4_position < Tlt_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Tlt_k4_position == Tlt_k3_position) {
          k3_k4_Dir = 0;
        }
        //***********k4-k5 move*************************is it going on to 6

        if (Tlt_k5_position > Tlt_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Tlt_k5_position < Tlt_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Tlt_k5_position == Tlt_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          Tlt_move4_Dest = 5;
        } else {
          Tlt_move4_Dest = 5;
          Tlt_move5_Rst = 1;
        }
      }
      //***********k5-k6 move*************************
      if (Tlt_move4_Dest == 5 && s6 != 0) {
        if (Tlt_k6_position > Tlt_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Tlt_k6_position < Tlt_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Tlt_k6_position == Tlt_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Tlt_move4_Dest = 6;
        }
      }
      break;
    case 5:
      Tlt_move4_Rst = 1;
      Tlt_move4_Dest = 5;
      //***********k4-k5 move*************************is it going on to 6
      if (Tlt_move4_Dest == 5 && s6 != 0) {
        if (Tlt_k5_position > Tlt_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Tlt_k5_position < Tlt_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Tlt_k5_position == Tlt_k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************

        if (Tlt_k6_position > Tlt_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Tlt_k6_position < Tlt_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Tlt_k6_position == Tlt_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Tlt_move4_Dest = 6;
        }
      }
      break;
  }
  //*********************Tlt motor Move5 plan***********************************

  switch (Tlt_move4_Dest) {                                                       //Find the full move start

    case 5:

      Tlt_move4_Dest = 5;

      //***********k4-k5 move*************************is it going on to 6
      if (Tlt_move5_Dest == 5 && s6 != 0) {
        if (Tlt_k5_position > Tlt_k4_position) {                                    //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Tlt_k5_position < Tlt_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Tlt_k5_position == Tlt_k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************is it going on to 6

        if (Tlt_k6_position > Tlt_k5_position) {                                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Tlt_k6_position < Tlt_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Tlt_k6_position == Tlt_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Tlt_move5_Dest = 6;
        } else {
          Tlt_move5_Dest = 6;
          //Tlt_move6_Rst = 1;
        }
      }
      break;
  }
  return;
}

void stepper2_plan() {
  k1_k2_Dir = 0;
  k2_k3_Dir = 0;
  k3_k4_Dir = 0;
  k4_k5_Dir = 0;
  k5_k6_Dir = 0;

  Pan_move2_Rst = 0;
  Pan_move3_Rst = 0;
  Pan_move4_Rst = 0;
  Pan_move5_Rst = 0;
  //*********************Pan motor Move1 plan***********************************

  //***********k1-k2 move*************************
  Pan_move1_Dest = 1;                                            //The starting key
  if (s2 != 0) {
    KeyB = Pan_k2_position;                                      //check to see if there was a move
    KeyA = Pan_k1_position;
    Find_KeyDist();
    if (KeyDist != 0) {
      Pan_move1_Dest = 2;
    } else {                                                     //No move. Next possible move is from 2 so set it but tell move 2 it is a restart not moving on from 1
      Pan_move1_Dest = 2;
      Pan_move2_Rst = 1;
    }
  }

  // Test for continouse mov to k3
  if (Pan_move1_Dest == 2) {
    Pan_move1_Dest = 2;
    if (Pan_k2_position > Pan_k1_position) {                      //check direction if + or -
      k1_k2_Dir = 1;
    }
    if (Pan_k2_position < Pan_k1_position) {                      //If not it must be ether less - or 0
      k1_k2_Dir = -1;
    }
    if (Pan_k2_position == Pan_k1_position)  {
      k1_k2_Dir = 0;
    }
    //***********k2-k3 move*************************
    if (Pan_k3_position > Pan_k2_position) {                      //check direction if + or -
      k2_k3_Dir = 1;
    }
    if (Pan_k3_position < Pan_k2_position) {
      k2_k3_Dir = -1;
    }
    if (Pan_k3_position == Pan_k2_position) {
      k2_k3_Dir = 0;
    }

    //***************look for direction change**********************
    if ((k2_k3_Dir != 0) && (k1_k2_Dir == k2_k3_Dir)) {
      Pan_move1_Dest = 3;
    }
  }
  //***********k3-k4 move*************************
  if (Pan_move1_Dest == 3) {                                   //if the last key advanced check the next
    if (Pan_k4_position > Pan_k3_position) {
      k3_k4_Dir = 1;
    }
    if (Pan_k4_position < Pan_k3_position) {
      k3_k4_Dir = -1;
    }
    if (Pan_k4_position == Pan_k3_position) {
      k3_k4_Dir = 0;
    }

    if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {          //***************look for direction change**********************
      Pan_move1_Dest = 4;
    }
  }
  //***********k4-k5 move*************************
  if (Pan_move1_Dest == 4) {                                    //if the last key advanced check the next
    if (Pan_k5_position > Pan_k4_position) {                    //check direction if + or -
      k4_k5_Dir = 1;
    }
    if (Pan_k5_position < Pan_k4_position) {
      k4_k5_Dir = -1;
    }
    if (Pan_k5_position == Pan_k4_position) {
      k4_k5_Dir = 0;
    }

    if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {          //***************look for direction change**********************
      Pan_move1_Dest = 5;
    }
  }
  //***********k5-k6 move*************************
  if (Pan_move1_Dest == 5) {
    if (Pan_k6_position > Pan_k5_position) {                       //check direction if + or -
      k5_k6_Dir = 1;
    }
    if (Pan_k6_position < Pan_k5_position) {
      k5_k6_Dir = -1;
    }
    if (Pan_k6_position == Pan_k5_position) {
      k5_k6_Dir = 0;
    }
    if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {           //***************look for direction change**********************
      Pan_move1_Dest = 6;
    }
  }


  k1_k2_Dir = 0;
  k2_k3_Dir = 0;
  k3_k4_Dir = 0;
  k4_k5_Dir = 0;
  k5_k6_Dir = 0;

  //*********************Pan motor Move2 plan***********************************

  switch (Pan_move1_Dest) {                                //Find the full move start if move one only took us to 2 then 2-3 must be a restart for a change in direction


    case 2:
      Pan_move2_Dest = 2;
      //*********************k1-k2**************************
      if (Pan_move2_Dest == 2 && s3 != 0) {
        if (Pan_k2_position > Pan_k1_position) {                      //check direction if + or -
          k1_k2_Dir = 1;
        }
        if (Pan_k2_position < Pan_k1_position) {
          k1_k2_Dir = -1;
        }
        if (Pan_k2_position == Pan_k1_position) {
          k1_k2_Dir = 0;
        }

        //*********************k2-k3**************************
        if (Pan_k3_position > Pan_k2_position) {                      //check direction if + or -
          k2_k3_Dir = 1;
        }
        if (Pan_k3_position < Pan_k2_position) {
          k2_k3_Dir = -1;
        }
        if (Pan_k3_position == Pan_k2_position) {
          k2_k3_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k2_k3_Dir != 0) && (k1_k2_Dir == k2_k3_Dir)) {                                                                 // if no direction change
          Pan_move2_Dest = 3;
        } else {
          Pan_move2_Dest = 3;
          Pan_move3_Rst = 1;
        }
      }
      //*********************k3-k4**************************
      if (Pan_move2_Dest == 3 && s4 != 0) {
        if (Pan_k4_position > Pan_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Pan_k4_position < Pan_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Pan_k4_position == Pan_k3_position) {
          k3_k4_Dir = 0;
        }

        //***************look for direction change**********************
        if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {                                                                 // if no direction change
          Pan_move2_Dest = 4;
        }
      }
      //***********k4-k5 move*************************
      if (Pan_move2_Dest == 4 && s5 != 0) {
        if (Pan_k5_position > Pan_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Pan_k5_position < Pan_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Pan_k5_position == Pan_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                        // if no direction change
          Pan_move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (Pan_move2_Dest == 5 && s6 != 0) {
        if (Pan_k6_position > Pan_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Pan_k6_position < Pan_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Pan_k6_position == Pan_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Pan_move2_Dest = 6;
        }
      }
      break;
    case 3:
      Pan_move2_Dest = 3;


      //***********k2-k3 move*************************
      if (Pan_move2_Dest == 3 && s4 != 0) {
        if (Pan_k3_position > Pan_k2_position) {                      //check direction if + or -
          k2_k3_Dir = 1;
        }
        if (Pan_k3_position < Pan_k2_position) {
          k2_k3_Dir = -1;
        }
        if (Pan_k3_position == Pan_k2_position) {
          k2_k3_Dir = 0;
        }

        //***********k3-k4 move*************************
        if (Pan_k4_position > Pan_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Pan_k4_position < Pan_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Pan_k4_position == Pan_k3_position) {
          k3_k4_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {                        // if no direction change
          Pan_move2_Dest = 4;
        }
      }
      //***********k4-k5 move*************************
      if (Pan_move2_Dest == 4 && s5 != 0) {
        if (Pan_k5_position > Pan_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Pan_k5_position < Pan_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Pan_k5_position == Pan_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                        // if no direction change
          Pan_move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (Pan_move2_Dest == 5 && s6 != 0) {

        if (Pan_k6_position > Pan_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Pan_k6_position < Pan_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Pan_k6_position == Pan_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Pan_move2_Dest = 6;
        }
      }
      break;
    case 4:
      Pan_move2_Dest = 4;

      //***********k3-k4 move*************************

      if (Pan_move2_Dest == 4 && s5 != 0) {
        if (Pan_k4_position > Pan_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Pan_k4_position < Pan_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Pan_k4_position == Pan_k3_position) {
          k3_k4_Dir = 0;
        }
        //***********k4-k5 move*************************
        if (Pan_k5_position > Pan_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Pan_k5_position < Pan_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Pan_k5_position == Pan_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          Pan_move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (Pan_move2_Dest == 5 && s6 != 0) {
        if (Pan_k6_position > Pan_k5_position) {                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Pan_k6_position < Pan_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Pan_k6_position == Pan_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Pan_move2_Dest = 6;
        }
      }
      break;

    case 5:
      Pan_move2_Dest = 5;
      //***********k4-k5 move*************************

      if (Pan_move2_Dest == 5 && s6 != 0) {
        if (Pan_k5_position > Pan_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Pan_k5_position < Pan_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Pan_k5_position == Pan_k4_position) {
          k4_k5_Dir = 0;
        }

        //***********k5-k6 move*************************
        if (Pan_k6_position > Pan_k5_position) {                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Pan_k6_position < Pan_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Pan_k6_position == Pan_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Pan_move2_Dest = 6;
        }
      }
      break;
  }

  k1_k2_Dir = 0;
  k2_k3_Dir = 0;
  k3_k4_Dir = 0;
  k4_k5_Dir = 0;
  k5_k6_Dir = 0;
  //*********************Pan motor Move3 plan***********************************

  switch (Pan_move2_Dest) {                                                        //Find the full move start

    case 3:

      Pan_move3_Dest = 3;
      //***********k2-k3 move*************************
      if (Pan_move3_Dest == 3 && s4 != 0) {
        if (Pan_k3_position > Pan_k2_position) {                                  //check direction if + or -
          k2_k3_Dir = 1;
        }
        if (Pan_k3_position < Pan_k4_position) {
          k2_k3_Dir = -1;
        }
        if (Pan_k3_position == Pan_k4_position) {
          k2_k3_Dir = 0;
        }
        //***********k3-k4 move*************************

        if (Pan_k4_position > Pan_k3_position) {                                  //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Pan_k4_position < Pan_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Pan_k4_position == Pan_k3_position) {
          k3_k4_Dir = 0;
        }

        //***************look for direction change**********************
        if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {                          // if no direction change
          Pan_move3_Dest = 4;
        } else {
          Pan_move3_Dest = 4;
          Pan_move4_Rst = 1;
        }
      }
      //***********k4-k5 move*************************
      if (Pan_move3_Dest == 4 && s5 != 0) {
        if (Pan_k5_position > Pan_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Pan_k5_position < Pan_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Pan_k5_position == Pan_k4_position) {
          k4_k5_Dir = 0;
        }
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          Pan_move3_Dest = 5;
        }
      }
      //***********Test for moving on 3-6
      if (Pan_move3_Dest == 5 && s6 != 0) {
        if (Pan_k6_position > Pan_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Pan_k6_position < Pan_k5_position) {
          k5_k6_Dir = 0;
        }
        if (Pan_k6_position == Pan_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Pan_move3_Dest = 6;
        }
      }
      break;
    case 4:
      Pan_move3_Dest = 4;

      //***********k3-k4 move*************************
      if (Pan_move3_Dest == 4 && s5 != 0) {
        if (Pan_k4_position > Pan_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Pan_k4_position < Pan_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Pan_k4_position == Pan_k3_position) {
          k3_k4_Dir = 0;
        }

        //***********k4-k5 move*************************

        if (Pan_k5_position > Pan_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Pan_k5_position < Pan_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Pan_k5_position == Pan_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          Pan_move3_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (Pan_move3_Dest == 5 && s6 != 0) {
        if (Pan_k6_position > Pan_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Pan_k6_position < Pan_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Pan_k6_position == Pan_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Pan_move3_Dest = 6;
        }
      }
      break;
    case 5:
      Pan_move3_Dest = 5;
      //***********k4-k5 move*************************
      if (Pan_move3_Dest == 5 && s6 != 0) {
        if (Pan_k5_position > Pan_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Pan_k5_position < Pan_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Pan_k5_position == Pan_k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************
        if (Pan_k6_position > Pan_k5_position) {                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Pan_k6_position < Pan_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Pan_k6_position == Pan_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Pan_move3_Dest = 6;
        }
      }
      break;
  }
  k1_k2_Dir = 0;
  k2_k3_Dir = 0;
  k3_k4_Dir = 0;
  k4_k5_Dir = 0;
  k5_k6_Dir = 0;
  //*********************Pan motor Move4 plan***********************************

  switch (Pan_move3_Dest) {                                //Find the full move start


    case 4:

      Pan_move4_Dest = 4;


      //***********k3-k4 move*************************is it going on to 6
      if (Pan_move4_Dest == 4 && s5 != 0) {
        if (Pan_k4_position > Pan_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Pan_k4_position < Pan_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Pan_k4_position == Pan_k3_position) {
          k3_k4_Dir = 0;
        }
        //***********k4-k5 move*************************is it going on to 6

        if (Pan_k5_position > Pan_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Pan_k5_position < Pan_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Pan_k5_position == Pan_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          Pan_move4_Dest = 5;
        } else {
          Pan_move4_Dest = 5;
          Pan_move5_Rst = 1;
        }
      }
      //***********k5-k6 move*************************
      if (Pan_move4_Dest == 5 && s6 != 0) {
        if (Pan_k6_position > Pan_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Pan_k6_position < Pan_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Pan_k6_position == Pan_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Pan_move4_Dest = 6;
        }
      }
      break;
    case 5:
      Pan_move4_Rst = 1;
      Pan_move4_Dest = 5;
      //***********k4-k5 move*************************is it going on to 6
      if (Pan_move4_Dest == 5 && s6 != 0) {
        if (Pan_k5_position > Pan_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Pan_k5_position < Pan_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Pan_k5_position == Pan_k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************

        if (Pan_k6_position > Pan_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Pan_k6_position < Pan_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Pan_k6_position == Pan_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Pan_move4_Dest = 6;
        }
      }
      break;
  }
  //*********************Pan motor Move5 plan***********************************

  switch (Pan_move4_Dest) {                                                       //Find the full move start

    case 5:

      Pan_move4_Dest = 5;

      //***********k4-k5 move*************************is it going on to 6
      if (Pan_move5_Dest == 5 && s6 != 0) {
        if (Pan_k5_position > Pan_k4_position) {                                    //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Pan_k5_position < Pan_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Pan_k5_position == Pan_k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************is it going on to 6

        if (Pan_k6_position > Pan_k5_position) {                                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Pan_k6_position < Pan_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Pan_k6_position == Pan_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Pan_move5_Dest = 6;
        } else {
          Pan_move5_Dest = 6;
          //Pan_move6_Rst = 1;
        }
      }
      break;
  }
  return;
}

void stepper3_plan() {
  k1_k2_Dir = 0;
  k2_k3_Dir = 0;
  k3_k4_Dir = 0;
  k4_k5_Dir = 0;
  k5_k6_Dir = 0;

  Foc_move2_Rst = 0;
  Foc_move3_Rst = 0;
  Foc_move4_Rst = 0;
  Foc_move5_Rst = 0;
  //*********************Foc motor Move1 plan***********************************

  //***********k1-k2 move*************************
  Foc_move1_Dest = 1;                                            //The starting key
  if (s2 != 0) {
    KeyB = Foc_k2_position;                                      //check to see if there was a move
    KeyA = Foc_k1_position;
    Find_KeyDist();
    if (KeyDist != 0) {
      Foc_move1_Dest = 2;
    } else {                                                     //No move. Next possible move is from 2 so set it but tell move 2 it is a restart not moving on from 1
      Foc_move1_Dest = 2;
      Foc_move2_Rst = 1;
    }
  }

  // Test for continouse mov to k3
  if (Foc_move1_Dest == 2) {
    Foc_move1_Dest = 2;
    if (Foc_k2_position > Foc_k1_position) {                      //check direction if + or -
      k1_k2_Dir = 1;
    }
    if (Foc_k2_position < Foc_k1_position) {                      //If not it must be ether less - or 0
      k1_k2_Dir = -1;
    }
    if (Foc_k2_position == Foc_k1_position)  {
      k1_k2_Dir = 0;
    }
    //***********k2-k3 move*************************
    if (Foc_k3_position > Foc_k2_position) {                      //check direction if + or -
      k2_k3_Dir = 1;
    }
    if (Foc_k3_position < Foc_k2_position) {
      k2_k3_Dir = -1;
    }
    if (Foc_k3_position == Foc_k2_position) {
      k2_k3_Dir = 0;
    }

    //***************look for direction change**********************
    if ((k2_k3_Dir != 0) && (k1_k2_Dir == k2_k3_Dir)) {
      Foc_move1_Dest = 3;
    }
  }
  //***********k3-k4 move*************************
  if (Foc_move1_Dest == 3) {                                   //if the last key advanced check the next
    if (Foc_k4_position > Foc_k3_position) {
      k3_k4_Dir = 1;
    }
    if (Foc_k4_position < Foc_k3_position) {
      k3_k4_Dir = -1;
    }
    if (Foc_k4_position == Foc_k3_position) {
      k3_k4_Dir = 0;
    }

    if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {          //***************look for direction change**********************
      Foc_move1_Dest = 4;
    }
  }
  //***********k4-k5 move*************************
  if (Foc_move1_Dest == 4) {                                    //if the last key advanced check the next
    if (Foc_k5_position > Foc_k4_position) {                    //check direction if + or -
      k4_k5_Dir = 1;
    }
    if (Foc_k5_position < Foc_k4_position) {
      k4_k5_Dir = -1;
    }
    if (Foc_k5_position == Foc_k4_position) {
      k4_k5_Dir = 0;
    }

    if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {          //***************look for direction change**********************
      Foc_move1_Dest = 5;
    }
  }
  //***********k5-k6 move*************************
  if (Foc_move1_Dest == 5) {
    if (Foc_k6_position > Foc_k5_position) {                       //check direction if + or -
      k5_k6_Dir = 1;
    }
    if (Foc_k6_position < Foc_k5_position) {
      k5_k6_Dir = -1;
    }
    if (Foc_k6_position == Foc_k5_position) {
      k5_k6_Dir = 0;
    }
    if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {           //***************look for direction change**********************
      Foc_move1_Dest = 6;
    }
  }


  k1_k2_Dir = 0;
  k2_k3_Dir = 0;
  k3_k4_Dir = 0;
  k4_k5_Dir = 0;
  k5_k6_Dir = 0;

  //*********************Foc motor Move2 plan***********************************

  switch (Foc_move1_Dest) {                                //Find the full move start if move one only took us to 2 then 2-3 must be a restart for a change in direction


    case 2:
      Foc_move2_Dest = 2;
      //*********************k1-k2**************************
      if (Foc_move2_Dest == 2 && s3 != 0) {
        if (Foc_k2_position > Foc_k1_position) {                      //check direction if + or -
          k1_k2_Dir = 1;
        }
        if (Foc_k2_position < Foc_k1_position) {
          k1_k2_Dir = -1;
        }
        if (Foc_k2_position == Foc_k1_position) {
          k1_k2_Dir = 0;
        }

        //*********************k2-k3**************************
        if (Foc_k3_position > Foc_k2_position) {                      //check direction if + or -
          k2_k3_Dir = 1;
        }
        if (Foc_k3_position < Foc_k2_position) {
          k2_k3_Dir = -1;
        }
        if (Foc_k3_position == Foc_k2_position) {
          k2_k3_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k2_k3_Dir != 0) && (k1_k2_Dir == k2_k3_Dir)) {                                                                 // if no direction change
          Foc_move2_Dest = 3;
        } else {
          Foc_move2_Dest = 3;
          Foc_move3_Rst = 1;
        }
      }
      //*********************k3-k4**************************
      if (Foc_move2_Dest == 3 && s4 != 0) {
        if (Foc_k4_position > Foc_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Foc_k4_position < Foc_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Foc_k4_position == Foc_k3_position) {
          k3_k4_Dir = 0;
        }

        //***************look for direction change**********************
        if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {                                                                 // if no direction change
          Foc_move2_Dest = 4;
        }
      }
      //***********k4-k5 move*************************
      if (Foc_move2_Dest == 4 && s5 != 0) {
        if (Foc_k5_position > Foc_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Foc_k5_position < Foc_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Foc_k5_position == Foc_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                        // if no direction change
          Foc_move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (Foc_move2_Dest == 5 && s6 != 0) {
        if (Foc_k6_position > Foc_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Foc_k6_position < Foc_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Foc_k6_position == Foc_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Foc_move2_Dest = 6;
        }
      }
      break;
    case 3:
      Foc_move2_Dest = 3;


      //***********k2-k3 move*************************
      if (Foc_move2_Dest == 3 && s4 != 0) {
        if (Foc_k3_position > Foc_k2_position) {                      //check direction if + or -
          k2_k3_Dir = 1;
        }
        if (Foc_k3_position < Foc_k2_position) {
          k2_k3_Dir = -1;
        }
        if (Foc_k3_position == Foc_k2_position) {
          k2_k3_Dir = 0;
        }

        //***********k3-k4 move*************************
        if (Foc_k4_position > Foc_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Foc_k4_position < Foc_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Foc_k4_position == Foc_k3_position) {
          k3_k4_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {                        // if no direction change
          Foc_move2_Dest = 4;
        }
      }
      //***********k4-k5 move*************************
      if (Foc_move2_Dest == 4 && s5 != 0) {
        if (Foc_k5_position > Foc_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Foc_k5_position < Foc_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Foc_k5_position == Foc_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                        // if no direction change
          Foc_move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (Foc_move2_Dest == 5 && s6 != 0) {

        if (Foc_k6_position > Foc_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Foc_k6_position < Foc_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Foc_k6_position == Foc_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Foc_move2_Dest = 6;
        }
      }
      break;
    case 4:
      Foc_move2_Dest = 4;

      //***********k3-k4 move*************************

      if (Foc_move2_Dest == 4 && s5 != 0) {
        if (Foc_k4_position > Foc_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Foc_k4_position < Foc_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Foc_k4_position == Foc_k3_position) {
          k3_k4_Dir = 0;
        }
        //***********k4-k5 move*************************
        if (Foc_k5_position > Foc_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Foc_k5_position < Foc_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Foc_k5_position == Foc_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          Foc_move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (Foc_move2_Dest == 5 && s6 != 0) {
        if (Foc_k6_position > Foc_k5_position) {                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Foc_k6_position < Foc_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Foc_k6_position == Foc_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Foc_move2_Dest = 6;
        }
      }
      break;

    case 5:
      Foc_move2_Dest = 5;
      //***********k4-k5 move*************************

      if (Foc_move2_Dest == 5 && s6 != 0) {
        if (Foc_k5_position > Foc_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Foc_k5_position < Foc_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Foc_k5_position == Foc_k4_position) {
          k4_k5_Dir = 0;
        }

        //***********k5-k6 move*************************
        if (Foc_k6_position > Foc_k5_position) {                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Foc_k6_position < Foc_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Foc_k6_position == Foc_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Foc_move2_Dest = 6;
        }
      }
      break;
  }

  k1_k2_Dir = 0;
  k2_k3_Dir = 0;
  k3_k4_Dir = 0;
  k4_k5_Dir = 0;
  k5_k6_Dir = 0;
  //*********************Foc motor Move3 plan***********************************

  switch (Foc_move2_Dest) {                                                        //Find the full move start

    case 3:

      Foc_move3_Dest = 3;
      //***********k2-k3 move*************************
      if (Foc_move3_Dest == 3 && s4 != 0) {
        if (Foc_k3_position > Foc_k2_position) {                                  //check direction if + or -
          k2_k3_Dir = 1;
        }
        if (Foc_k3_position < Foc_k4_position) {
          k2_k3_Dir = -1;
        }
        if (Foc_k3_position == Foc_k4_position) {
          k2_k3_Dir = 0;
        }
        //***********k3-k4 move*************************

        if (Foc_k4_position > Foc_k3_position) {                                  //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Foc_k4_position < Foc_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Foc_k4_position == Foc_k3_position) {
          k3_k4_Dir = 0;
        }

        //***************look for direction change**********************
        if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {                          // if no direction change
          Foc_move3_Dest = 4;
        } else {
          Foc_move3_Dest = 4;
          Foc_move4_Rst = 1;
        }
      }
      //***********k4-k5 move*************************
      if (Foc_move3_Dest == 4 && s5 != 0) {
        if (Foc_k5_position > Foc_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Foc_k5_position < Foc_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Foc_k5_position == Foc_k4_position) {
          k4_k5_Dir = 0;
        }
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          Foc_move3_Dest = 5;
        }
      }
      //***********Test for moving on 3-6
      if (Foc_move3_Dest == 5 && s6 != 0) {
        if (Foc_k6_position > Foc_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Foc_k6_position < Foc_k5_position) {
          k5_k6_Dir = 0;
        }
        if (Foc_k6_position == Foc_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Foc_move3_Dest = 6;
        }
      }
      break;
    case 4:
      Foc_move3_Dest = 4;

      //***********k3-k4 move*************************
      if (Foc_move3_Dest == 4 && s5 != 0) {
        if (Foc_k4_position > Foc_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Foc_k4_position < Foc_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Foc_k4_position == Foc_k3_position) {
          k3_k4_Dir = 0;
        }

        //***********k4-k5 move*************************

        if (Foc_k5_position > Foc_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Foc_k5_position < Foc_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Foc_k5_position == Foc_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          Foc_move3_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (Foc_move3_Dest == 5 && s6 != 0) {
        if (Foc_k6_position > Foc_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Foc_k6_position < Foc_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Foc_k6_position == Foc_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Foc_move3_Dest = 6;
        }
      }
      break;
    case 5:
      Foc_move3_Dest = 5;
      //***********k4-k5 move*************************
      if (Foc_move3_Dest == 5 && s6 != 0) {
        if (Foc_k5_position > Foc_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Foc_k5_position < Foc_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Foc_k5_position == Foc_k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************
        if (Foc_k6_position > Foc_k5_position) {                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Foc_k6_position < Foc_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Foc_k6_position == Foc_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Foc_move3_Dest = 6;
        }
      }
      break;
  }
  k1_k2_Dir = 0;
  k2_k3_Dir = 0;
  k3_k4_Dir = 0;
  k4_k5_Dir = 0;
  k5_k6_Dir = 0;
  //*********************Foc motor Move4 plan***********************************

  switch (Foc_move3_Dest) {                                //Find the full move start


    case 4:

      Foc_move4_Dest = 4;


      //***********k3-k4 move*************************is it going on to 6
      if (Foc_move4_Dest == 4 && s5 != 0) {
        if (Foc_k4_position > Foc_k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (Foc_k4_position < Foc_k3_position) {
          k3_k4_Dir = -1;
        }
        if (Foc_k4_position == Foc_k3_position) {
          k3_k4_Dir = 0;
        }
        //***********k4-k5 move*************************is it going on to 6

        if (Foc_k5_position > Foc_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Foc_k5_position < Foc_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Foc_k5_position == Foc_k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          Foc_move4_Dest = 5;
        } else {
          Foc_move4_Dest = 5;
          Foc_move5_Rst = 1;
        }
      }
      //***********k5-k6 move*************************
      if (Foc_move4_Dest == 5 && s6 != 0) {
        if (Foc_k6_position > Foc_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Foc_k6_position < Foc_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Foc_k6_position == Foc_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Foc_move4_Dest = 6;
        }
      }
      break;
    case 5:
      Foc_move4_Rst = 1;
      Foc_move4_Dest = 5;
      //***********k4-k5 move*************************is it going on to 6
      if (Foc_move4_Dest == 5 && s6 != 0) {
        if (Foc_k5_position > Foc_k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Foc_k5_position < Foc_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Foc_k5_position == Foc_k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************

        if (Foc_k6_position > Foc_k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Foc_k6_position < Foc_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Foc_k6_position == Foc_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Foc_move4_Dest = 6;
        }
      }
      break;
  }
  //*********************Foc motor Move5 plan***********************************

  switch (Foc_move4_Dest) {                                                       //Find the full move start

    case 5:

      Foc_move4_Dest = 5;

      //***********k4-k5 move*************************is it going on to 6
      if (Foc_move5_Dest == 5 && s6 != 0) {
        if (Foc_k5_position > Foc_k4_position) {                                    //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (Foc_k5_position < Foc_k4_position) {
          k4_k5_Dir = -1;
        }
        if (Foc_k5_position == Foc_k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************is it going on to 6

        if (Foc_k6_position > Foc_k5_position) {                                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (Foc_k6_position < Foc_k5_position) {
          k5_k6_Dir = -1;
        }
        if (Foc_k6_position == Foc_k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          Foc_move5_Dest = 6;
        } else {
          Foc_move5_Dest = 6;
          //Foc_move6_Rst = 1;
        }
      }
      break;
  }
  return;
}
//*****PTZ Control Function*****
void PTZ_Control() {

  //Record State
  if (Rec != lastRecState) {
    digitalWrite(CAM, HIGH);                                                    //Start/stop Record on camera. Switch must be flipped
    delay (200);
    digitalWrite(CAM, LOW);
    if (TLY == 1) {
      if (Tally == 0) {
        Tally = 3;                                                             //Turn off/On the tally light
        leds[0] = CRGB::Red;
        leds[1] = CRGB::Red;
        FastLED.show();
      } else {

        Tally = 0;                                            //Turn off the tally light
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
        FastLED.show();
      }
    }
    lastRecState = Rec;
  }

  //Joystick comands
  //***PAN No limits***
  //    if (abs(Joy_Pan_Speed) > 150 ) {
  //      if (Joy_Pan_Speed > 1) {
  //        Joy_Pan_Speed = Joy_Pan_Speed * 1.5;                                      //Compensation for poor Analoge joystick calibration
  //      }
  //      pan_is_moving = true;
  //      stepper2->setAcceleration(abs(Joy_Pan_Accel));
  //      if (abs(Joy_Pan_Speed) < 750 ) {
  //        stepper2->setSpeedInHz(abs(Joy_Pan_Speed / 4));
  //      } else {
  //        stepper2->setSpeedInHz(abs(Joy_Pan_Speed / 1.5));                           //If the joystick is all the way over step up to full speed
  //      }
  //      if (Joy_Pan_Speed > 1 && stepper2->getCurrentPosition() > (Forward_P_Out -75)) {
  //        stepper2->runForward();
  //      } else {
  //
  //        if (Joy_Pan_Speed < 1 && stepper2->getCurrentPosition() < (Forward_P_In +75)) {
  //          stepper2->runBackward();
  //        }
  //      }
  //    } else {
  //      if (pan_is_moving) {
  //        stepper2->setAcceleration(abs(Joy_Pan_Accel));                             //Breaking acceloration on the stick moving back to 0
  //        stepper2->applySpeedAcceleration();
  //        stepper2->stopMove();
  //        pan_is_moving = false;
  //        PA = ((stepper2->getCurrentPosition()) / 22) / 2;                          //Steps to Deg angle /2 for sending
  //      }
  //    }

  //***TILT No limits***
  //  if (abs(Joy_Tilt_Speed) > 150) {
  //    tilt_is_moving = true;
  //    stepper1->setAcceleration(abs(Joy_Tilt_Accel * 8));
  //    stepper1->setSpeedInHz(abs(Joy_Tilt_Speed / 2 ));
  //    if (Joy_Tilt_Speed > 1) {
  //      stepper1->runForward();
  //    } else {
  //      stepper1->runBackward();
  //    }
  //  } else {
  //    if (tilt_is_moving) {
  //      stepper1->setAcceleration(abs(Joy_Tilt_Accel * 8));                         //Breaking acceloration on the stick moving back to 0
  //      stepper1->applySpeedAcceleration();
  //      stepper1->stopMove();
  //      tilt_is_moving = false;
  //
  //      TA = (stepper1->getCurrentPosition()) / 650;
  //
  //    }
  //  }

  //***Tilt moves new with limits***
  if (abs(Joy_Tilt_Speed) > 150 ) {
    tilt_is_moving = true;
    stepper1->setAcceleration(abs(Joy_Tilt_Accel * 3));
    stepper1->setSpeedInHz(abs(Joy_Tilt_Speed ));

    //    //Reverse Tilt Direction for OBS and vMix
    //    if (T_Rev = 1) {                                                                  //Reverse the Tilt direction for bad joystick direction choices in vMix and OBS!
    //      if (Joy_Tilt_Speed > 0) {
    //        Joy_Tilt_Speed = Joy_Tilt_Speed - (Joy_Tilt_Speed * 2);
    //      } else {
    //        if (Joy_Tilt_Speed < 0) {
    //          Joy_Tilt_Speed = (abs(Joy_Tilt_Speed));
    //        }
    //      }
    //    }

    // Forward Move
    if (Joy_Tilt_Speed > 150 && stepper1->getCurrentPosition() < Forward_T_Out) {
      stepper1->setAcceleration(abs(Joy_Tilt_Accel * 3));
      stepper1->setSpeedInHz(abs(Joy_Tilt_Speed));
      if (stepper1->getCurrentPosition() > (Forward_T_Out - 100 ) && abs(Joy_Tilt_Speed) > 150) {
        if (tilt_is_moving = true) {
          stepper1->moveTo(Forward_T_Out);
        }

      } else {
        if (tilt_is_moving = true && abs(Joy_Tilt_Speed) > 150 ) {
          stepper1->runForward();
        }
      }
      if (stepper1->getCurrentPosition() >= (Forward_T_Out) ) {
        stepper1->forceStopAndNewPosition(Forward_T_Out);
        tilt_is_moving = false;
        TA = (stepper1->getCurrentPosition()) / 650;
      }
    }

    //Backward move
    if (Joy_Tilt_Speed < -150 && stepper1->getCurrentPosition() > Forward_T_In) {
      stepper1->setAcceleration(abs(Joy_Tilt_Accel * 3));                             //If starting fresh Max accel to catch up on turnarround
      stepper1->setSpeedInHz(abs(Joy_Tilt_Speed));
      if (stepper1->getCurrentPosition() < (Forward_T_In + 100 ) && abs(Joy_Tilt_Speed) > 150) {
        if (tilt_is_moving = true) {
          stepper1->moveTo(Forward_T_In);
        }
      } else {
        if (tilt_is_moving = true && abs(Joy_Tilt_Speed) > 150) {
          stepper1->runBackward();
        }
      }
      if (stepper1->getCurrentPosition() <= (Forward_T_In)) {
        stepper1->forceStopAndNewPosition(-3500);
        tilt_is_moving = false;
        TA = (stepper1->getCurrentPosition()) / 650;
      }
    }
  } else {                                                                          //No command but stepper is still moving
    if (tilt_is_moving) {
      stepper1->setAcceleration(1800);                                              //Breaking acceloration on the stick moving back to 0
      stepper1->applySpeedAcceleration();
      stepper1->stopMove();
      //Joy_Tilt_Speed=0;
      tilt_is_moving = false;
      Tilt_Stop = 0;
      TA = (stepper1->getCurrentPosition()) / 650;
    }
  }


  //***Pan moves new with limits***
  if (abs(Joy_Pan_Speed) > 150 ) {
    pan_is_moving = true;
    stepper2->setAcceleration(abs(Joy_Pan_Accel));


    // Forward Move
    if (Joy_Pan_Speed > 150 && stepper2->getCurrentPosition() < Forward_P_Out) {
      stepper2->setAcceleration(abs(Joy_Pan_Accel));

      if (Joy_Pan_Speed < 300) {
        stepper2->setSpeedInHz(abs(Joy_Pan_Speed) / 3);
      }
      if (Joy_Pan_Speed > 300 && Joy_Pan_Speed < 400 ) {
        stepper2->setSpeedInHz(abs(Joy_Pan_Speed) / 2);
      }
      if (Joy_Pan_Speed > 400 && Joy_Pan_Speed < 500) {
        stepper2->setSpeedInHz(abs(Joy_Pan_Speed) / 1.5);
      }
      if (Joy_Pan_Speed > 500 ) {
        stepper2->setSpeedInHz(abs(Joy_Pan_Speed));
      }
      if (stepper2->getCurrentPosition() > (Forward_P_Out - 100 ) && abs(Joy_Pan_Speed) > 150) {
        if (pan_is_moving = true) {
          stepper2->moveTo(Forward_P_Out);
        }

      } else {
        if (pan_is_moving = true && abs(Joy_Pan_Speed) > 150 ) {
          stepper2->runForward();
        }
      }
      if (stepper2->getCurrentPosition() >= (Forward_P_Out) ) {
        stepper2->forceStopAndNewPosition(Forward_P_Out);
        pan_is_moving = false;
        PA = ((stepper2->getCurrentPosition()) / 22) / 2;
      }
    }

    //Backward move
    if (Joy_Pan_Speed < -150 && stepper2->getCurrentPosition() > Forward_P_In) {
      stepper2->setAcceleration(abs(Joy_Pan_Accel ));                           
      
      if (Joy_Pan_Speed > -300) {
        stepper2->setSpeedInHz(abs(Joy_Pan_Speed) / 3);
      }
      if (Joy_Pan_Speed < -300 && Joy_Pan_Speed > -400 ) {
        stepper2->setSpeedInHz(abs(Joy_Pan_Speed) / 2);
      }
      if (Joy_Pan_Speed < -400 && Joy_Pan_Speed > -500) {
        stepper2->setSpeedInHz(abs(Joy_Pan_Speed) / 1.5);
      }
      if (Joy_Pan_Speed < -500 ) {
        stepper2->setSpeedInHz(abs(Joy_Pan_Speed));
      }


      if (stepper2->getCurrentPosition() < (Forward_P_In + 100 ) && abs(Joy_Pan_Speed) > 150) {
        if (pan_is_moving = true) {
          stepper2->moveTo(Forward_P_In);
        }
      } else {
        if (pan_is_moving = true && abs(Joy_Pan_Speed) > 150) {
          stepper2->runBackward();
        }
      }
      if (stepper2->getCurrentPosition() <= (Forward_P_In)) {
        stepper2->forceStopAndNewPosition(-3500);
        pan_is_moving = false;
        PA = ((stepper2->getCurrentPosition()) / 22) / 2;
      }
    }
  } else {                                                                          //No command but stepper is still moving
    if (pan_is_moving) {
      stepper2->setAcceleration(1800);                                              //Breaking acceloration on the stick moving back to 0
      stepper2->applySpeedAcceleration();
      //stepper2->forceStopAndNewPosition(stepper2->getCurrentPosition());
      stepper2->stopMove();
      //Joy_Pan_Speed=0;
      pan_is_moving = false;
      Pan_Stop = 0;
      PA = ((stepper2->getCurrentPosition()) / 22) / 2;
    }
  }


  //***FOCUS +moves***
  if (cam_F_Out > cam_F_In) {                                                          //Check the diretion the Focus motor is working in + or -
    if (abs(Joy_Focus_Speed) > 1000 && Focus_Stop == 0) {
      focus_is_moving = true;
      stepper3->setAcceleration(abs(Joy_Focus_Accel * 4));
      stepper3->setSpeedInHz(abs(Joy_Focus_Speed));

      // Forward Move
      if (Joy_Focus_Speed > 1000 && stepper3->getCurrentPosition() < cam_F_Out) {
        stepper3->setAcceleration(abs(Joy_Focus_Accel * 4));
        stepper3->setSpeedInHz(abs(Joy_Focus_Speed));
        if (stepper3->getCurrentPosition() > (cam_F_Out - 100 ) && abs(Joy_Focus_Speed) > 1000) {
          if (focus_is_moving = true) {
            stepper3->moveTo(cam_F_Out);
          }

        } else {
          if (focus_is_moving = true) {
            stepper3->runForward();
          }
        }
        if (stepper3->getCurrentPosition() > (cam_F_Out) ) {
          stepper3->forceStopAndNewPosition(cam_F_Out);
          focus_is_moving = false;
          FA = stepper3->getCurrentPosition();
        }
      }

      //Backward move
      if (Joy_Focus_Speed < 1000 && stepper3->getCurrentPosition() > cam_F_In) {
        stepper3->setAcceleration(abs(Joy_Focus_Accel * 4));                             //If starting fresh Max accel to catch up on turnarround
        stepper3->setSpeedInHz(abs(Joy_Focus_Speed));
        if (stepper3->getCurrentPosition() < (cam_F_In + 100 ) && abs(Joy_Focus_Speed) > 1000) {
          if (focus_is_moving = true) {
            stepper3->moveTo(cam_F_In);
          }
        } else {
          if (focus_is_moving = true) {
            stepper3->runBackward();
          }
        }
        if (stepper3->getCurrentPosition() <= (cam_F_In)) {
          stepper3->forceStopAndNewPosition(cam_F_In);
          focus_is_moving = false;
          FA = stepper3->getCurrentPosition();
        }
      }
    } else {                                                                          //No command but stepper is still moving
      if (focus_is_moving) {
        stepper3->setAcceleration(12000);                                              //Breaking acceloration on the stick moving back to 0
        stepper3->applySpeedAcceleration();
        stepper3->forceStopAndNewPosition(stepper3->getCurrentPosition());
        //stepper3->stopMove();
        focus_is_moving = false;
        Focus_Stop = 0;
        FA = stepper3->getCurrentPosition();
      }
    }
  }
  //***FOCUS -moves***

  if (cam_F_Out < cam_F_In) {                                                          //Check the diretion the Focus motor is working in + or -


    if (abs(Joy_Focus_Speed) > 1000 && Focus_Stop == 0) {
      focus_is_moving = true;
      stepper3->setAcceleration(abs(Joy_Focus_Accel * 4));                             //If starting fresh Max accel to catch up on turnarround
      stepper3->setSpeedInHz(abs(Joy_Focus_Speed ));

      // Forward Move
      if (Joy_Focus_Speed > 1000 && stepper3->getCurrentPosition() > cam_F_Out) {
        stepper3->setAcceleration(abs(Joy_Focus_Accel * 4));                             //If starting fresh Max accel to catch up on turnarround
        stepper3->setSpeedInHz(abs(Joy_Focus_Speed));
        if (stepper3->getCurrentPosition() < (cam_F_Out + 100 ) && abs(Joy_Focus_Speed) > 1000) {  //Close to end move to end
          if (focus_is_moving = true) {
            stepper3->moveTo(cam_F_Out);
          }

        } else {
          if (focus_is_moving = true) {
            stepper3->runBackward();
          }
        }
        if (stepper3->getCurrentPosition() <= (cam_F_Out) ) {
          stepper3->forceStopAndNewPosition(cam_F_Out);
          focus_is_moving = false;
          FA = stepper3->getCurrentPosition();
        }
      }

      //Backward move
      if (Joy_Focus_Speed < 1000 && stepper3->getCurrentPosition() < cam_F_In) {
        Serial.print("Backword move");
        stepper3->setAcceleration(abs(Joy_Focus_Accel * 4));                                      //If starting fresh Max accel to catch up on turnarround
        stepper3->setSpeedInHz(abs(Joy_Focus_Speed));
        if (stepper3->getCurrentPosition() > (cam_F_In - 100 ) && abs(Joy_Focus_Speed) > 1000) {  // If close to end
          if (focus_is_moving = true) {
            stepper3->moveTo(cam_F_In);
          }
        } else {
          if (focus_is_moving = true) {
            stepper3->runForward();
          }
        }
        if (stepper3->getCurrentPosition() >= (cam_F_In)) {
          stepper3->forceStopAndNewPosition(cam_F_In);
          focus_is_moving = false;
          FA = stepper3->getCurrentPosition();
        }
      }
    } else {                                                                          //No command but stepper is still moving
      if (focus_is_moving) {
        stepper3->setAcceleration(12000);                                              //Breaking acceloration on the stick moving back to 0
        stepper3->applySpeedAcceleration();
        stepper3->forceStopAndNewPosition(stepper3->getCurrentPosition());
        //stepper3->stopMove();
        focus_is_moving = false;
        Focus_Stop = 0;
        FA = stepper3->getCurrentPosition();
      }
    }
  }









  //***ZOOM +moves***
  if (cam_Z_Out > cam_Z_In) {                                                          //Check the diretion the Focus motor is working in + or -
    if (abs(Joy_Zoom_Speed) > 1000 && Zoom_Stop == 0) {
      zoom_is_moving = true;
      stepper4->setAcceleration(abs(Joy_Zoom_Accel * 4));                             //If starting fresh Max accel to catch up on turnarround
      stepper4->setSpeedInHz(abs(Joy_Zoom_Speed));

      // Forward Move
      if (Joy_Zoom_Speed > 1000 && stepper4->getCurrentPosition() < cam_Z_Out) {
        stepper4->setAcceleration(abs(Joy_Zoom_Accel * 4));                             //If starting fresh Max accel to catch up on turnarround
        stepper4->setSpeedInHz(abs(Joy_Zoom_Speed));
        if (stepper4->getCurrentPosition() > (cam_Z_Out - 100 ) && abs(Joy_Zoom_Speed) > 1000) {
          if (zoom_is_moving = true) {
            stepper4->moveTo(cam_Z_Out);
          }

        } else {
          if (zoom_is_moving = true) {
            stepper4->runForward();
          }
        }
        if (stepper4->getCurrentPosition() > (cam_Z_Out) ) {
          stepper4->forceStopAndNewPosition(cam_Z_Out);
          zoom_is_moving = false;
          ZA = stepper4->getCurrentPosition();
        }
      }

      //Backward move
      if (Joy_Zoom_Speed < 1000 && stepper4->getCurrentPosition() > cam_Z_In) {
        stepper4->setAcceleration(abs(Joy_Zoom_Accel * 4));                             //If starting fresh Max accel to catch up on turnarround
        stepper4->setSpeedInHz(abs(Joy_Zoom_Speed));
        if (stepper4->getCurrentPosition() < (cam_Z_In + 100 ) && abs(Joy_Zoom_Speed) > 1000) {
          if (zoom_is_moving = true) {
            stepper4->moveTo(cam_Z_In);
          }
        } else {
          if (zoom_is_moving = true) {
            stepper4->runBackward();
          }
        }
        if (stepper4->getCurrentPosition() <= (cam_Z_In)) {
          stepper4->forceStopAndNewPosition(cam_Z_In);
          zoom_is_moving = false;
          ZA = stepper4->getCurrentPosition();
        }
      }
    } else {                                                                          //No command but stepper is still moving
      if (zoom_is_moving) {
        stepper4->setAcceleration(12000);                                              //Breaking acceloration on the stick moving back to 0
        stepper4->applySpeedAcceleration();
        stepper4->forceStopAndNewPosition(stepper4->getCurrentPosition());
        //stepper4->stopMove();
        zoom_is_moving = false;
        Zoom_Stop = 0;
        ZA = stepper4->getCurrentPosition();
      }
    }
  }

  //***ZOOM -moves***

  if (cam_Z_Out < cam_Z_In) {                                                          //Check the diretion the Zoom motor is working in + or -


    if (abs(Joy_Zoom_Speed) > 1000 && Zoom_Stop == 0) {
      zoom_is_moving = true;
      stepper4->setAcceleration(abs(Joy_Zoom_Accel * 4));                             //If starting fresh Max accel to catch up on turnarround
      stepper4->setSpeedInHz(abs(Joy_Zoom_Speed));

      // Forward Move
      if (Joy_Zoom_Speed > 1000 && stepper4->getCurrentPosition() > cam_Z_Out) {
        stepper4->setAcceleration(abs(Joy_Zoom_Accel * 4));                             //If starting fresh Max accel to catch up on turnarround
        stepper4->setSpeedInHz(abs(Joy_Zoom_Speed));
        if (stepper4->getCurrentPosition() < (cam_Z_Out + 100 ) && abs(Joy_Zoom_Speed) > 1000) {  //Close to end move to end
          if (zoom_is_moving = true) {
            stepper4->moveTo(cam_Z_Out);
          }

        } else {
          if (zoom_is_moving = true) {
            stepper4->runBackward();
          }
        }
        if (stepper4->getCurrentPosition() <= (cam_Z_Out) ) {
          stepper4->forceStopAndNewPosition(cam_Z_Out);
          zoom_is_moving = false;
          ZA = stepper4->getCurrentPosition();
        }
      }

      //Backward move
      if (Joy_Zoom_Speed < 1000 && stepper4->getCurrentPosition() < cam_Z_In) {
        Serial.print("Backword move");
        stepper4->setAcceleration(abs(Joy_Zoom_Accel * 4));                                      //If starting fresh Max accel to catch up on turnarround
        stepper4->setSpeedInHz(abs(Joy_Zoom_Speed));
        if (stepper4->getCurrentPosition() > (cam_Z_In - 100 ) && abs(Joy_Zoom_Speed) > 1000) {  // If close to end
          if (zoom_is_moving = true) {
            stepper4->moveTo(cam_Z_In);
          }
        } else {
          if (zoom_is_moving = true) {
            stepper4->runForward();
          }
        }
        if (stepper4->getCurrentPosition() >= (cam_Z_In)) {
          stepper4->forceStopAndNewPosition(cam_Z_In);
          zoom_is_moving = false;
          ZA = stepper4->getCurrentPosition();
        }
      }
    } else {                                                                          //No command but stepper is still moving
      if (zoom_is_moving) {
        stepper4->setAcceleration(12000);                                              //Breaking acceloration on the stick moving back to 0
        stepper4->applySpeedAcceleration();
        stepper4->forceStopAndNewPosition(stepper4->getCurrentPosition());
        //stepper4->stopMove();
        zoom_is_moving = false;
        Zoom_Stop = 0;
        ZA = stepper3->getCurrentPosition();
      }
    }
  }





}



void SetID() {
  SysMemory.begin("ID", false);
  SysMemory.putUInt("PTZ_ID", PTZ_ID_Nex);
  SysMemory.end();
  PTZ_ID = PTZ_ID_Nex;
  //BatCheck();    // update OLED
}

void Load_SysMemory() {
  //SysMemory Recover last setup
  SysMemory.begin("IPvalues", false);                              //Recover the last IP adress settings
  IP1 = SysMemory.getUInt("IP1", 0);
  IP2 = SysMemory.getUInt("IP2", 0);
  IP3 = SysMemory.getUInt("IP3", 0);
  IP4 = SysMemory.getUInt("IP4", 0);
  IPGW = SysMemory.getUInt("IPGW", 0);
  UDP = SysMemory.getUInt("UDP", 0);
  SysMemory.end();
  if (IP1 == 0) {                                                //If there is no valid IP in memorry set a default
    IP1 = 192;
    IP2 = 168;
    IP3 = 100;
    IP4 = 100;
    IPGW = 100;
    UDP = 1259;
  }
  SysMemory.begin("In_positions", false);                       //Recover the last In_Positions from memory before shutdown
  TLTin_position = SysMemory.getUInt("TLTin_position", 0);
  PANin_position = SysMemory.getUInt("PANin_position", 0);
  FOCin_position = SysMemory.getUInt("FOCin_position", 0);
  ZMin_position = SysMemory.getUInt("ZMin_position", 0);
  SysMemory.end();

  SysMemory.begin("Out_positions", false);                      //Recover the last In_Positions from memory before shutdown
  TLTout_position = SysMemory.getUInt("TLTout_position", 0);
  PANout_position = SysMemory.getUInt("PANout_position", 0);
  FOCout_position = SysMemory.getUInt("FOCout_position", 0);
  ZMout_position = SysMemory.getUInt("ZMout_position", 0);
  SysMemory.end();

  SysMemory.begin("FocLmts", false);                           //Recover Focus Limits
  cam_F_In = SysMemory.getUInt("cam_F_In", 0);
  cam_F_Out = SysMemory.getUInt("cam_F_Out", 0);
  SysMemory.end();
  if ( cam_F_Out == 0) {                                         //Default move
    cam_F_Out = 2000;
  }

  SysMemory.begin("ZoomLmts", false);                           //Recover Zoom Limits
  cam_Z_In = SysMemory.getUInt("cam_Z_In", 0);
  cam_Z_Out = SysMemory.getUInt("cam_Z_Out", 0);
  SysMemory.end();
  if ( cam_Z_Out == 0) {                                         //Default move
    cam_Z_Out = 2000;
  }

  SysMemory.begin("TiltLmts", false);                           //Recover Tilt Limits
  Forward_T_In = SysMemory.getUInt("Forward_T_In", 0);
  Forward_T_Out = SysMemory.getUInt("Forward_T_Out", 0);
  SysMemory.end();
  if ( Forward_T_In == 0) {                                         //Default move
    Forward_T_In = -3500;
  }
  if ( Forward_T_Out == 0) {                                         //Default move
    Forward_T_Out = 7000;
  }

  SysMemory.begin("P1pos", false);                        //Recover PTZ Cam poses
  P1_T = SysMemory.getUInt("P1_T", 10);
  P1_P = SysMemory.getUInt("P1_P", 10);
  P1_F = SysMemory.getUInt("P1_F", 10);
  P1_Z = SysMemory.getUInt("P1_Z", 10);
  SysMemory.end();

  SysMemory.begin("P2pos", false);
  P2_T = SysMemory.getUInt("P2_T", 10);
  P2_P = SysMemory.getUInt("P2_P", 10);
  P2_F = SysMemory.getUInt("P2_F", 10);
  P2_Z = SysMemory.getUInt("P2_Z", 10);
  SysMemory.end();

  SysMemory.begin("P3pos", false);
  P3_T = SysMemory.getUInt("P3_T", 10);
  P3_P = SysMemory.getUInt("P3_P", 10);
  P3_F = SysMemory.getUInt("P3_F", 10);
  P3_Z = SysMemory.getUInt("P3_Z", 10);
  SysMemory.end();

  SysMemory.begin("P4pos", false);
  P4_T = SysMemory.getUInt("P4_T", 10);
  P4_P = SysMemory.getUInt("P4_P", 10);
  P4_F = SysMemory.getUInt("P4_F", 10);
  P4_Z = SysMemory.getUInt("P4_Z", 10);
  SysMemory.end();

  SysMemory.begin("P5pos", false);
  P5_T = SysMemory.getUInt("P5_T", 10);
  P5_P = SysMemory.getUInt("P5_P", 10);
  P5_F = SysMemory.getUInt("P5_F", 10);
  P5_Z = SysMemory.getUInt("P5_Z", 10);
  SysMemory.end();

  SysMemory.begin("P6pos", false);
  P6_T = SysMemory.getUInt("P6_T", 10);
  P6_P = SysMemory.getUInt("P6_P", 10);
  P6_F = SysMemory.getUInt("P6_F", 10);
  P6_Z = SysMemory.getUInt("P6_Z", 10);
  SysMemory.end();

  SysMemory.begin("P7pos", false);
  P7_T = SysMemory.getUInt("P7_T", 10);
  P7_P = SysMemory.getUInt("P7_P", 10);
  P7_F = SysMemory.getUInt("P7_F", 10);
  P7_Z = SysMemory.getUInt("P7_Z", 10);
  SysMemory.end();

  SysMemory.begin("P8pos", false);
  P8_T = SysMemory.getUInt("P8_T", 10);
  P8_P = SysMemory.getUInt("P8_P", 10);
  P8_F = SysMemory.getUInt("P8_F", 10);
  P8_Z = SysMemory.getUInt("P8_Z", 10);
  SysMemory.end();

  SysMemory.begin("P9pos", false);
  P9_T = SysMemory.getUInt("P9_T", 10);
  P9_P = SysMemory.getUInt("P9_P", 10);
  P9_F = SysMemory.getUInt("P9_F", 10);
  P9_Z = SysMemory.getUInt("P9_Z", 10);
  SysMemory.end();

  SysMemory.begin("P10pos", false);
  P10_T = SysMemory.getUInt("P10_T", 10);
  P10_P = SysMemory.getUInt("P10_P", 10);
  P10_F = SysMemory.getUInt("P10_F", 10);
  P10_Z = SysMemory.getUInt("P10_Z", 10);
  SysMemory.end();

  SysMemory.begin("P11pos", false);
  P11_T = SysMemory.getUInt("P11_T", 10);
  P11_P = SysMemory.getUInt("P11_P", 10);
  P11_F = SysMemory.getUInt("P11_F", 10);
  P11_Z = SysMemory.getUInt("P11_Z", 10);
  SysMemory.end();

  SysMemory.begin("P12pos", false);
  P12_T = SysMemory.getUInt("P12_T", 10);
  P12_P = SysMemory.getUInt("P12_P", 10);
  P12_F = SysMemory.getUInt("P12_F", 10);
  P12_Z = SysMemory.getUInt("P12_Z", 10);
  SysMemory.end();

  SysMemory.begin("P13pos", false);
  P13_T = SysMemory.getUInt("P13_T", 10);
  P13_P = SysMemory.getUInt("P13_P", 10);
  P13_F = SysMemory.getUInt("P13_F", 10);
  P13_Z = SysMemory.getUInt("P13_Z", 10);
  SysMemory.end();

  SysMemory.begin("P14pos", false);
  P14_T = SysMemory.getUInt("P14_T", 10);
  P14_P = SysMemory.getUInt("P14_P", 10);
  P14_F = SysMemory.getUInt("P14_F", 10);
  P14_Z = SysMemory.getUInt("P14_Z", 10);
  SysMemory.end();

  SysMemory.begin("P15pos", false);
  P15_T = SysMemory.getUInt("P15_T", 10);
  P15_P = SysMemory.getUInt("P15_P", 10);
  P15_F = SysMemory.getUInt("P15_F", 10);
  P15_Z = SysMemory.getUInt("P15_Z", 10);
  SysMemory.end();

  SysMemory.begin("P16pos", false);
  P16_T = SysMemory.getUInt("P16_T", 10);
  P16_P = SysMemory.getUInt("P16_P", 10);
  P16_F = SysMemory.getUInt("P16_F", 10);
  P16_Z = SysMemory.getUInt("P16_Z", 10);
  SysMemory.end();

  SysMemory.begin("k1_pos", false);                               //Recover sequencer Key k1
  Tlt_k1_position = SysMemory.getUInt("Tlt_k1_pos", 0);
  Pan_k1_position = SysMemory.getUInt("Pan_k1_pos", 0);
  Foc_k1_position = SysMemory.getUInt("Foc_k1_pos", 0);
  //Zoom_k1_position = SysMemory.getUInt("Tlt_k1_pos", 0);
  SysMemory.end();

  SysMemory.begin("k2_pos", false);                               //Recover sequencer Key k2
  Tlt_k2_position = SysMemory.getUInt("Tlt_k2_pos", 0);
  Pan_k2_position = SysMemory.getUInt("Pan_k2_pos", 0);
  Foc_k2_position = SysMemory.getUInt("Foc_k2_pos", 0);
  //Zoom_k2_position = SysMemory.getUInt("Tlt_k2_pos", 0);
  SysMemory.end();

  SysMemory.begin("k3_pos", false);                               //Recover sequencer Key k3
  Tlt_k3_position = SysMemory.getUInt("Tlt_k3_pos", 0);
  Pan_k3_position = SysMemory.getUInt("Pan_k3_pos", 0);
  Foc_k3_position = SysMemory.getUInt("Foc_k3_pos", 0);
  //Zoom_k3_position = SysMemory.getUInt("Tlt_k3_pos", 0);
  SysMemory.end();

  SysMemory.begin("k4_pos", false);                                 //Recover sequencer Key k4
  Tlt_k4_position = SysMemory.getUInt("Tlt_k4_pos", 0);
  Pan_k4_position = SysMemory.getUInt("Pan_k4_pos", 0);
  Foc_k4_position = SysMemory.getUInt("Foc_k4_pos", 0);
  //Zoom_k4_position = SysMemory.getUInt("Tlt_k4_pos", 0);
  SysMemory.end();

  SysMemory.begin("k5_pos", false);                               //Recover sequencer Key k5
  Tlt_k5_position = SysMemory.getUInt("Tlt_k5_pos", 0);
  Pan_k5_position = SysMemory.getUInt("Pan_k5_pos", 0);
  Foc_k5_position = SysMemory.getUInt("Foc_k5_pos", 0);
  //Zoom_k5_position = SysMemory.getUInt("Tlt_k5_pos", 0);
  SysMemory.end();

  SysMemory.begin("k6_pos", false);                                 //Recover sequencer Key k6
  Tlt_k6_position = SysMemory.getUInt("Tlt_k6_pos", 0);
  Pan_k6_position = SysMemory.getUInt("Pan_k6_pos", 0);
  Foc_k6_position = SysMemory.getUInt("Foc_k6_pos", 0);
  //Zoom_k6_position = SysMemory.getUInt("Tlt_k6_pos", 0);
  SysMemory.end();

}







void ClearKeys() {                                                //Function to return system memmory of keys back to 0
  SysMemory.begin("In_positions", false);
  TLTin_position = 0;
  PANin_position = 0;
  FOCin_position = 0;
  ZMin_position = 0;
  SysMemory.putUInt("TLTin_position", TLTin_position);
  SysMemory.putUInt("PANin_position", PANin_position);
  SysMemory.putUInt("FOCin_position", FOCin_position);
  SysMemory.putUInt("ZMin_position", ZMin_position);
  SysMemory.end();

  SysMemory.begin("Out_positions", false);
  TLTout_position = 0;
  PANout_position = 0;
  FOCout_position = 0;
  ZMout_position = 0;
  SysMemory.putUInt("TLTout_position", TLTout_position);
  SysMemory.putUInt("PANout_position", PANout_position);
  SysMemory.putUInt("FOCout_position", FOCout_position);
  SysMemory.putUInt("ZMout_position", ZMout_position);
  SysMemory.end();

  SysMemory.begin("k1_pos", false);
  Tlt_k1_position = 0;
  Pan_k1_position = 0;
  Foc_k1_position = 0;
  SysMemory.putUInt("Tlt_k1_pos", Tlt_k1_position);
  SysMemory.putUInt("Pan_k1_pos", Pan_k1_position);
  SysMemory.putUInt("Foc_k1_pos", Foc_k1_position);
  SysMemory.end();

  SysMemory.begin("k2_pos", false);
  Tlt_k2_position = 0;
  Pan_k2_position = 0;
  Foc_k2_position = 0;
  SysMemory.putUInt("Tlt_k2_pos", Tlt_k2_position);
  SysMemory.putUInt("Pan_k2_pos", Pan_k2_position);
  SysMemory.putUInt("Foc_k2_pos", Foc_k2_position);
  SysMemory.end();

  SysMemory.begin("k3_pos", false);
  Tlt_k3_position = 0;
  Pan_k3_position = 0;
  Foc_k3_position = 0;
  SysMemory.putUInt("Tlt_k3_pos", Tlt_k3_position);
  SysMemory.putUInt("Pan_k3_pos", Pan_k3_position);
  SysMemory.putUInt("Foc_k3_pos", Foc_k3_position);
  SysMemory.end();

  SysMemory.begin("k4_pos", false);
  Tlt_k4_position = 0;
  Pan_k4_position = 0;
  Foc_k4_position = 0;
  SysMemory.putUInt("Tlt_k4_pos", Tlt_k4_position);
  SysMemory.putUInt("Pan_k4_pos", Pan_k4_position);
  SysMemory.putUInt("Foc_k4_pos", Foc_k4_position);
  SysMemory.end();

  SysMemory.begin("K5_pos", false);
  Tlt_k5_position = 0;
  Pan_k5_position = 0;
  Foc_k5_position = 0;
  SysMemory.putUInt("Tlt_k5_pos", Tlt_k5_position);
  SysMemory.putUInt("Pan_k5_pos", Pan_k5_position);
  SysMemory.putUInt("Foc_k5_pos", Foc_k5_position);
  SysMemory.end();

  SysMemory.begin("k6_pos", false);
  Tlt_k6_position = 0;
  Pan_k6_position = 0;
  Foc_k6_position = 0;
  SysMemory.putUInt("Tlt_k6_pos", Tlt_k6_position);
  SysMemory.putUInt("Pan_k6_pos", Pan_k6_position);
  SysMemory.putUInt("Foc_k6_pos", Foc_k6_position);
  SysMemory.end();




  SysMemory.begin("P1_pos", false);
  P1_T = 10;
  P1_P = 10;
  P1_F = 10;
  P1_Z = 10;
  SysMemory.putUInt("P1_T", P1_T);
  SysMemory.putUInt("P1_P", P1_P);
  SysMemory.putUInt("P1_F", P1_F);
  SysMemory.putUInt("P1_Z", P1_Z);
  SysMemory.end();

  SysMemory.begin("P2_pos", false);
  P2_T = 10;
  P2_P = 10;
  P2_F = 10;
  P2_Z = 10;
  SysMemory.putUInt("P2_T", P2_T);
  SysMemory.putUInt("P2_P", P2_P);
  SysMemory.putUInt("P2_F", P2_F);
  SysMemory.putUInt("P2_Z", P2_Z);
  SysMemory.end();

  SysMemory.begin("P3_pos", false);
  P3_T = 10;
  P3_P = 10;
  P3_F = 10;
  P3_Z = 10;
  SysMemory.putUInt("P3_T", P3_T);
  SysMemory.putUInt("P3_P", P3_P);
  SysMemory.putUInt("P3_F", P3_F);
  SysMemory.putUInt("P3_Z", P3_Z);
  SysMemory.end();

  SysMemory.begin("P4_pos", false);
  P4_T = 10;
  P4_P = 10;
  P4_F = 10;
  P4_Z = 10;
  SysMemory.putUInt("P4_T", P4_T);
  SysMemory.putUInt("P4_P", P4_P);
  SysMemory.putUInt("P4_F", P4_F);
  SysMemory.putUInt("P4_Z", P4_Z);
  SysMemory.end();

  SysMemory.begin("P5_pos", false);
  P5_T = 10;
  P5_P = 10;
  P5_F = 10;
  P5_Z = 10;
  SysMemory.putUInt("P5_T", P5_T);
  SysMemory.putUInt("P5_P", P5_P);
  SysMemory.putUInt("P5_F", P5_F);
  SysMemory.putUInt("P5_Z", P5_Z);
  SysMemory.end();

  SysMemory.begin("P6_pos", false);
  P6_T = 10;
  P6_P = 10;
  P6_F = 10;
  P6_Z = 10;
  SysMemory.putUInt("P6_T", P6_T);
  SysMemory.putUInt("P6_P", P6_P);
  SysMemory.putUInt("P6_F", P6_F);
  SysMemory.putUInt("P6_Z", P6_Z);
  SysMemory.end();

  clrK = 0;
  T_ResetEncoder = 1;
  P_ResetEncoder = 1;
  F_ResetEncoder = 1;
  Z_ResetEncoder = 1;


}
//void ClearKeys() {                                                //Function to return system memmory of keys back to 0
//  clrK = 0;
//  nvs_flash_erase(); // erase the NVS partition and...
//  nvs_flash_init(); // initialize the NVS partition.
//
//  PTZ_ID_Nex = PTZ_ID;
//  SetID();
//  ESP.restart();
//}

void Percent_100(void) {
  TCA9548A(4);                          //OLED on bus 4
  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(35, 16);            // Start at top-left corner
  display.cp437(true);                  // Use full 256 char 'Code Page 437' font
  if (PTZ_ID > 3) {
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
  TCA9548A(4);                          //OLED on bus 3
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 16);
  display.cp437(true);
  if (PTZ_ID > 3) {
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
  TCA9548A(4);                          //OLED on bus 3
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 16);
  display.cp437(true);
  if (PTZ_ID == 4) {
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
  TCA9548A(4);                          //OLED on bus 3
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 16);
  display.cp437(true);
  if (PTZ_ID == 4) {
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
  TCA9548A(4);                          //OLED on bus 3
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 16);
  display.cp437(true);
  if (PTZ_ID == 4) {
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
  if (BatV > 7.5) {
    Percent_100();
  } else {
    if (BatV > 7.4) {
      Percent_75();
    } else {
      if (BatV > 7.3) {
        Percent_50();
      } else {
        if (BatV > 7.2) {
          Percent_25();
        } else {
          Percent_0();
        }
      }
    }
  }
}

void Oledmessage() {
  TCA9548A(4);                          //OLED on bus 3
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 16);
  display.cp437(true);
  display.printf("Sld:%d\n", Sld);
  display.display();
  delay(2000);
}

//**********************************Parse VISCA commands**************



void  listenForVisca() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (UARTport.available() > 0 && newData == false) {
    rc = UARTport.read();
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
  showNewData();
}

void showNewData() {

  char * StrTokIndx;
  if (newData == true) {
    //    T_Rev = 1;                                                              //If were using VISCA reverse the Tilt directions for bad joysticks
    //String Speed = (receivedChars);
    ViscaComand = atoi(receivedChars);
    //Serial.println(receivedChars);

    if (ViscaComand  == 14) {     //Tally light command
      StrTokIndx = strtok(receivedChars, ",");
      ViscaComand = atoi(StrTokIndx);
      StrTokIndx = strtok(NULL, ",");
      Tally = atoi(StrTokIndx) + 1;

    }


    if (ViscaComand  == 40 || ViscaComand  == 41 || ViscaComand  == 42) {     //If the received command has more than one componant part
      StrTokIndx = strtok(receivedChars, ",");
      ViscaComand = atoi(StrTokIndx);
      StrTokIndx = strtok(NULL, ",");
      PTZ_Pose = atoi(StrTokIndx) + 1;

    }
    if (ViscaComand  == 13 || ViscaComand  == 23 || ViscaComand  == 31 || ViscaComand  == 32 || ViscaComand  == 11 || ViscaComand  == 21 || ViscaComand  == 12 || ViscaComand  == 22 ) {  //If the received command has more than one componant part
      StrTokIndx = strtok(receivedChars, ",");
      ViscaComand = atoi(StrTokIndx);
      StrTokIndx = strtok(NULL, ",");
      VPanSpeed = atoi(StrTokIndx);
      StrTokIndx = strtok(NULL, ",");
      VTiltSpeed = atoi(StrTokIndx);
    }
    if (ViscaComand  == 9 ) {                             //A VISCA Absolute move request
      Serial.println("vMix absolute move request received");
      StrTokIndx = strtok(receivedChars, ",");
      ViscaComand = atoi(StrTokIndx);
      StrTokIndx = strtok(NULL, ",");
      VPanSpeed = atoi(StrTokIndx);
      StrTokIndx = strtok(NULL, ",");
      VTiltSpeed = atoi(StrTokIndx);
      StrTokIndx = strtok(NULL, ",");
      P_position = atoi(StrTokIndx);
      StrTokIndx = strtok(NULL, ",");
      T_position = atoi(StrTokIndx);
      StrTokIndx = strtok(NULL, ",");
      F_position = atoi(StrTokIndx);
      StrTokIndx = strtok(NULL, ",");
      Z_position = atoi(StrTokIndx);
      if (cam_F_In < cam_F_Out) { //Focus is negativ
        F_position = F_position - (F_position * 2);
      }
      if (cam_Z_In < cam_Z_Out) { //Zoom is negativ
        Z_position = Z_position - (Z_position * 2);
      }
    }
    parseVISCA();
  }
  newData = false;
}
void parseVISCA() {
  PTZ_Cam = PTZ_ID;                                      //While reading VISA over IP force the system to egnore ID setup
  char buffer[50];
  switch (ViscaComand)  {
    case 64: Serial.print(", PT Home ");
      Home(); break;

    //IP comands

    case 14: Serial.print("Tally Light Comand");      //Hardware required to control an LED required here This might also be used to trigger on camera recording
      switch (Tally) {
        case 0: Serial.println("\nTally 0"); break; //Flashing (in prevue)
        case 1: Serial.println("\nTally 1"); break; //Flashing (in prevue)
        case 2: Serial.println("\nTally 2"); break; //Light Always on (Live)
        case 3: Serial.println("\nTally 3"); break; //Normal or (off inactive)
        case 4: Serial.println("\nTally 4"); break; //Normal or (off inactive)
      }

    case 665: Serial.print("IP1 request");
      sprintf(buffer, "%d\n", IP1 );
      UARTport.print(buffer);
      UARTport.flush(); break;
    case 666: Serial.print("IP2 request");
      sprintf(buffer, "%d\n", IP2 );
      UARTport.print(buffer);
      UARTport.flush(); break;

    case 667: Serial.print("IP3 request");
      sprintf(buffer, "%d\n", IP3 );
      UARTport.print(buffer);
      UARTport.flush(); break;

    case 668: Serial.print("IP4 request");
      sprintf(buffer, "%d\n", IP4 );
      UARTport.print(buffer);
      UARTport.flush(); break;


    case 669: Serial.print("UDPport request");
      UDP = 1259;
      sprintf(buffer, "%d\n", UDP );
      UARTport.print(buffer);
      UARTport.flush(); break;

    case 670: Serial.print("UDPport request");

      sprintf(buffer, "%d\n", IPGW );
      UARTport.print(buffer);
      UARTport.flush(); break;


    //Position reports
    case 78: Serial.print(", Pan Position? ");        //Return Pan position
      sprintf(buffer, "%d\n", PA );
      UARTport.print(buffer);
      UARTport.flush();
      //Serial.print(buffer);
      break;
    case 79: Serial.print(", Tilt Position? ");       //Return Tilt position

      sprintf(buffer, "%d\n", TA );
      UARTport.print(buffer);
      UARTport.flush();

      //Serial.print(buffer);
      break;
    case 80: Serial.print(", Zoom Position? ");       //Return Zoom position
      ZA = stepper4->getCurrentPosition();
      ZA = ZA / ((cam_Z_Out - cam_Z_In) / 100);           //Current position as a % of the posible movement between the limits
      sprintf(buffer, "%d\n", ZA );
      UARTport.print(buffer);
      UARTport.flush();
      break;

    case 81: //Serial.print(", Focus Position? ");       //Return Focus position
      FA = stepper3->getCurrentPosition();
      FA = FA / ((cam_F_Out - cam_F_In) / 100);           //Current position as a % of the posible movement between the limits
      sprintf(buffer, "%d\n", FA );
      UARTport.print(buffer);
      UARTport.flush();
      break;

    //PT Moves


    case 9: Serial.print("Absolute Position request");
      VISCApose();
      break;

    case 33: Serial.print("PT Stop ");
      if (pan_is_moving) {
        Joy_Pan_Speed = (0);
      }
      if (tilt_is_moving) {
        Joy_Tilt_Speed = (0);
      }
      if (focus_is_moving) {
        Joy_Focus_Speed = (0);
      }
      if (zoom_is_moving) {
        Joy_Zoom_Speed = (0);

      }
      break;





    case 13: Serial.print("Pan Left ");
      Serial.println(VPanSpeed);
      Joy_Pan_Accel = 2000;
      if (VPanSpeed < 3) {
        Joy_Pan_Speed = (VPanSpeed * 251);
        Joy_Pan_Speed = (Joy_Pan_Speed - (Joy_Pan_Speed * 2));   //Posative to negative value
      } else {
        Joy_Pan_Speed = (VPanSpeed * 200);
        Joy_Pan_Speed = (Joy_Pan_Speed - (Joy_Pan_Speed * 2));   //Posative to negative value
      }
      break;

    case 23: Serial.print("Pan Right ");
      Serial.println(VPanSpeed);
      Joy_Pan_Accel = 2000;
      if (VPanSpeed < 3) {
        Joy_Pan_Speed = (VPanSpeed * 251);

      } else {
        Joy_Pan_Speed = (VPanSpeed * 200);

      }
      break;

    case 31: Serial.print("Tilt Up ");
      Serial.println(VTiltSpeed);
      if (VTiltSpeed < 3) {
        Joy_Tilt_Speed = (VTiltSpeed * 251);
      } else {
        Joy_Tilt_Speed = (VTiltSpeed * 200);
      }
      Joy_Tilt_Speed = Joy_Tilt_Speed * 2;
      break;

    case 32: Serial.print("Tilt Down ");
      Serial.println(VTiltSpeed);
      if (VTiltSpeed < 3) {
        Joy_Tilt_Speed = (VTiltSpeed * 251);
        Joy_Tilt_Speed = (Joy_Tilt_Speed - (Joy_Tilt_Speed * 2));   //Posative to negative value
      } else {
        Joy_Tilt_Speed = (VTiltSpeed * 200);
        Joy_Tilt_Speed = (Joy_Tilt_Speed - (Joy_Tilt_Speed * 2));   //Posative to negative value
      }
      Joy_Tilt_Speed = Joy_Tilt_Speed * 2;
      break;



    case 11: Serial.print("Up Left ");
      Serial.print(VPanSpeed);
      Serial.print(" , ");
      Serial.println(VTiltSpeed);
      Joy_Pan_Accel = 2000;
      if (VPanSpeed < 3) {
        Joy_Pan_Speed = (VPanSpeed * 251);
        Joy_Pan_Speed = (Joy_Pan_Speed - (Joy_Pan_Speed * 2));   //Posative to negative value
      } else {
        Joy_Pan_Speed = (VPanSpeed * 200);
        Joy_Pan_Speed = (Joy_Pan_Speed - (Joy_Pan_Speed * 2));   //Posative to negative value
      }
      if (VTiltSpeed < 3) {
        Joy_Tilt_Speed = (VTiltSpeed * 251);
      } else {
        Joy_Tilt_Speed = (VTiltSpeed * 200);
      } break;

    case 21: Serial.print("Up Right ");
      Serial.print(VPanSpeed);
      Serial.print(" , ");
      Serial.println(VTiltSpeed);
      Joy_Pan_Accel = 2000;
      if (VPanSpeed < 3) {
        Joy_Pan_Speed = (VPanSpeed * 251);

      } else {
        Joy_Pan_Speed = (VPanSpeed * 200);

      }
      if (VTiltSpeed < 3) {
        Joy_Tilt_Speed = (VTiltSpeed * 251);
      } else {
        Joy_Tilt_Speed = (VTiltSpeed * 200);
      }

      break;

    case 12: Serial.print("Down Left ");
      Serial.print(VPanSpeed);
      Serial.print(" , ");
      Serial.println(VTiltSpeed);
      Joy_Pan_Accel = 2000;
      if (VPanSpeed < 3) {
        Joy_Pan_Speed = (VPanSpeed * 251);
        Joy_Pan_Speed = (Joy_Pan_Speed - (Joy_Pan_Speed * 2));   //Posative to negative value
      } else {
        Joy_Pan_Speed = (VPanSpeed * 200);
        Joy_Pan_Speed = (Joy_Pan_Speed - (Joy_Pan_Speed * 2));   //Posative to negative value
      }
      if (VTiltSpeed < 3) {
        Joy_Tilt_Speed = (VTiltSpeed * 251);
        Joy_Tilt_Speed = (Joy_Tilt_Speed - (Joy_Tilt_Speed * 2));   //Posative to negative value
      } else {
        Joy_Tilt_Speed = (VTiltSpeed * 200);
        Joy_Tilt_Speed = (Joy_Tilt_Speed - (Joy_Tilt_Speed * 2));   //Posative to negative value
      } break;

    case 22: Serial.print("Down Right ");
      Serial.print(VPanSpeed);
      Serial.print(" , ");
      Serial.println(VTiltSpeed);
      Joy_Pan_Accel = 2000;
      if (VPanSpeed < 3) {
        Joy_Pan_Speed = (VPanSpeed * 251);

      } else {
        Joy_Pan_Speed = (VPanSpeed * 200);

      }
      if (VTiltSpeed < 3) {
        Joy_Tilt_Speed = (VTiltSpeed * 251);
        Joy_Tilt_Speed = (Joy_Tilt_Speed - (Joy_Tilt_Speed * 2));   //Posative to negative value
      } else {
        Joy_Tilt_Speed = (VTiltSpeed * 200);
        Joy_Tilt_Speed = (Joy_Tilt_Speed - (Joy_Tilt_Speed * 2));   //Posative to negative value
      }
      break;

    //Zoom moves
    case 700: Serial.print("Zoom Stop ");
      if (zoom_is_moving) {
        Joy_Zoom_Speed = (0);
      } break;
    case 748: Serial.print("Zoom out p1  ");
      VZoomSpeed = 1010;
      Joy_Zoom_Speed = (VZoomSpeed ); break;

    case 749: Serial.print(", Zoom out p2  ");
      VZoomSpeed = 1500;
      Joy_Zoom_Speed = (VZoomSpeed ); break;
    case 750: Serial.print(", Zoom out p3  ");
      VZoomSpeed = 2000;
      Joy_Zoom_Speed = (VZoomSpeed ); break;
    case 751: Serial.print(", Zoom out p4  ");
      VZoomSpeed = 2500; break;
      Joy_Zoom_Speed = (VZoomSpeed ); break;
    case 752: Serial.print(", Zoom out p5  ");
      VZoomSpeed = 3000;
      Joy_Zoom_Speed = (VZoomSpeed ); break;
    case 753: Serial.print(", Zoom out p6  ");
      VZoomSpeed = 3500;
      Joy_Zoom_Speed = (VZoomSpeed ); break;
    case 754: Serial.print(", Zoom out p7  ");
      VZoomSpeed = 4000;
      Joy_Zoom_Speed = (VZoomSpeed ); break;
    case 755: Serial.print(", Zoom out p8  ");
      VZoomSpeed = 4500; break;
      Joy_Zoom_Speed = (VZoomSpeed ); break;
    case 756: Serial.print(", Zoom out p9  ");
      VZoomSpeed = 5000;
      Joy_Zoom_Speed = (VZoomSpeed ); break;

    case 732: Serial.print(", Zoom in p1 ");
      VZoomSpeed = 1010;
      Joy_Zoom_Speed = (VZoomSpeed );
      Joy_Zoom_Speed = (Joy_Zoom_Speed - (Joy_Zoom_Speed * 2));   //Posative to negative value
      break;

    case 733: Serial.print(", Zoom in p2 ");
      VZoomSpeed = 1500;
      Joy_Zoom_Speed = (VZoomSpeed );
      Joy_Zoom_Speed = (Joy_Zoom_Speed - (Joy_Zoom_Speed * 2));   //Posative to negative value
      break;
    case 734: Serial.print(", Zoom in p3 ");
      VZoomSpeed = 2000;
      Joy_Zoom_Speed = (VZoomSpeed );
      Joy_Zoom_Speed = (Joy_Zoom_Speed - (Joy_Zoom_Speed * 2));   //Posative to negative value
      break;
    case 735: Serial.print(", Zoom in p4 ");
      VZoomSpeed = 2500;
      Joy_Zoom_Speed = (VZoomSpeed );
      Joy_Zoom_Speed = (Joy_Zoom_Speed - (Joy_Zoom_Speed * 2));   //Posative to negative value
      break;
    case 736: Serial.print(", Zoom in p5 ");
      VZoomSpeed = 3000;
      Joy_Zoom_Speed = (VZoomSpeed );
      Joy_Zoom_Speed = (Joy_Zoom_Speed - (Joy_Zoom_Speed * 2));   //Posative to negative value
      break;
    case 737: Serial.print(", Zoom in p6 ");
      VZoomSpeed = 3500;
      Joy_Zoom_Speed = (VZoomSpeed );
      Joy_Zoom_Speed = (Joy_Zoom_Speed - (Joy_Zoom_Speed * 2));   //Posative to negative value
      break;
    case 738: Serial.print(", Zoom in p7 ");
      VZoomSpeed = 4000;
      Joy_Zoom_Speed = (VZoomSpeed );
      Joy_Zoom_Speed = (Joy_Zoom_Speed - (Joy_Zoom_Speed * 2));   //Posative to negative value
      break;
    case 739: Serial.print(", Zoom in p8 ");
      VZoomSpeed = 4500;
      Joy_Zoom_Speed = (VZoomSpeed );
      Joy_Zoom_Speed = (Joy_Zoom_Speed - (Joy_Zoom_Speed * 2));   //Posative to negative value
      break;

    //Focus
    case 800: Serial.print(", Focus Stop ");
      if (focus_is_moving) {
        Joy_Focus_Speed = (0);
      } break;

    case 832: Serial.print(", Focus Near p1 ");
      VFocusSpeed = 1010;
      Joy_Focus_Speed = (VFocusSpeed ); break;

    case 833: Serial.print(", Focus Near p2 ");
      VFocusSpeed = 1500;
      Joy_Focus_Speed = (VFocusSpeed ); break;

    case 834: Serial.print(", Focus Near p3 ");
      VFocusSpeed = 2000;
      Joy_Focus_Speed = (VFocusSpeed ); break;

    case 835: Serial.print(", Focus Near p4 ");
      VFocusSpeed = 2500;
      Joy_Focus_Speed = (VFocusSpeed ); break;

    case 836: Serial.print(", Focus Near p5 ");
      VFocusSpeed = 3000;
      Joy_Focus_Speed = (VFocusSpeed ); break;

    case 837: Serial.print(", Focus Near p6 ");
      VFocusSpeed = 3500;
      Joy_Focus_Speed = (VFocusSpeed ); break;

    case 838: Serial.print(", Focus Near p7 ");
      VFocusSpeed = 4000;
      Joy_Focus_Speed = (VFocusSpeed ); break;

    case 839: Serial.print(", Focus Near p8 ");
      VFocusSpeed = 4500;; break;
      Joy_Focus_Speed = (VFocusSpeed ); break;

    //Focus Far

    case 848: Serial.print(", Focus Far p1 ");
      VFocusSpeed = 1010;
      Joy_Focus_Speed = (VFocusSpeed );
      Joy_Focus_Speed = (Joy_Focus_Speed - (Joy_Focus_Speed * 2));   //Posative to negative value
      break;

    case 849: Serial.print(", Focus Far p2 ");
      VFocusSpeed = 1500;
      Joy_Focus_Speed = (VFocusSpeed );
      Joy_Focus_Speed = (Joy_Focus_Speed - (Joy_Focus_Speed * 2));   //Posative to negative value
      break;

    case 850: Serial.print(", Focus Far p3 ");
      VFocusSpeed = 2000;
      Joy_Focus_Speed = (VFocusSpeed );
      Joy_Focus_Speed = (Joy_Focus_Speed - (Joy_Focus_Speed * 2));   //Posative to negative value
      break;

    case 851: Serial.print(", Focus Far p4 ");
      VFocusSpeed = 2500;
      Joy_Focus_Speed = (VFocusSpeed );
      Joy_Focus_Speed = (Joy_Focus_Speed - (Joy_Focus_Speed * 2));   //Posative to negative value
      break;

    case 852: Serial.print(", Focus Far p5 ");
      VFocusSpeed = 3000;
      Joy_Focus_Speed = (VFocusSpeed );
      Joy_Focus_Speed = (Joy_Focus_Speed - (Joy_Focus_Speed * 2));   //Posative to negative value
      break;

    case 853: Serial.print(", Focus Far p6 ");
      VFocusSpeed = 3500;
      Joy_Focus_Speed = (VFocusSpeed );
      Joy_Focus_Speed = (Joy_Focus_Speed - (Joy_Focus_Speed * 2));   //Posative to negative value
      break;

    case 854: Serial.print(", Focus Far p7 ");
      VFocusSpeed = 4000;
      Joy_Focus_Speed = (VFocusSpeed );
      Joy_Focus_Speed = (Joy_Focus_Speed - (Joy_Focus_Speed * 2));   //Posative to negative value
      break;

    case 855: Serial.print(", Focus Far p8 "); break;
      VFocusSpeed = 4500;
      Joy_Focus_Speed = (VFocusSpeed );
      Joy_Focus_Speed = (Joy_Focus_Speed - (Joy_Focus_Speed * 2));   //Posative to negative value
      break;

    //Ramp
    case 4901: Serial.print(", Ramp/EaseValue=1 "); break;
    case 4902: Serial.print(", Ramp/EaseValue=2 "); break;
    case 4903: Serial.print(", Ramp/EaseValue=3 "); break;
    case 2: Serial.print(", Start Recore "); break;



    //Pose positions
    case 41: Serial.print(", Save Pose:  ");
      Serial.println(PTZ_Pose);
      PTZ_Cam = PTZ_ID;
      lastPTZ_Pose = PTZ_Pose;
      PTZ_Save();
      break;

    case 42: Serial.print(", Move to pose ");
      Serial.println(PTZ_Pose);
      lastPTZ_Pose = PTZ_Pose;
      PTZ_MoveP();
      break;

    case 40: Serial.print(", Clear Pose:  ");
      Serial.println(PTZ_Pose);
      PTZ_ClearP();
      break;

    case 137: Serial.print(", Pose speed "); break;


    //Record
    case 3: Serial.print(", Stop Record "); break;

    case 126: Serial.print(", Record Mode setting "); break;

  }
}

void PTZ_ClearP() {
  int mem0;
  switch (PTZ_Pose) {
    case 1:
      P1_T = 0;
      P1_P = 0;
      P1_F = 0;
      P1_Z = 0;
      SysMemory.begin("P1pos", false);                         //save to memory
      SysMemory.putUInt("P1_T", mem0);
      SysMemory.putUInt("P1_P", mem0);
      SysMemory.putUInt("P1_F", mem0);
      SysMemory.putUInt("P1_Z", mem0);
      SysMemory.end();
      break;
    case 2:
      P2_T = 0;
      P2_P = 0;
      P2_F = 0;
      P2_Z = 0;
      SysMemory.begin("P2pos", false);                         //save to memory
      SysMemory.putUInt("P2_T", mem0);
      SysMemory.putUInt("P2_P", mem0);
      SysMemory.putUInt("P2_F", mem0);
      SysMemory.putUInt("P2_Z", mem0);
      SysMemory.end();
      break;
    case 3:
      P3_T = 0;
      P3_P = 0;
      P3_F = 0;
      P3_Z = 0;
      SysMemory.begin("P3pos", false);                         //save to memory
      SysMemory.putUInt("P3_T", mem0);
      SysMemory.putUInt("P3_P", mem0);
      SysMemory.putUInt("P3_F", mem0);
      SysMemory.putUInt("P3_Z", mem0);
      SysMemory.end();
      break;
    case 4:
      P4_T = 0;
      P4_P = 0;
      P4_F = 0;
      P4_Z = 0;
      SysMemory.begin("P4pos", false);                         //save to memory
      SysMemory.putUInt("P4_T", mem0);
      SysMemory.putUInt("P4_P", mem0);
      SysMemory.putUInt("P4_F", mem0);
      SysMemory.putUInt("P4_Z", mem0);
      SysMemory.end();
      break;
    case 5:
      P5_T = 0;
      P5_P = 0;
      P5_F = 0;
      P5_Z = 0;
      SysMemory.begin("P5pos", false);                         //save to memory
      SysMemory.putUInt("P5_T", mem0);
      SysMemory.putUInt("P5_P", mem0);
      SysMemory.putUInt("P5_F", mem0);
      //SysMemory.putUInt("P5_Z",mem0);
      SysMemory.end();
      break;
    case 6:
      P6_T = 0;
      P6_P = 0;
      P6_F = 0;
      P6_Z = 0;
      SysMemory.begin("P6pos", false);                         //save to memory
      SysMemory.putUInt("P6_T", mem0);
      SysMemory.putUInt("P6_P", mem0);
      SysMemory.putUInt("P6_F", mem0);
      SysMemory.putUInt("P6_Z", mem0);
      SysMemory.end();
      break;
    case 7:
      P7_T = 0;
      P7_P = 0;
      P7_F = 0;
      P7_Z = 0;
      SysMemory.begin("P7pos", false);                         //save to memory
      SysMemory.putUInt("P7_T", mem0);
      SysMemory.putUInt("P7_P", mem0);
      SysMemory.putUInt("P7_F", mem0);
      SysMemory.putUInt("P7_Z", mem0);
      SysMemory.end();
      break;
    case 8:
      P8_T = 0;
      P8_P = 0;
      P8_F = 0;
      P8_Z = 0;
      SysMemory.begin("P8pos", false);                         //save to memory
      SysMemory.putUInt("P8_T", mem0);
      SysMemory.putUInt("P8_P", mem0);
      SysMemory.putUInt("P8_F", mem0);
      SysMemory.putUInt("P8_Z", mem0);
      SysMemory.end();
      break;
    case 9:
      P9_T = 0;
      P9_P = 0;
      P9_F = 0;
      P9_Z = 0;
      SysMemory.begin("P9pos", false);                         //save to memory
      SysMemory.putUInt("P9_T", mem0);
      SysMemory.putUInt("P9_P", mem0);
      SysMemory.putUInt("P9_F", mem0);
      SysMemory.putUInt("P9_Z", mem0);
      SysMemory.end();
      break;
    case 10:
      P10_T = 0;
      P10_P = 0;
      P10_F = 0;
      P10_Z = 0;
      SysMemory.begin("P10pos", false);                         //save to memory
      SysMemory.putUInt("P10_T", mem0);
      SysMemory.putUInt("P10_P", mem0);
      SysMemory.putUInt("P10_F", mem0);
      SysMemory.putUInt("P10_Z", mem0);
      SysMemory.end();
      break;
    case 11:
      P11_T = 0;
      P11_P = 0;
      P11_F = 0;
      P11_Z = 0;
      SysMemory.begin("P11pos", false);                         //save to memory
      SysMemory.putUInt("P11_T", mem0);
      SysMemory.putUInt("P11_P", mem0);
      SysMemory.putUInt("P11_F", mem0);
      SysMemory.putUInt("P11_Z", mem0);
      SysMemory.end();
      break;
    case 12:
      P12_T = 0;
      P12_P = 0;
      P12_F = 0;
      P12_Z = 0;
      SysMemory.begin("P12pos", false);                         //save to memory
      SysMemory.putUInt("P12_T", mem0);
      SysMemory.putUInt("P12_P", mem0);
      SysMemory.putUInt("P12_F", mem0);
      SysMemory.putUInt("P12_Z", mem0);
      SysMemory.end();
      break;
    case 13:
      P13_T = 0;
      P13_P = 0;
      P13_F = 0;
      P13_Z = 0;
      SysMemory.begin("P13pos", false);                         //save to memory
      SysMemory.putUInt("P13_T", mem0);
      SysMemory.putUInt("P13_P", mem0);
      SysMemory.putUInt("P13_F", mem0);
      SysMemory.putUInt("P13_Z", mem0);
      SysMemory.end();
      break;
    case 14:
      P14_T = 0;
      P14_P = 0;
      P14_F = 0;
      P14_Z = 0;
      SysMemory.begin("P14pos", false);                         //save to memory
      SysMemory.putUInt("P14_T", mem0);
      SysMemory.putUInt("P14_P", mem0);
      SysMemory.putUInt("P14_F", mem0);
      SysMemory.putUInt("P14_Z", mem0);
      SysMemory.end();
      break;
    case 15:
      P15_T = 0;
      P15_P = 0;
      P15_F = 0;
      P15_Z = 0;
      SysMemory.begin("P15pos", false);                         //save to memory
      SysMemory.putUInt("P15_T", mem0);
      SysMemory.putUInt("P15_P", mem0);
      SysMemory.putUInt("P15_F", mem0);
      SysMemory.putUInt("P15_Z", mem0);
      SysMemory.end();
      break;
    case 16:
      P16_T = 0;
      P16_P = 0;
      P16_F = 0;
      P16_Z = 0;
      SysMemory.begin("P16pos", false);                         //save to memory
      SysMemory.putUInt("P16_T", mem0);
      SysMemory.putUInt("P16_P", mem0);
      SysMemory.putUInt("P16_F", mem0);
      SysMemory.putUInt("P16_Z", mem0);
      SysMemory.end();
      break;

  }
}

void VISCApose() {

  //  T_position = (P16_T);
  //  P_position = (P16_P);
  //  F_position = (P16_F);
  //  Z_position = (P16_Z);
  //Angle position back to steps
  P_position = (P_position * 22);
  T_position = (T_position * 325);
  F_position = cam_F_In - ((cam_F_Out - cam_F_In) / 100) * F_position;                    //Turn the VISCA % back to a Step position
  Z_position = cam_Z_In - ((cam_Z_Out - cam_Z_In) / 100) * Z_position;                    //Turn the VISCA % back to a Step position
  //Z_position = ((cam_Z_Out - cam_Z_In) / 100) * F_position;                      //Turn the VISCA % back to a Step position
  //******************Stepper moves************************************************************
  if (stepper2->getCurrentPosition() != (P_position) && (P_position != 10)) {    //Run to  position

    stepper1->setSpeedInHz(VTiltSpeed * 200);                //Tilt
    stepper1->setAcceleration(2000);

    stepper2->setSpeedInHz(VPanSpeed * 200);                //Pan
    stepper2->setAcceleration(1000);

    stepper3->setSpeedInHz(2000);                           //Focus
    stepper3->setAcceleration(1000);

    stepper4->setSpeedInHz(2000);                      //Zoom
    stepper4->setAcceleration(1000);


    stepper1->moveTo(T_position);
    delay(10);
    stepper2->moveTo(P_position);
    delay(10);
    stepper3->moveTo(F_position);
    delay(10);
    stepper4->moveTo(Z_position);

    while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() || stepper4->isRunning() ) {
      delay(10);
    }
  }
}








void saveIP() {
  Serial.printf("Saving IP Adress: %d.%d.%d.%d %d\n", IP1, IP2, IP3, IP4, IPGW, UDP);
  SysMemory.begin("IPvalues", false);                         //save to memory
  SysMemory.putUInt("IP1", IP1);
  SysMemory.putUInt("IP2", IP2);
  SysMemory.putUInt("IP3", IP3);
  SysMemory.putUInt("IP4", IP4);
  SysMemory.putUInt("IPGW", IPGW);
  SysMemory.putUInt("UDP", UDP);
  SysMemory.end();
}

void Home() {
  Serial.print("Running HOME ");
  digitalWrite(StepFOC, LOW);                                   //Power up the steppers
  digitalWrite(StepD, LOW);
  stepper1->setSpeedInHz(5000);                                 //Tilt
  stepper1->setAcceleration(2000);

  stepper2->setSpeedInHz(1500);                                  //Pan
  stepper2->setAcceleration(1000);

  stepper3->setSpeedInHz(2000);                                  //Focus
  stepper3->setAcceleration(1000);

  stepper4->setSpeedInHz(2000);                                  //Zoom
  stepper4->setAcceleration(1000);


  stepper1->moveTo(0);
  delay(10);
  stepper2->moveTo(0);
  delay(10);
  stepper3->moveTo(0);
  delay(10);
  stepper4->moveTo(0);

  while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() || stepper4->isRunning()) {
    delay(10);
  }
}

void homeStepper() {

  stepper1->setSpeedInHz(1500);                         // Home the tilt
  stepper1->setAcceleration(700);
  stepper1->move(8000);
  while (digitalRead(Hall_Tilt) == HIGH) {             // Make the Stepper move CCW until the switch is activated
    delay(20);                                         //"MoveTo" move to a position and stop,
  }
  stepper1->forceStopAndNewPosition(5);                  //Stops dead
  stepper1->move(-1400);
  delay(20);
  while (stepper1->isRunning() || stepper2->isRunning()) {
    delay(20);
  }

  stepper1->setCurrentPosition(0);                       //Set this position as home 0
  //  T_ResetEncoder = 1;
  //  Start_T_Encoder();
  //  //vTaskDelay(20);


  stepper2->setSpeedInHz(300);                         // Home the Pan
  stepper2->setAcceleration(300);
  stepper2->move(3000);
  while (digitalRead(Hall_Pan) == HIGH) {
    delay(10);
  }
  stepper2->forceStopAndNewPosition(5);                  //Stops dead
  stepper2->move(-800);
  delay(20);
  while (stepper2->isRunning()) {
    delay(20);
  }

  stepper2->setCurrentPosition(0);                       //Set this position as home 0
  //  P_ResetEncoder = 1;
  //  Start_P_Encoder();
  //  //vTaskDelay(20);



  SendNextionValues();
  return;

}
