//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*Digital Bird OT PanTilt Head part of the Digital Bird Motion Control System Created by Colin Henderson 2021
  This code is in the public domain...
  You can: copy it, use it, modify it, share it or just plain ignore it!
  Thx!
*/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This version is for the Over The Top pan/tilt head

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


// As in StepperDemo for Motor 1 on ESP32

#define step1_pinSTEP 26                      // Pin Gpio 12 connected to STEP pin of Stepper Driver
#define step1_pinDIR  27                      // Pin Gpio 26 connected to DIR pin of Stepper Driver
#define step2_pinSTEP 12                      // Pin Gpio 12 connected to STEP pin of Stepper Driver
#define step2_pinDIR  14                      // Pin Gpio 26 connected to DIR pin of Stepper Driver
#define step3_pinSTEP 25                      // Pin Gpio 12 connected to STEP pin of Stepper Driver
#define step3_pinDIR  33                      // Pin Gpio 26 connected to DIR pin of Stepper Driver


#define CAM A10                               // Pin Gpio 4 Camera trigger
#define StepD 13                              // Pin GPio 13 Activate Deactivate steppers
#define Mount_PIN A12                         //Switch Gpio 2 to mount the head
#define DisMount_PIN A13                      //Switch Gpio 15 to dismount the head
#define StepFOC 32                            //Focus motor enable pin for Stepper3

const byte pan_PIN = A6;                      // pin 34 Joy Stick input
const byte tilt_PIN = A7;                     // Tilt 35 Joy Stick input
const int BatPin = 39;                        //Battery monitoring pin
float BatV;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;



//Variables
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

float speedfactor = 1;
float accelerationfactor = 1;
int pan_is_moving;
int tilt_is_moving;
int focus_is_moving;

String DBFirmwareVersion = "frm V:4.00";      //Current firmware version

//*******************************************************************************************************************
//Change these valus depending on which pan tilt head you havebuilt over the top or balanced
int Tilt_J_Speed = 150;                       //Set this to 150 for the Balanced PT 200 for the Over the Top PT
int Pan_J_Speed = 4000;                        //Set this to 4000  for both the Balanced PT and Over the Top PT
//********************************************************************************************************************
//****************************PTZ Variables*************************************//
int Joy_Pan_Speed;                           //PTZ variables
int Joy_Pan_Accel;
int Joy_Tilt_Speed;
int Joy_Tilt_Accel;
int Joy_Focus_Speed;
int Joy_Focus_Accel;
int Joy_Zoom_Speed;
int Joy_Zoom_Accel;


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
int cam_F_Out = 16000;                       //Actual value used dependant on ID

int F_Stop;                                   //Used to prevent the focus motor crashing into the end over and over
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

int P1_P = 10;                          //PTZ Pan positions
int P2_P = 10;
int P3_P = 10;
int P4_P = 10;
int P5_P = 10;
int P6_P = 10;

int P1_F = 10;                          //PTZ Focus positions
int P2_F = 10;
int P3_F = 10;
int P4_F = 10;
int P5_F = 10;
int P6_F = 10;

int P1_Z = 10;                          //PTZ Zoom positions
int P2_Z = 10;
int P3_Z = 10;
int P4_Z = 10;
int P5_Z = 10;
int P6_Z = 10;

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
int Mount = 0;                                 //Is the Mount function active 1 true 0 false

long tilt;
long pan;

long TiltJoySpeed;
long PanJoySpeed;
long tilt_AVG;
long pan_AVG;


int PANin_position = 0;                      // variable to hold IN positions for slider
int TLTin_position = 0;
int FOCin_position = 0;

int PANout_position = 0;                     // variable to hold OUT position for slider
int TLTout_position = 0;
int FOCout_position = 0;

int PANtravel_dist = 0;                        //Distance to travel
int TLTtravel_dist = 0;
int FOCtravel_dist = 0;

float PANstep_speed = 0;                        // default travel speed between IN and OUT points
float TLTstep_speed = 0;
float FOCstep_speed = 0;

int PANfps_step_dist = 0;
int TLTfps_step_dist = 0;
int FOCfps_step_dist = 0;

int TLTease_Value = 0;                      // Variable to hold actual Ease value used for acceleration calculation
int PANease_Value = 0;
int FOCease_Value = 0;

int  TLTTlps_step_dist;
int  PANTlps_step_dist;
int  FOCTlps_step_dist;

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

int Nextion_Fnc;
int Nextion_play;
int But_Com;
int TpsM;

int ResetEncoder = 1;

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


//Structure to send WIFI data. Must match the receiver structure currently 136bytes of 250 max fo ESP-Now
typedef struct struct_message {

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

  int Ez;       //6                                   //Ease Value
  int Bo;                                             //Number of bounces
  int TT;                                             //Is the turntable present 1 or 0
  int PT;                                             //Is the turntable present 1 or 0 2 PanTilt Bounce trigger prevents cumulative speed errors
  int Sld;                                                //Is the Slider  present 1 or 0
  int JB;                                                //Is the Jib  present 1 or 0
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
  //int mess;
} struct_message;

struct_message NextionValues;                             //Define structure for sending values to remote Nextion Control
struct_message incomingValues;                            //Define structure for receiving values from remote Nextion control


AS5600 encoder;                                           //AMS AS5600 Encoder setup   The encoder keeps track of the focus motor position on stepper3


int WIFIOUT[8];                                           //set up 5 element array for sendinf values to pantilt
//#define RXD2 16                                           //Hardware Serial2 on ESP32 Dev (must also be a common earth between nextion and esp32)
//#define TXD2 17

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


  Serial.begin(115200);
  //  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Wire.begin();
  Serial.println("\nRunning DigitalBird Pan Tilt Head ESP32 PT1.0\n");


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
  pinMode(Mount_PIN, INPUT_PULLUP);                             //pulled high Mount switch
  pinMode(DisMount_PIN, INPUT_PULLUP);                          //pulled high DisMount switch
  pinMode(tilt_PIN, INPUT);                                     //Joystick Tilt
  pinMode(pan_PIN, INPUT);                                      //Joystick pan
  pinMode (CAM, OUTPUT);                                        //Camera Shutter
  digitalWrite(StepD, LOW);                                     //Stepper Driver Activation
  digitalWrite(StepFOC, LOW);                                   //Enable Focus motor
  digitalWrite(CAM, LOW);                                       //Camera Shutter
  stepper1->setCurrentPosition(0);                              //Set all steppers to position 0
  stepper2->setCurrentPosition(0);
  stepper3->setCurrentPosition(0);


  //SysMemory Recover last setup
  SysMemory.begin("ID", false);                                 //Recover the last PTZ_ID from memory before shutdown
  PTZ_ID = SysMemory.getUInt("PTZ_ID", 4);
  SysMemory.end();
  lastPTZ_ID = PTZ_ID;                                          //set last PTZ_ID for comparison when changing ID
  Serial.printf("PTZ_ID is set to: %d \n", PTZ_ID);

  Load_SysMemory();
  Serial.printf("\ncam_F_Out: %d \n", cam_F_Out);
  Serial.printf("\ncam_F_In: %d \n", cam_F_In);

  digitalWrite(StepFOC, LOW);                                   //Power UP the Focus stepper

  InitialValues();                                              //onboard Joystick calobration



}//end setup


void loop() {

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
    ReadMount();
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
        //SendNextionValues();
        OutP = 0;                                            //Reset Inpoint press back to 0
        lastOutP = 0;
        break;
    }
  }


  if  (Nextion_play == 1 && (PTZ_ID == 4)) {                  //Play comand receved
    if (stepper1->getCurrentPosition() != (TLTin_position) || stepper2->getCurrentPosition() != (PANin_position) ) {
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
    digitalWrite(StepFOC, HIGH);                                    //Power down the Focus stepper for manual positioning
    delay(20);
    // FS_Limits();
    while (LM == 1) {
      cam_F_In = (S_position);                                      //Set position IN
      stepper3->setCurrentPosition(S_position);
      delay(10);
    }
    SysMemory.begin("FocLmts", false);                              //save to memory
    SysMemory.putUInt("cam_F_In", cam_F_In);
    SysMemory.end();

    while (LM == 2) {
      cam_F_Out = (S_position);                                     //Set position OUT
      stepper3->setCurrentPosition(S_position);
      delay(10);
    }
    SysMemory.begin("FocLmts", false);                        //save to memory
    SysMemory.putUInt("cam_F_Out", cam_F_Out);
    SysMemory.end();
    //Serial.printf("\ncam_F_Out: %d \n", cam_F_Out);
    if (LM == 0) {


      digitalWrite(StepFOC, LOW);                                    //Power UP the Focus stepper only for manual positioning
      delay(20);
      stepper3->setSpeedInHz(3000);
      stepper3->setAcceleration(1000);
      stepper3->moveTo(cam_F_In);
      while (stepper3->isRunning()) {
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




//*********************************************************StartEncoder*************************************
void coreas1signments( void * pvParameters ) {
  for (;;) {
    StartEncoder();
    vTaskDelay(10);
    BatCheck();
  };
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

  S_position = ((E_position / 2.56));       //Ajust encoder to stepper values the number of steps eqiv
  S_position = (S_position * 2);            //Ajust encoder to stepper values the number of steps eqiv
  //Serial.println(S_position);
}





//*********************************************************INpoint set*************************************
void INpointSet() {
  // Buttonset = 1;
  //digitalWrite(StepFOC, HIGH);
  delay(10);
  switch (InP) {
    case 1:                                                       //First Press: Request a new Inpoint
      if (stepper1->getCurrentPosition() != (TLTin_position) || stepper2->getCurrentPosition() != (PANin_position) || stepper3->getCurrentPosition() != (FOCin_position) ) {  //Run to In position
        stepper1->setSpeedInUs(200);
        stepper1->setAcceleration(3000);

        stepper2->setSpeedInUs(200);
        stepper2->setAcceleration(3000);

        stepper3->setSpeedInUs(50);
        stepper3->setAcceleration(2000);

        stepper1->moveTo(TLTin_position);
        delay(10);
        stepper2->moveTo(PANin_position);
        delay(10);
        stepper3->moveTo(FOCin_position);


        while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() ) {                           //delay until move complete
          delay(10);
        }
      }
      if (usejoy == false) {
        digitalWrite(StepFOC, HIGH);                                    //Power down the Focus stepper only for manual positioning
        delay(10);
      }
      if (usejoy) {
        PTZ_Control();
      }
      //InP = 0;
      break;

    case 2:                                                           //Second inpoint press take the encoder values and set as inpoint
      TLTin_position = stepper1->getCurrentPosition();                //Burn in positions
      PANin_position = stepper2->getCurrentPosition();
      FOCin_position = (S_position);                                  //Only the Focus motor has an encoder this is getting the value from the encoder
      stepper3->setCurrentPosition(S_position);
      InP = 0;
      delay(10);

      SysMemory.begin("In_positions", false);                         //save to memory
      SysMemory.putUInt("TLTin_position", TLTin_position);
      SysMemory.putUInt("PANin_position", PANin_position);
      SysMemory.putUInt("FOCin_position", FOCin_position);
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
      if (stepper1->getCurrentPosition() != (TLTout_position) || stepper2->getCurrentPosition() != (PANout_position) || stepper3->getCurrentPosition() != (FOCout_position) ) {

        stepper1->setSpeedInUs(200);                                 //Setup stepper speed and acceloration
        stepper1->setAcceleration(3000);
        stepper2->setSpeedInUs(200);
        stepper2->setAcceleration(3000);
        stepper3->setSpeedInUs(50);
        stepper3->setAcceleration(2000);

        stepper1->moveTo(TLTout_position);                            //Start moves
        delay(10);
        stepper2->moveTo(PANout_position);
        delay(10);
        stepper3->moveTo(FOCout_position);

        while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() ) {                           //delay until move complete
          delay(10);
        }
      }
      if (usejoy == false) {
        digitalWrite(StepFOC, HIGH);                                      //Power down the Focus stepper for manual positioning
        delay(10);
      }
      if (usejoy) {
        PTZ_Control();
      }
      //Serial.println("leaving out press1");
      //OutP = 0;
      break;

    case 2:                                                           //Second outpoint press take the encoder values and set as inpoint
      TLTout_position = stepper1->getCurrentPosition();                //Burn in positions
      PANout_position = stepper2->getCurrentPosition();
      FOCout_position = (S_position);
      stepper3->setCurrentPosition(S_position);
      OutP = 0;

      SysMemory.begin("Out_positions", false);                         //save to memory
      SysMemory.putUInt("TLTout_position", TLTout_position);
      SysMemory.putUInt("PANout_position", PANout_position);
      SysMemory.putUInt("FOCout_position", FOCout_position);
      SysMemory.end();


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
        //Z_position = (P1_Z);
      }
      break;
    case 2:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P2_T);
        P_position = (P2_P);
        F_position = (P2_F);
        //Z_position = (P2_Z);
      }
      break;
    case 3:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P3_T);
        P_position = (P3_P);
        F_position = (P3_F);
        //Z_position = (P3_Z);
      }
      break;
    case 4:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P4_T);
        P_position = (P4_P);
        F_position = (P4_F);
        //Z_position = (P4_Z);
      }
      break;
    case 5:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P5_T);
        P_position = (P5_P);
        F_position = (P5_F);
        //Z_position = (P5_Z);
      }
      break;
    case 6:
      if (PTZ_Cam == PTZ_ID) {
        T_position = (P6_T);
        P_position = (P6_P);
        F_position = (P6_F);
        //Z_position = (P6_Z);
      }
      break;
  }


  //******************Stepper moves************************************************************
  if (stepper2->getCurrentPosition() != (P_position) && (P_position != 10)) {    //Run to  position

    stepper1->setSpeedInHz(2000);                           //Tilt
    stepper1->setAcceleration(1000);

    stepper2->setSpeedInHz(1500);                           //Pan
    stepper2->setAcceleration(1000);

    stepper3->setSpeedInHz(2000);                           //Focus
    stepper3->setAcceleration(1000);

    //   stepper4->setSpeedInHz(1000);                      //Zoom
    //   stepper4->setAcceleration(500);


    stepper1->moveTo(T_position);
    delay(10);
    stepper2->moveTo(P_position);
    delay(10);
    stepper3->moveTo(F_position);
    //delay(10);
    //stepper4->moveTo(Z_position);

    while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() ) {
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
        //P1_Z = (stepper3->getCurrentPosition());

        SysMemory.begin("P1pos", false);                         //save to memory
        SysMemory.putUInt("P1_T", P1_T);
        SysMemory.putUInt("P1_P", P1_P);
        SysMemory.putUInt("P1_F", P1_F);
        //SysMemory.putUInt("P1_Z", P1_Z);
        SysMemory.end();
      }
      break;
    case 2:
      if (PTZ_Cam == PTZ_ID) {
        P2_T = (stepper1->getCurrentPosition());
        P2_P = (stepper2->getCurrentPosition());
        P2_F = (stepper3->getCurrentPosition());
        //P2_Z = (stepper3->getCurrentPosition());

        SysMemory.begin("P2pos", false);                         //save to memory
        SysMemory.putUInt("P2_T", P2_T);
        SysMemory.putUInt("P2_P", P2_P);
        SysMemory.putUInt("P2_F", P2_F);
        //SysMemory.putUInt("P2_Z", P2_Z);
        SysMemory.end();
      }
      break;
    case 3:
      if (PTZ_Cam == PTZ_ID) {
        P3_T = (stepper1->getCurrentPosition());
        P3_P = (stepper2->getCurrentPosition());
        P3_F = (stepper3->getCurrentPosition());
        //P3_Z = (stepper3->getCurrentPosition());

        SysMemory.begin("P3pos", false);                         //save to memory
        SysMemory.putUInt("P3_T", P3_T);
        SysMemory.putUInt("P3_P", P3_P);
        SysMemory.putUInt("P3_F", P3_F);
        //SysMemory.putUInt("P3_Z", P3_Z);
        SysMemory.end();
      }
      break;
    case 4:
      if (PTZ_Cam == PTZ_ID) {
        P4_T = (stepper1->getCurrentPosition());
        P4_P = (stepper2->getCurrentPosition());
        P4_F = (stepper3->getCurrentPosition());
        //P4_Z = (stepper3->getCurrentPosition());

        SysMemory.begin("P4pos", false);                         //save to memory
        SysMemory.putUInt("P4_T", P4_T);
        SysMemory.putUInt("P4_P", P4_P);
        SysMemory.putUInt("P4_F", P4_F);
        //SysMemory.putUInt("P4_Z", P4_Z);
        SysMemory.end();
      }
      break;
    case 5:
      if (PTZ_Cam == PTZ_ID) {
        P5_T = (stepper1->getCurrentPosition());
        P5_P = (stepper2->getCurrentPosition());
        P5_F = (stepper3->getCurrentPosition());
        //P5_Z = (stepper3->getCurrentPosition());

        SysMemory.begin("P5pos", false);                         //save to memory
        SysMemory.putUInt("P5_T", P5_T);
        SysMemory.putUInt("P5_P", P5_P);
        SysMemory.putUInt("P5_F", P5_F);
        //SysMemory.putUInt("P5_Z", P5_Z);
        SysMemory.end();
      }
      break;
    case 6:
      if (PTZ_Cam == PTZ_ID) {
        P6_T = (stepper1->getCurrentPosition());
        P6_P = (stepper2->getCurrentPosition());
        P6_F = (stepper3->getCurrentPosition());
        //P6_Z = (stepper3->getCurrentPosition());

        SysMemory.begin("P6pos", false);                         //save to memory
        SysMemory.putUInt("P6_T", P6_T);
        SysMemory.putUInt("P6_P", P6_P);
        SysMemory.putUInt("P6_F", P6_F);
        //SysMemory.putUInt("P6_Z", P6_Z);
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
      Foc_k1_position = (S_position);
      //      Zoom_k1_position= (S_position);

      SysMemory.begin("k1_pos", false);                               //save to memory
      SysMemory.putUInt("Tlt_k1_pos", Tlt_k1_position);
      SysMemory.putUInt("Pan_k1_pos", Pan_k1_position);
      SysMemory.putUInt("Foc_k1_pos", Foc_k1_position);
      //SysMemory.putUInt("Foc_k1_pos", Foc_k1_position);
      SysMemory.end();

      stepper3->setCurrentPosition(S_position);
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
      Foc_k2_position = (S_position);
      //Zoom_k2_position = (S_position);

      SysMemory.begin("k2_pos", false);                               //save to memory
      SysMemory.putUInt("Tlt_k2_pos", Tlt_k2_position);
      SysMemory.putUInt("Pan_k2_pos", Pan_k2_position);
      SysMemory.putUInt("Foc_k2_pos", Foc_k2_position);
      //SysMemory.putUInt("Foc_k2_pos", Foc_k2_position);
      SysMemory.end();

      stepper3->setCurrentPosition(S_position);
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
      Foc_k3_position = (S_position);
      //Foc_k3_position = (S_position);

      SysMemory.begin("k3_pos", false);                               //save to memory
      SysMemory.putUInt("Tlt_k3_pos", Tlt_k3_position);
      SysMemory.putUInt("Pan_k3_pos", Pan_k3_position);
      SysMemory.putUInt("Foc_k3_pos", Foc_k3_position);
      //SysMemory.putUInt("Foc_k3_pos", Foc_k3_position);
      SysMemory.end();
      stepper3->setCurrentPosition(S_position);
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
      Foc_k4_position = (S_position);
      //Zoom_k4_position = (S_position);
      SysMemory.begin("k4_pos", false);                               //save to memory
      SysMemory.putUInt("Tlt_k4_pos", Tlt_k4_position);
      SysMemory.putUInt("Pan_k4_pos", Pan_k4_position);
      SysMemory.putUInt("Foc_k4_pos", Foc_k4_position);
      //SysMemory.putUInt("Zoom_k4_pos", Zoom_k4_position);
      SysMemory.end();

      stepper3->setCurrentPosition(S_position);
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
      Foc_k5_position = (S_position);
      //Zoom_k5_position = (S_position);

      SysMemory.begin("k5_pos", false);                               //save to memory
      SysMemory.putUInt("Tlt_k5_pos", Tlt_k5_position);
      SysMemory.putUInt("Pan_k5_pos", Pan_k5_position);
      SysMemory.putUInt("Foc_k5_pos", Foc_k5_position);
      //SysMemory.putUInt("Foc_k5_pos", Foc_k5_position);
      SysMemory.end();


      stepper3->setCurrentPosition(S_position);
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
      Foc_k6_position = (S_position);
      //Zoom_k6_position = (S_position);
      SysMemory.begin("k6_pos", false);                               //save to memory
      SysMemory.putUInt("Tlt_k6_pos", Tlt_k6_position);
      SysMemory.putUInt("Pan_k6_pos", Pan_k6_position);
      SysMemory.putUInt("Foc_k6_pos", Foc_k6_position);
      //SysMemory.putUInt("Foc_k6_pos", Foc_k6_position);
      SysMemory.end();

      stepper3->setCurrentPosition(S_position);
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
  Sld = int(incomingValues.Sld);               //The slider is the originator so here for referance only
  //PT = int(incomingValues.PT);               //Pan Tilt available not required by Pan Tilt
  TT = int(incomingValues.TT);                 //Turntable available 1 or 0
  JB = int(incomingValues.JB);                   //Jib available 1 or 0
  Joy_Pan_Speed = int(incomingValues.Ps);      //Joystick Pan PTZ Controler speed
  Joy_Pan_Accel = int(incomingValues.Ma) / 4;   //Joystick Pan PTZ Controler Acceloration

  Joy_Tilt_Speed = int(incomingValues.Ts);      //Joystick Pan PTZ Controler speed
  Joy_Tilt_Accel = int(incomingValues.Ma);      //Joystick Pan PTZ Controler Acceloration

  Joy_Focus_Speed = int(incomingValues.Fs);      //Joystick Pan PTZ Controler speed
  Joy_Focus_Accel = int(incomingValues.Ma);      //Joystick Pan PTZ Controler Acceloration

  Joy_Zoom_Speed = int(incomingValues.Zs);      //Joystick Pan PTZ Controler speed
  Joy_Zoom_Accel = int(incomingValues.Ma);      //Joystick Pan PTZ Controler Acceloration

  PTZ_ID_Nex = int(incomingValues.ID);               //PTZ messagefor ID
  PTZ_Cam = int(incomingValues.CA);              //Current PTZ camera
  PTZ_SaveP = int(incomingValues.SP);            //PTZ save position 1 or 0
  PTZ_Pose = int(incomingValues.Pz);             //Current PTZ pose 1-6

  stopM_play = int(incomingValues.SM);           //Stopmotion Trigger SM
  stopM_frames = int(incomingValues.Tps);
  stopM_cancel = int(incomingValues.SC);         //Stopmmotion cancel SC
  usejoy = int(incomingValues.Jb);               //Joystick button state 1 or 0
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
  NextionValues.Ez = ease_InOut;            // Acceloration control
  NextionValues.Bo = Bounce;                 // Number of bounces
  NextionValues.Cro = Crono_time;           // Time in sec of system move
  //NextionValues.Fnc = Nextion_Fnc;         // Current Func of nextion display
  NextionValues.Tps = Tps;                  // Timlapse frames
  NextionValues.TpsD = TpsD;                // Timplapse delay or shutter open time in Bulb mode
  //NextionValues.TpsM = TpsM;                // Timelapse move command
  //NextionValues.In = PANin_position;      // InPoint step position value
  //NextionValues.Out = PANout_position;    // OutPoint Step position value
  NextionValues.Play = Nextion_play;        // play comand
  NextionValues.But = But_Com;             // Button reply
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
  //NextionValues.mess = mess;                  //message window
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(broadcastAddress, (uint8_t *) &NextionValues, sizeof(NextionValues));
}


//*********************************************ARM The SLIDER READY TO START**************************************************
void Start_1() {

  //  Arm the slider to start point ready for second play press
  //Serial.print("Entering Start1 ");
  digitalWrite(StepFOC, LOW);                                                                              //Engage the focus motor

  delay(100);

  stepper1->setSpeedInUs(150);
  stepper1->setAcceleration(1500);
  stepper2->setSpeedInUs(2500);
  stepper2->setAcceleration(1000);
  stepper3->setSpeedInUs(50);
  stepper3->setAcceleration(2000);
  if (SEQ == 0) {
    stepper1->moveTo(TLTin_position);
    delay(10);
    stepper2->moveTo(PANin_position);
    delay(10);
    stepper3->moveTo(FOCin_position);
  }
  if (SEQ == 1) {
    stepper1->moveTo(Tlt_k1_position);
    delay(10);
    stepper2->moveTo(Pan_k1_position);
    delay(10);
    stepper3->moveTo(Foc_k1_position);
  }
  while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning() ) {                              //delay until move complete (block)
    delay(10);
  }

  But_Com = 5;                                                                                                   //Stop the nextion timer

  if (PTZ_Cam == 0) {
    SendNextionValues();
    delay(100);
  }
  But_Com = 0;
}

//************************************************RUN THE SEQUENCE BASIC**************************************************************
void Start_2() {
  digitalWrite(StepFOC, LOW);                                       //Engage the focus stepper

  if (Sld == 0 && JB == 0) {                                                 //Only take charhe of the counting if no slider
    But_Com = 4;                                                    //Start the Nextion timer
    if (PTZ_Cam == 0) {
      SendNextionValues();
    }
    But_Com = 0;
  }
  if (TLTtravel_dist != 0) {
    stepper1->setSpeedInHz(TLTstep_speed);                           //Setup speed and acceloration values
    stepper1->setAcceleration(TLTease_Value);
    stepper1->moveTo(TLTout_position);                               //Move the steppers
    delay(2);
  }
  if (PANtravel_dist != 0) {
    stepper2->setSpeedInHz(PANstep_speed);
    stepper2->setAcceleration(PANease_Value);
    stepper2->moveTo(PANout_position);
    delay(2);
  }
  if (FOCtravel_dist != 0) {
    stepper3->setSpeedInHz(FOCstep_speed);
    stepper3->setAcceleration(FOCease_Value);
    stepper3->moveTo(FOCout_position);
  }
  while (stepper2->isRunning() || stepper1->isRunning() || stepper3->isRunning()   ) {           //delay until move complete (block)
    delay(2);
  }

  if (Bounce >= 1) {
    Start_3();
  } else  {                                                              // Count down number of bounces required
    digitalWrite(CAM, HIGH);                                             //Stop camera
    delay (200);
    digitalWrite(CAM, LOW);

    if (Sld == 0) {
      But_Com = 5;                                                       //Tell the nextion to stop the timer
      PT = 1;
      if (PTZ_Cam == 0) {
        SendNextionValues();
      }
      But_Com = 0;
    }
    Start_1();
  }

}

//******************************************RUN THE SEQUENCE with Bounce************************************************
void Start_3() {

  //BatCheck();
  stepper1->setSpeedInHz(TLTstep_speed);                 //Setup speed and acceloration values
  stepper1->setAcceleration(TLTease_Value);
  stepper2->setSpeedInHz(PANstep_speed);
  stepper2->setAcceleration(PANease_Value);
  stepper3->setSpeedInHz(FOCstep_speed);
  stepper3->setAcceleration(FOCease_Value);

  stepper1->moveTo(TLTin_position);                      //Move the steppers
  delay(2);
  stepper2->moveTo(PANin_position);
  delay(2);
  stepper3->moveTo(FOCin_position);
  while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning()) {                       //delay until move complete (block)
    delay(2);
  }
  if (Sld == 0) {                                      //Only take charge of the counting if no slider
    Bounce = Bounce - 1;
    But_Com = 6;                                       //Send current bounce counter back to nextion
    if (PTZ_Cam == 0) {
      SendNextionValues();
    }
    But_Com = 0;
  }
  if (Bounce >= 1) {

    PT = 2;
    if (PTZ_Cam == 0) {
      SendNextionValues();
      delay(500);
    }
    PT = 1;
    if (PTZ_Cam == 0) {
      SendNextionValues();
    }
    Start_2();

  } else {
    if (Sld == 0) {
      But_Com = 5;                                       //Tell the nextion to stop the timer
      PT = 1;
      if (PTZ_Cam == 0) {
        SendNextionValues();
      }
      But_Com = 0;
    }
    Start_1();
  }
  return;
}

//**********************************************Timelapse*******************************************
void Start_4() {
  digitalWrite(StepFOC, LOW);
  TLTtravel_dist = (TLTout_position - TLTin_position);                     // Distance we have to go
  PANtravel_dist = (PANout_position - PANin_position);                     // Distance we have to go
  FOCtravel_dist = (FOCout_position - FOCin_position);                     // Distance we have to go
  // TpsD = (crono_seconds * 1000);                                        //use the timer seconds as delay time for timelapse

  TLTTlps_step_dist = TLTtravel_dist / Tps;
  PANTlps_step_dist = PANtravel_dist / Tps;
  FOCTlps_step_dist = FOCtravel_dist / Tps;


  TLTstep_speed = (TLTtravel_dist / 5);
  PANstep_speed = (PANtravel_dist / 5);
  FOCstep_speed = (FOCtravel_dist / 5);


  stepper1->setSpeedInUs(300);
  stepper1->setAcceleration(1500);
  stepper2->setSpeedInUs(300);
  stepper2->setAcceleration(1500);
  stepper3->setSpeedInUs(50);
  stepper3->setAcceleration(2000);

  while (Tps >= 1) {
    //Oledmessage()
    if (Sld > 0 || JB > 0 ) {                                                     //If Slider or Jib  is conected give slider or jib  control over timing

      if (TpsM == 4) {                                                           //Trigger pantilt head timelapse move
        //Serial.println("Tps Trigger from slider");

        TpsM = 0;
        stepper1->moveTo(stepper1->getCurrentPosition() + TLTTlps_step_dist);    //Current position plus one fps move
        delay(5);
        stepper2->moveTo(stepper2->getCurrentPosition() + PANTlps_step_dist);    //Current position plus one fps move
        delay(5);
        stepper3->moveTo(stepper3->getCurrentPosition() + FOCTlps_step_dist);    //Current position plus one fps move

        while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning()) {                       //delay until move complete
          delay(5);
        }
        while (TpsM != 5) {
          delay(10);
        }
        //delay (500);                                                           //Take a second to ensure camera is still
        digitalWrite(CAM, HIGH);                                                 //Fire shutter
        delay(TpsD);                                                           //Time the shutter is open for
        digitalWrite(CAM, LOW);                                                 //Cose the shutter and move on

        if (Tps != 0) {
          Tps = Tps - 1;
        }

      }

    }

    if (Sld == 0 && JB == 0) {                                                  //If Slider or Jib is not with us take control
      //Serial.println("Tlps Trigger from PT");
      //Trigger pantilt head timelapse move
      SendNextionValues();                                                      //Trigger pantilt head timelapse move
      stepper1->moveTo(stepper1->getCurrentPosition() + TLTTlps_step_dist);     //Current position plus one fps move
      delay(10);
      stepper2->moveTo(stepper2->getCurrentPosition() + PANTlps_step_dist);     //Current position plus one fps move
      delay(10);
      stepper3->moveTo(stepper3->getCurrentPosition() + FOCTlps_step_dist);     //Current position plus one fps move

      if (Tps != 0) {
        Tps = Tps - 1;
      }
      But_Com = 7;                                                              //Tell the nextion to update the frame counter
      SendNextionValues();
      But_Com = 0;
      while (stepper1->isRunning()) {                                           //delay until move complete (blocking)
        delay(10);
      }
      delay (1000);                                                             //Take a second to ensure camera is still
      digitalWrite(CAM, HIGH);                                                  //Fire shutter
      delay(TpsD);                                                             //Time the shutter is open for
      digitalWrite(CAM, LOW);                                                   //Cose the shutter and move on
      delay (500);

    }
  }



  But_Com = 5;                                                                  //Tell the nextion to stop the timer
  SendNextionValues();
  But_Com = 0;

  Start_1();                                                                    //send the camera back to the start
}



//**********************************************Stop Motion*******************************************
void Start_7() {
  if (Sld == 0 && JB == 0) {                                                  //If no slider or jib take charge of the count
    if (stopM_active = 1) {                                                     // Is stopM active?
      SMC = SMC - 1;                                                            //Subtract frame
    } else {
      SMC = Tps;
      SMC = SMC - 1;
    }
    But_Com = 8;
    SendNextionValues();                                                    //Tell the nextion to update the stopmotion frame counter
  }
  TLTtravel_dist = (TLTout_position - TLTin_position);                           // Distance we have to go
  PANtravel_dist = (PANout_position - PANin_position);
  FOCtravel_dist = (FOCout_position - FOCin_position);
  //ZOOMtravel_dist = (ZOOMout_position - ZOOMin_position);                      //Ready for 4th axis

  TLTTlps_step_dist = (TLTtravel_dist / stopM_frames);
  PANTlps_step_dist = (PANtravel_dist / stopM_frames);
  FOCTlps_step_dist = (FOCtravel_dist / stopM_frames);
  //ZOOMTlps_step_dist = (FOCtravel_dist / stopM_frames);                        //Ready for 4th axis


  digitalWrite(StepD, LOW);


  TLTstep_speed = (TLTtravel_dist / 5);
  PANstep_speed = (PANtravel_dist / 5);
  FOCstep_speed = (FOCtravel_dist / 5);
  //ZOOMstep_speed = (ZOOMtravel_dist / 5);

  stepper1->setSpeedInUs(500);
  stepper1->setAcceleration(500);

  stepper2->setSpeedInUs(500);
  stepper2->setAcceleration(500);

  stepper3->setSpeedInUs(500);
  stepper3->setAcceleration(500);

  //stepper4->setSpeedInUs(500);
  //stepper4->setAcceleration(500);

  if (SMC >= 0  && stopM_cancel != 1) {                                           //Run the steppers

    stepper1->moveTo(stepper1->getCurrentPosition() + TLTTlps_step_dist);           //Current position plus one fps move
    delay(10);
    stepper2->moveTo(stepper2->getCurrentPosition() + PANTlps_step_dist);
    delay(10);
    stepper3->moveTo(stepper3->getCurrentPosition() + FOCTlps_step_dist);
    //delay(10);
    //stepper4 moveTo(stepper4->getCurrentPosition() + ZOOMTlps_step_dist);          //Ready for forth axis

    //delay until move complete. **Add stepper 4 if present
    while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning()) {
      delay(10);
    }
    delay (700);                                                                     //Take a second to ensure camera is still
    digitalWrite(CAM, HIGH);                                                         //Fire shutter
    delay (TpsD);                                                                    //Time the shutter is open for if on ball. If not on ball camera has control of shutter time and user must set this higher than shutter speed
    digitalWrite(CAM, LOW);                                                          //Cose the shutter and move on

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



  switch (ease_InOut) {
    case 0:                                                                //No ease
      TLTease_Value = 4000;
      PANease_Value = 4000;
      FOCease_Value = 4000;

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
  ResetEncoder = 1;
  return;
}

void ReadMount() {                                         //Listen for PanTilt mount buttons and execute
  stepper2->setSpeedInUs(150);                             //Larger value is slower!
  stepper2->setAcceleration(3000);

  if (digitalRead(Mount_PIN) == LOW) {
    delay(100); // Debounce the button
    while  (digitalRead(Mount_PIN) == LOW) {
      Mount = 1;
      stepper2-> runForward();
    }
  }
  if (digitalRead(Mount_PIN) == HIGH && Mount == 1) {
    stepper2->forceStopAndNewPosition(0);                  //Stops dead
    Mount = 0;
  }

  if (digitalRead(DisMount_PIN) == LOW) {
    delay(100); // Debounce the button
    while  (digitalRead(DisMount_PIN) == LOW) {
      Mount = 1;
      stepper2-> runBackward();
    }
  }

  if (digitalRead(DisMount_PIN) == HIGH && Mount == 1) {
    stepper2->forceStopAndNewPosition(0);                  //Stops dead
    Mount = 0;
  }
}

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
    digitalWrite(CAM, HIGH);                                              //Start/stop Record on camera. Switch must be flipped
    delay (200);
    digitalWrite(CAM, LOW);
    lastRecState = Rec;
  }

  //Joystick comands
  //***PAN***
  if (abs(Joy_Pan_Speed) > 150 ) {
    pan_is_moving = true;
    stepper2->setAcceleration(abs(Joy_Pan_Accel));                              //If starting fresh Max accel to catch up on turnarround
    stepper2->setSpeedInHz(abs(Joy_Pan_Speed / 4));
    if (Joy_Pan_Speed > 1) {
      stepper2->runForward();
    } else {
      stepper2->runBackward();
    }
  } else {
    if (pan_is_moving) {
      stepper2->setAcceleration(abs(Joy_Pan_Accel));                                         //Breaking acceloration on the stick moving back to 0
      stepper2->applySpeedAcceleration();
      stepper2->stopMove();
      pan_is_moving = false;
    }
  }
  //***TILT***
  if (abs(Joy_Tilt_Speed) > 250) {
    tilt_is_moving = true;
    stepper1->setAcceleration(abs(Joy_Tilt_Accel * 2));                             //If starting fresh Max accel to catch up on turnarround
    stepper1->setSpeedInHz(abs(Joy_Tilt_Speed / 2 ));
    if (Joy_Tilt_Speed > 1) {
      stepper1->runForward();
    } else {
      stepper1->runBackward();
    }
  } else {
    if (tilt_is_moving) {
      stepper1->setAcceleration(abs(Joy_Tilt_Accel * 2));                                         //Breaking acceloration on the stick moving back to 0
      stepper1->applySpeedAcceleration();
      stepper1->stopMove();
      tilt_is_moving = false;
    }
  }
  //***FOCUS***

  if (abs(Joy_Focus_Speed) > 1000) {
    focus_is_moving = true;
    stepper3->setAcceleration(abs(Joy_Focus_Accel * 3));                             //If starting fresh Max accel to catch up on turnarround
    stepper3->setSpeedInHz(abs(Joy_Focus_Speed / 2));



    // Forward Move
    if (Joy_Focus_Speed > 1000 && stepper3->getCurrentPosition() < cam_F_Out) {
      stepper3->setAcceleration(abs(Joy_Focus_Accel * 3));                             //If starting fresh Max accel to catch up on turnarround
      stepper3->setSpeedInHz(abs(Joy_Focus_Speed / 2));
      if (stepper3->getCurrentPosition() > (cam_F_Out - 75 ) && abs(Joy_Focus_Speed) > 1000) {
        if (focus_is_moving = true) {
          stepper3->moveTo(cam_F_Out);
        }

      } else {
        if (focus_is_moving = true) {
          stepper3->runForward();
        }
      }
      if (stepper3->getCurrentPosition() > (cam_F_Out)) {
        stepper3->forceStopAndNewPosition(cam_F_Out);
        focus_is_moving = false;
      }
    }

    //Backward move
    if (Joy_Focus_Speed < 1000 && stepper3->getCurrentPosition() > cam_F_In) {
      stepper3->setAcceleration(abs(Joy_Focus_Accel * 3));                             //If starting fresh Max accel to catch up on turnarround
      stepper3->setSpeedInHz(abs(Joy_Focus_Speed / 2));
      if (stepper3->getCurrentPosition() < (cam_F_In + 75 ) && abs(Joy_Focus_Speed) > 1000) {
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
      }
    }
  } else {                                                                          //No command but stepper is still moving
    if (focus_is_moving) {
      stepper3->setAcceleration(12000);                                              //Breaking acceloration on the stick moving back to 0
      stepper3->applySpeedAcceleration();
      stepper3->forceStopAndNewPosition(stepper3->getCurrentPosition());
      //stepper3->stopMove();
      focus_is_moving = false;
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

  SysMemory.begin("In_positions", false);                       //Recover the last In_Positions from memory before shutdown
  TLTin_position = SysMemory.getUInt("TLTin_position", 0);
  PANin_position = SysMemory.getUInt("PANin_position", 0);
  FOCin_position = SysMemory.getUInt("FOCin_position", 0);
  SysMemory.end();

  SysMemory.begin("Out_positions", false);                      //Recover the last In_Positions from memory before shutdown
  TLTout_position =  SysMemory.getUInt("TLTout_position", 0);
  PANout_position = SysMemory.getUInt("PANout_position", 0);
  FOCout_position = SysMemory.getUInt("FOCout_position", 0);
  SysMemory.end();

  SysMemory.begin("FocLmts", false);                           //Recover Focus Limits
  cam_F_In = SysMemory.getUInt("cam_F_In", 0);
  cam_F_Out = SysMemory.getUInt("cam_F_Out", 0);
  SysMemory.end();

  SysMemory.begin("P1pos", false);                        //Recover PTZ Cam poses
  P1_T = SysMemory.getUInt("P1_T", 10);
  P1_P = SysMemory.getUInt("P1_P", 10);
  P1_F = SysMemory.getUInt("P1_F", 10);
  SysMemory.end();

  SysMemory.begin("P2pos", false);
  P2_T = SysMemory.getUInt("P2_T", 10);
  P2_P = SysMemory.getUInt("P2_P", 10);
  P2_F = SysMemory.getUInt("P2_F", 10);
  SysMemory.end();

  SysMemory.begin("P3pos", false);
  P3_T = SysMemory.getUInt("P3_T", 10);
  P3_P = SysMemory.getUInt("P3_P", 10);
  P3_F = SysMemory.getUInt("P3_F", 10);
  SysMemory.end();

  SysMemory.begin("P4pos", false);
  P4_T = SysMemory.getUInt("P4_T", 10);
  P4_P = SysMemory.getUInt("P4_P", 10);
  P4_F = SysMemory.getUInt("P4_F", 10);
  SysMemory.end();

  SysMemory.begin("P5pos", false);
  P5_T = SysMemory.getUInt("P5_T", 10);
  P5_P = SysMemory.getUInt("P5_P", 10);
  P5_F = SysMemory.getUInt("P5_F", 10);
  SysMemory.end();

  SysMemory.begin("P6pos", false);
  P6_T = SysMemory.getUInt("P6_T", 10);
  P6_P = SysMemory.getUInt("P6_P", 10);
  P6_F = SysMemory.getUInt("P6_F", 10);
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
  SysMemory.putUInt("TLTin_position", TLTin_position);
  SysMemory.putUInt("PANin_position", PANin_position);
  SysMemory.putUInt("FOCin_position", FOCin_position);
  SysMemory.end();

  SysMemory.begin("Out_positions", false);
  TLTout_position = 0;
  PANout_position = 0;
  FOCout_position = 0;
  SysMemory.putUInt("TLTout_position", TLTout_position);
  SysMemory.putUInt("PANout_position", PANout_position);
  SysMemory.putUInt("FOCout_position", FOCout_position);
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
  SysMemory.putUInt("P1_T", P1_T);
  SysMemory.putUInt("P1_P", P1_P);
  SysMemory.putUInt("P1_F", P1_F);
  SysMemory.end();

  SysMemory.begin("P2_pos", false);
  P2_T = 10;
  P2_P = 10;
  P2_F = 10;
  SysMemory.putUInt("P2_T", P2_T);
  SysMemory.putUInt("P2_P", P2_P);
  SysMemory.putUInt("P2_F", P2_F);
  SysMemory.end();

  SysMemory.begin("P3_pos", false);
  P3_T = 10;
  P3_P = 10;
  P3_F = 10;
  SysMemory.putUInt("P3_T", P3_T);
  SysMemory.putUInt("P3_P", P3_P);
  SysMemory.putUInt("P3_F", P3_F);
  SysMemory.end();

  SysMemory.begin("P4_pos", false);
  P4_T = 10;
  P4_P = 10;
  P4_F = 10;
  SysMemory.putUInt("P4_T", P4_T);
  SysMemory.putUInt("P4_P", P4_P);
  SysMemory.putUInt("P4_F", P4_F);
  SysMemory.end();

  SysMemory.begin("P5_pos", false);
  P5_T = 10;
  P5_P = 10;
  P5_F = 10;
  SysMemory.putUInt("P5_T", P5_T);
  SysMemory.putUInt("P5_P", P5_P);
  SysMemory.putUInt("P5_F", P5_F);
  SysMemory.end();

  SysMemory.begin("P6_pos", false);
  P6_T = 10;
  P6_P = 10;
  P6_F = 10;
  SysMemory.putUInt("P6_T", P6_T);
  SysMemory.putUInt("P6_P", P6_P);
  SysMemory.putUInt("P6_F", P6_F);
  SysMemory.end();

  clrK = 0;

}





void Percent_100(void) {
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
