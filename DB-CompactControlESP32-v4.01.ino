//DBNextionESP32 V4.01
//* Digital Bird Motion Control System Nextion Remote firmware for Adafruit HUZZAH32-ESP32 Feather Board
//This software should be installed on the remote dispaly Adafruit HUZZAH32-ESP32.//
//Written by Colin Henderson April 2021


#include <esp_now.h>
#include <EasyNextionLibrary.h>
#include <trigger.h>
#include <WiFi.h>

#include <WiFiUdp.h>
#include <Preferences.h>
#include <Wire.h>
/*
  ESP32 uses Hardware serial RX:16, TX:17
  Serial pins are defined in the ESPNexUpload.cpp file
*/
//



//Variables
int Snd;
int MSeqTime;
int FPS = 10;
long tiltS;
long tiltA;
long panS;
long panA;
long focusS;
long focusA;
long zoomS;
long zoomA;
long slidS;
long slidA;
long masterA;

int stopMactive;
int tilt_AVG;
int pan_AVG;
int foc_AVG;
int zoom_AVG;
int slid_AVG;

String NexMenu;


int  ZoomJoyAccel;

int sendjoy;
int joypass;
int usejoy;

int PTZ_Cam = 4;                          //Current PTZ camera defaulted to 4 can be 1-3
int PTZ_ID = 0;                           //PTZ Hardwae ID defaulted to 0 not used by PTZ functions
int PTZ_Active;
int PTZ_SaveP;
int PTZ_Pose;
int LM;                                   //Focus/Zoom limits set
// Phisical recoed button
int RecordBut;                            //State of phisical record button on PTZ controler
int Cam0_RecordBut;
int Cam1_RecordBut;
int Cam2_RecordBut;
int Cam3_RecordBut;

int lastRecordBut;                        //Last recorded state of button
int lastCam1_RecordBut;
int lastCam2_RecordBut;
int lastCam3_RecordBut;

int Tilt_J_Speed;
int Pan_J_Speed;
int Foc_J_Speed;
int Zoom_J_Speed;
int Slid_J_Speed;

int SM;                                     //stopMotion play true or false
int SC;                                     //stopMotion cancel true or false
int SMC;                                    //Stop Motion counter
int SMC_last;

float NexBatV;
long Rail_Length = 500;                     //Edit this to use any rail length you require in mm
long Max_Steps = 0;                         //Variable used to set the max number of steps for the given leangth of rail
unsigned long previousMillis = 0;
long set_speed = 0;
long in_position = 0;                       // variable to hold IN position for slider
long out_position = 0;                      // variable to hold OUT position for slider
long travel_dist = 0;
long step_speed = 0;                        // default travel speed between IN and OUT points
int Crono_time = 10;
int crono_hours = 0;
int crono_minutes = 0;
int crono_seconds = 10;
String DBFirmwareVersion = "DBv4.01";
String nextion_message;                   // variable to hold received message for Nextion LCD
int move_left = 0;                        // variable to detect move left on Nextion LCD
int move_right = 0;                       // variable to confirm move right on Nextion LCD
int start_cycle = 0;                      // variable to confirm start of travel between IN and OUT points
long Home_position = 0;                   // Variable to st home position on slider
int Ease_Value = 0;                       // Variable to collect Ease_Value from nextion screen value range 1-3
int ease_time = 0;                        // Extra time allowed for ease in out calculation
int move_finished = 1;                    // Used to check if move is completed
long initial_homing = -1;                 // Used to Home Stepper at startup
int Bounce = 0;
int Tps = 0;
int Tps_countdown = 0;
int Tps_step_dist = 0;
int TpsD = 1000;
int Buttonset = 0;
int timelapse_projected = 0;
int getEncoder = 0;
int InP = 0;
int OutP = 0;
int clrK;                                 //Clear all keyframe memories back to 0
int P1;                                   //Sequencer button states
int P2;
int P3;
int P4;
int P5;
int P6;



int s1 = 50;
int s2 = 50;
int s3 = 50;
int s4 = 0;
int s5 = 0;
int s6 = 0;
int a1 = 0;
int a2 = 0;
int a3 = 0;
int a4 = 0;
int a5 = 0;
int a6 = 0;
int SE = 0;

int TiltPanComand = 0;
int playbuttoncounter = 1;
String dataTx;
String wifimessage;
int PanTiltPlay;
int PanTiltInPoint;
int PanTiltOutPoint;
int Nextion_Func;
int Nextion_play;
int But_Com;
int TpsM;

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

int Sld = 0;                                            //Is the Slider  present 1 or 0
int PT;                                                 //Is the Pan Tilt present 1 0 0
int TT;                                                 //Is the turntable present 1 or 0
int JB;                                                 //Is the Jib present 1 or 0
int mess;

//*******************************************************************************************************************
//Change these valus depending on which pan tilt head you havebuilt over the top or balanced
// These speed controls would be beter placed on rotary encoders one for each axis
//int Tilt_J_Speed = 200;                       //Set this to 150 for the Balanced PT 200 for the Over the Top PT
//int Pan_J_Speed = 4000;                        //Set this to 150 for the Balanced PT 4000 for the Over the Top PT
//int foc_J_Speed = 4000;
//int zoom_J_Speed = 4000;
//int slid_J_Speed = 4000;
//output for each axis requires two pices of information Speed and Accelorations this is 60 bits to many!


//********************************************************************************************************************


//Structure example to send data
//Must match the receiver structure
//Structure to send WIFI data. Must match the receiver structure
//Structure to send WIFI data. Must match the receiver structure

//Structure to send WIFI data. Must match the receiver structure currently 136bytes of 250 max fo ESP-Now
typedef struct struct_message {
  int Snd;                                             //1= Controller 2=slider, 3=Pan Tilt 4=Turntable 5=Jib
  int But;                                             //But_Com Nextion button press holder
  int Jb;                                              //Joystick button state true or false
  //Joystick controler peramiters
  int Ts;                                              //Tilt speed
  int Ps;
  int Fs;
  int Zs;
  int Ss;
  int Ma;                                              //Master acceloration pot
  // PTZ peramiters
  int ID;                                              //Hardware ID for PTZ
  int CA;                                              //Camera selection for PTZ
  int SP;                                              //PTZ_SaveP PTZ save pose
  int Pz;                                              //PTZ_Pose PTZ current pose
  int Rc;                                              //Record button state for PTZ
  int LM;                                              //Focus/zoom limits set
  // stopMotion peramiters
  int SM;                                              //stopMotion play true or false
  int SC;                                              //stopMotion cancel
  int SMC;                                             //Stop motion count

  int Ez;                                               //Ease Value
  int Bo;                                               //Number of bounces
  int TT;                                               //Is the turntable present 1 or 0
  int PT;                                               //Is the turntable present 1 or 0 2 PanTilt Bounce trigger prevents cumulative speed errors
  int Sld;                                              //Is the Slider  present 1 or 0
  int JB;                                               //Is the Jib  present 1 or 0
  long In;                                              //Slider In position in steps
  int P1;                                               //Key button preses
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
  int Cro;                                                //Time in seconds of move
  //int Fnc;                                              //Nection Function
  int Tps;                                                //Timlapse frames required
  long Out;                                               //Slider Out position in steps

  int InP;                                                //Inpoint button press
  int clrK;                                                 //Tell all devices currently active to clear key memories value 1 or 0
  int OutP;                                               //Out point button press
  int TpsD;                                               //Timlapse Delay in seconds
  int TpsM;                                               //Timlapse shutter time in seconds
  int Play;                                               //Play button press
  //int mess;
} struct_message;

struct_message NextionValues;     // Create a struct_message to Hold outgoing values
struct_message incomingValues;    // Create a struct_message to hold incoming values from TiltPan and Turntable

int WIFIOUT[8];                   //set up 5 element array for sendinf values to pantilt
//*********************************Use thit TX RX for Feather bord
#define RXD2 16                   //Hardware Serial2 on ESP32 Dev (must also be a common earth between nextion and esp32)should be gpio 3 for lolin32
#define TXD2 17                   //Should be gpio 1 for Lolin32 if using Seril1
//**********************************For LOLIN32 comment out the two lines above and uncomment the two lines bellow
//#define RXD2 3                   //Hardware Serial2 on ESP32 Dev (must also be a common earth between nextion and esp32)should be gpio 3 for lolin32
//#define TXD2 1                   //Should be gpio 1 for Lolin32 if using Seril1

EasyNex myNex(Serial2);           // Create an object of EasyNex class with the name < myNex >
#define Nextion_Dim_Time 120
hw_timer_t * timer = NULL;


void IRAM_ATTR Nextion_Dim() {
  String dims = "dims=" + String(5);  // set display brightness to 100%
  myNex.writeStr(dims.c_str());

}
void Nextion_Wake() {
  String dims = "dims=" + String(100);  // set display brightness to 100%
  myNex.writeStr(dims.c_str());
}

//Joystick control
const int panS_PIN = 0;                    // Pan Joy
const int tiltS_PIN = 0;                   // Tilt Joy
const int slidS_PIN = 0;                   // Slider joy
const int focusS_PIN = 0;                  // Focus joy
const int zoomS_PIN = 0;                   // Zoom joy
const int masterA_PIN = 0;                 // Master pot
const int NexBat = 35;                   //Battery monitoring pin
//PTZ Play button
#define CAM 0                             // Pin Gpio 4 Camera trigger
#define LED 0                              // Pin forplay LED

byte lastLEDstate = LOW;
byte ledState = LOW;
//unsigned long debounceDuration = 50; // millis
unsigned long prevTime = 0;

//Joystick button
#define JBT 0                              //Joystick button





void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("\nRunning DigitalBird Nextion ESP32 controler\n");

  //Start Nextion Dim Timer
  timer = timerBegin(0, 80, true);                                  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &Nextion_Dim, true);                  // edge (not level) triggered
  timerAlarmWrite(timer, (Nextion_Dim_Time * 1000000), true);       // 1000000 * 1 us = 1 s, autoreload true
  timerAlarmEnable(timer);                                          // enable

  //**********************Prepair Nextion***********************************//
  String dims = "dims=" + String(100);  // set display brightness to 100%
  myNex.writeStr(dims.c_str());


  myNex.writeStr("t9.txt", String(DBFirmwareVersion));
  delay(100);
  myNex.writeStr("t2.txt", "10");                                 //set seconds deay at 2

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


  //pinMode(NexBat, INPUT);

  NexBatV = analogRead(NexBat);
  NexBatV = ((NexBatV / 4095) * 2) * 3.3;

  pinMode(JBT, INPUT_PULLUP);                           //Joystick button


  pinMode(CAM, INPUT_PULLUP);                           //Camera switch pulled high
  pinMode(LED, OUTPUT);
  InitialValues();                                              //onboard Joystick calobration

  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");


  pinMode(masterA_PIN, INPUT);


  String NexMenu = myNex.readStr("menu.txt");
  Serial.printf("\nNextion Menu is currently %d \n", NexMenu.toInt());


}








void loop() {

  NexBatV = analogRead(NexBat);
  NexBatV = ((NexBatV / 4095) * 2) * 3.3;
  //  if (PTZ_Active || usejoy) {
  //    ReadAnalog(); //Joystick setup
  //  }

  //*************************************Joystick top button press Function*******************************************
  //  if (digitalRead(JBT) == LOW) {
  //    long unsigned int currTime = millis();
  //    if (currTime - prevTime > 50) {
  //      if (usejoy) {                               //If usejoy is on
  //        myNex.writeNum("b0.pic", 43);             //Turn it off
  //        usejoy = 0;
  //      } else {                                    //else turn it on
  //        myNex.writeNum("b0.pic", 44);
  //        usejoy = 1;
  //      }
  //
  //      prevTime = currTime;
  //      delay(200);
  //    }
  //  }

  //*************************************PTZ Record button log*******************************************
  //  if (digitalRead(CAM) == LOW) {
  //    long unsigned int currTime = millis();
  //    if (currTime - prevTime > 50)
  //      digitalWrite(LED, !digitalRead(LED));
  //    prevTime = currTime;
  //    if (digitalRead(LED) == LOW) {
  //
  //      switch (PTZ_Cam) {
  //        case 1:
  //          Cam1_RecordBut = 0;
  //          break;
  //        case 2:
  //          Cam2_RecordBut = 0;
  //          break;
  //        case 3:
  //          Cam3_RecordBut = 0;
  //          break;
  //      }
  //      RecordBut = 0;                       //For any other  Play command
  //    } else {
  //      //Serial.println("Play ON");           //Turn on the camera
  //      switch (PTZ_Cam) {
  //        case 1:
  //          Cam1_RecordBut = 1;
  //          break;
  //        case 2:
  //          Cam2_RecordBut = 1;
  //          break;
  //        case 3:
  //          Cam3_RecordBut = 1;
  //          break;
  //      }
  //      RecordBut = 1;                       //For any other Play command
  //    }
  //  } else {                                 //Button not in depressed state ON
  //
  //    if (PTZ_Active == 0 && (RecordBut != lastRecordBut)) {  //If PTZ is not active
  //      String NexMenu = myNex.readStr("menu.txt");
  //      switch (NexMenu.toInt()) {
  //        case 1:                                             //Nextion menu is home
  //          But_Com = 1;                                      //If we are in the home menu Send the home screen play command
  //          lastRecordBut = RecordBut;
  //          trigger9();                                       //Home Play button press
  //          break;
  //        case 2:                                             //Nextion menu is sequencer
  //          Nextion_play = 2;
  //          SendNextionValues();
  //          delay(200);
  //          Nextion_play = 0;                                 //Reset the button
  //          lastRecordBut = RecordBut;
  //          myNex.writeNum("b5.pic", 7);                     //Turn play red
  //          break;
  //        case 6:                                             //Nextion menu is stopmotion
  //          trigger39();
  //          //          SM = 1;
  //          //          SendNextionValues();
  //          //          delay(200);
  //          //          SM = 0;
  //          lastRecordBut = RecordBut;
  //          myNex.writeNum("Play.pic", 7);                     //Turn play red
  //          break;
  //      }
  //    }
  //
  //    if (PTZ_Active) {                                     //If PTZ is active
  //      switch (PTZ_Cam) {
  //        case 1:
  //          if (Cam1_RecordBut != lastCam1_RecordBut) {
  //            SendNextionValues();
  //            lastCam1_RecordBut = Cam1_RecordBut;
  //          }
  //          break;
  //        case 2:
  //          if (Cam2_RecordBut != lastCam2_RecordBut) {
  //            SendNextionValues();
  //            lastCam2_RecordBut = Cam2_RecordBut;
  //          }
  //          break;
  //        case 3:
  //          if (Cam3_RecordBut != lastCam3_RecordBut) {
  //            SendNextionValues();
  //            lastCam3_RecordBut = Cam3_RecordBut;
  //          }
  //          break;
  //      }
  //    }
  //  }


  //*************************************Listen for Nextion LCD comands*******************************************
  myNex.NextionListen();   // nextion_message.NextionListen(); //check for message from Nextion touchscreen


  //*********************StopMotion Canceled***********************************
  if (SC == 1) {
    myNex.writeStr("SMC_counter.txt", "CANCEL");        //Move canceled
    delay(1000);
    myNex.writeStr("SMC_counter.txt", String(Tps));        //Send counter back to start
    delay(20);
    SMC = Tps;
    SC = 0;
    SendNextionValues();
    delay(20);
  }

  //*************************************Listen for slave device button comands*****************************************
  if (But_Com != 0) {                                   //If a button comand has been received from a slave

    switch (But_Com) {

      case 1:                                            //Play request from slave
        Nextion_play = 1;
        But_Com = 0;                                     //Reset the Slave button request
        SendNextionValues();                             //Send out the play command
        Nextion_play = 0;                                //Reset play
        //digitalWrite(LED, LOW);
        lastRecordBut = 0;
        RecordBut = 0;
        break;
      case 2:                                            //InPoint set request from slave
        trigger7();                                      //Same as if pressed on nextion
        But_Com = 0;
        break;
      case 3:                                            //OutPoint set request from slave
        trigger8();                                      //Same as if pressed on nextion
        But_Com = 0;
        break;
      case 4:
        //delay(5);
        myNex.writeNum("rewind.val", 0);                 //zero the nextion timer
        delay(20);
        myNex.writeNum("TimerStop.val", 1);              // timer Start
        But_Com = 0;
        break;
      case 5:
        myNex.writeNum("TimerStop.val", 0);                  //Stop nextion timer
        delay(100);
        digitalWrite(LED, LOW);                              //Turn off the button LED
        myNex.writeNum("Play.pic", 8);                     //Turn play yellow
        delay(50);
        myNex.writeNum("b5.pic", 8);                     //Turn play yellow
        But_Com = 0;
        delay(1000);
      case 6:
        delay(5);
        myNex.writeStr("bounce.txt", String(Bounce));        //update Bounce counter main
        delay(5);
        myNex.writeStr("Bounce.txt", String(Bounce));        //update Bounce counter SEQ
        But_Com = 0;
        break;
      case 7:
        delay(5);
        myNex.writeStr("tlps.txt", String(Tps));            //update timlapse counter
        delay(5);
        if (Tps == 0) {
          crono_seconds = 10;
          Crono_time = 10;
          SendNextionValues();
          delay(5);
          myNex.writeStr("sec.txt", String(crono_seconds));
          But_Com = 0;
        }
        But_Com = 0;
        break;
      case 8:                                                           //Stop Motion counter update
        delay(30);
        if (SMC != 0) {
          myNex.writeStr("SMC_counter.txt", String(SMC));              //Update Nextion frame counter
          delay(50);
          //digitalWrite(LED, LOW);                                     //Turn OFF the button LED
          myNex.writeNum("Play.pic", 8);                              //Turn play yellow
          delay(50);
        } else {
          //digitalWrite(LED, LOW);                                     //Turn OFF the button LED
          myNex.writeStr("SMC_counter.txt", "Complete");        //Update Nextion frame counter
          delay(10);
          myNex.writeStr("SMC_counter.txt", String(Tps));        //Send counter back to start
          delay(20);
        }
        But_Com = 0;
        break;
    }
  }


} //end loop




//***************************Nextion Button Press Functions*****************************//

void trigger1() {                              //Nextion m0 Hours
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 0;
  SendNextionValues();
  UpdateSEQ();
}
void trigger2() {                             //Nextion m1 Min
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 1;
  SendNextionValues();
}

void trigger3() {                             // Nextion m2 Sec
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 2;
  SendNextionValues();
}

void trigger4() {                             //Nextion m3 Ease
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 3;
  SendNextionValues();
}

void trigger5() {                             //Nextion m4 Bounce
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 4;
  SendNextionValues();
}

void trigger6() {                            //Nextion m5 Tps
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Func = 5;
  SendNextionValues();
}

void trigger7() {                            //Nextion m6 InPoint
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 6;
  ++InP;
  switch (InP) {
    case 1:                                  //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                  //Second Press: Record position
      SendNextionValues();
      InP = 0;                               //Reset InPoint press back to 0
      break;
  }
}

void trigger8() {                            // Nextion m7 OutPoint

  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 7;
  ++OutP;
  switch (OutP) {
    case 1:                                   //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                   //Second Press: Record position
      SendNextionValues();
      OutP = 0;                               //Reset InPoint press back to 0
      break;
  }
}

void trigger9() {                                                              // Nextion m7 Play
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  if (stopMactive == 1) {
    stopMactive = 0;
    myNex.writeStr("tlps.txt", "0");
    Tps = 0;
  }

  Nextion_Wake();
  Nextion_play = 1;
  SendNextionValues();
  digitalWrite(LED, HIGH);                                                      //Turn on LED
  myNex.writeNum("Play.pic", 7);                                               //Turn play red
  delay(200);
  Nextion_play = 0;                                                            //Reset the button
}


void trigger10() {                                                               // Nextion Up arrow
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  switch (Nextion_Func) {

    case 0:                                                                      // + time Hours on nextion
      if (crono_hours >= 0 && crono_hours <= 23) {                               //limit hours that can be set between 0 & 24
        crono_hours = crono_hours + 1;                                           // increase by 1
        myNex.writeStr("hours.txt", String(crono_hours));
        Crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
      }
      break;
    case 1:                                                                      // + time Minuts on nextion
      if (crono_minutes >= 0 && crono_hours <= 59) {                             //limit Minuits that can be set between 0 & 60
        crono_minutes = crono_minutes + 1;                                       // increase by 1
        myNex.writeStr("min.txt", String(crono_minutes));
        Crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
      }
      break;
    case 2:                                                                      // + time Seconds on nextion
      if (crono_seconds >= 0 && crono_seconds <= 59) {                           //limit seconds that can be set between 0 & 60
        crono_seconds = crono_seconds + 1;                                       // increase by 1
        myNex.writeStr("sec.txt", String(crono_seconds));
        Crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
      }
      if (Tps > 0) {
        TpsD = (crono_seconds * 1000);
      }
      break;
    case 3:                                                                     // + Ease value
      if (Ease_Value >= 0 && Ease_Value <= 2) {                                 //limit Ease that can be set between 0 & 3
        Ease_Value = Ease_Value + 1;                                            // increase  by 1
        myNex.writeStr("ease.txt", String(Ease_Value));
      }
      break;
    case 4:                                                                     //+Bounce Value Func
      if (Bounce <= 10 && Bounce != 500) {
        Bounce = Bounce + 1;                                                    // increase  by 1
        myNex.writeStr("bounce.txt", String(Bounce));                           // display value in t4 Bounce box on Nextion
        myNex.writeStr("bounce.txt", String(Bounce));
      } else {
        Bounce = 500;                                                           //If greater than ten assume infinity 500 bring it down again to 10
        myNex.writeStr("bounce.txt", "Inf");
        myNex.writeStr("bounce.txt", "Inf");
      }
      break;
    case 5:                                                                     // + Tps on nextion
      if (Tps >= 0 ) {
        Tps = Tps + 25;                                                         // increase Tps by 25
        myNex.writeStr("tlps.txt", String(Tps));                                // display seconds in t2 box on Nextion
        if (Tps != 0) {
          timelapse_projected = (5 + (Tps * 2.501));
          crono_seconds = 2;
          myNex.writeStr("sec.txt", "2");   //set seconds deay at 2
          myNex.writeStr("min.txt", "0");   //set min at 0
          myNex.writeStr("hours.txt", "0");   //set hours at 0
          myNex.writeStr("t8.txt", String(timelapse_projected));
        }
      }
      break;
    case 20:
      MSeqTime++;                                                          //Add a second to SequenceTime
      myNex.writeStr("SeqTime.txt", String(MSeqTime));                     //Update Nextion
      Tps = (MSeqTime * FPS);
      myNex.writeStr("SMC_counter.txt", String(Tps));
      delay(20);
      break;
    case 21:
      TpsD = (TpsD + 1000);
      myNex.writeStr("shutterT.txt", String(TpsD / 1000));                  //Update Nextion
      break;
    case 22:
      FPS++;
      myNex.writeStr("fps.txt", String(FPS));                               //Motion control FPS add
      Tps = (MSeqTime * FPS);
      myNex.writeStr("SMC_counter.txt", String(Tps));
      delay(20);

      break;
  }
  SC = 1;
  PanTiltPlay = 1;
  SendNextionValues();
  SC = 0;
}

void trigger11() {                                                         // Nextion Down arrow
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  switch (Nextion_Func) {

    case 0:                                                                    // + time Hours on nextion
      if (crono_hours >= 1) {                                                   //limit hours that can be set between 0 & 24
        crono_hours = crono_hours - 1;                                          // increase by 1
        myNex.writeStr("hours.txt", String(crono_hours));
        Crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
      }
      break;
    case 1:                                                                     // - time Minuts on nextion
      if (crono_minutes >= 1) {                                                 //limit Minuits that can be set between 0 & 60
        crono_minutes = crono_minutes - 1;
        myNex.writeStr("min.txt", String(crono_minutes));
        Crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
      }
      break;
    case 2:                                                                    // + time Seconds on nextion
      if (crono_seconds >= 1) {                                                 //limit seconds that can be set between 0 & 60
        crono_seconds = crono_seconds - 1;
        myNex.writeStr("sec.txt", String(crono_seconds));
        Crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
      }
      break;
    case 3:                                                                      // + Ease value
      if (Ease_Value >= 1) {                                                    //limit Ease that can be set between 0 & 3
        Ease_Value = Ease_Value - 1;                                            // increase  by 1
        myNex.writeStr("ease.txt", String(Ease_Value));
      }
      break;
    case 4:                                                                     //-Bounce Value Func
      if (Bounce <= 10 && Bounce != 500 && Bounce != 0) {
        Bounce = Bounce - 1;
        myNex.writeStr("bounce.txt", String(Bounce));                           // display value in t4 Bounce box on Nextion
        myNex.writeStr("bounce.txt", String(Bounce));
      } else {
        Bounce = 10;                                                            //If greater than ten assume infinity 500 bring it down again to 10
        myNex.writeStr("bounce.txt", String(Bounce));
        myNex.writeStr("bounce.txt", String(Bounce));
      }
      break;
    case 5:                                                                    // + Tps on nextion
      if (Tps > 0 ) {
        Tps = Tps - 25;                                                        // increase Tps by 25
        myNex.writeStr("tlps.txt", String(Tps));                               // display seconds in t2 box on Nextion
        if (Tps == 0) {
          timelapse_projected = 0;
          crono_seconds = 10;
          Crono_time = 10;
          myNex.writeStr("t8.txt", String(timelapse_projected));
          myNex.writeStr("sec.txt", "10");   //set seconds deay at 2
          myNex.writeStr("min.txt", "0");   //set min at 0
          myNex.writeStr("hours.txt", "0");   //set hours at 0

        }
      }
      break;

    case 20:
      if (MSeqTime >= 1) {
        MSeqTime = (MSeqTime - 1);                                      //Subtract a second from SequenceTime
        myNex.writeStr("SeqTime.txt", String(MSeqTime));                //Update Nextion
        Tps = (MSeqTime * FPS);
        myNex.writeStr("SMC_counter.txt", String(Tps));
        delay(20);

      }
      break;
    case 21:
      if (TpsD >= 1000) {
        TpsD = (TpsD - 1000);
        myNex.writeStr("shutterT.txt", String(TpsD / 1000));         //Update Nextion
      }
      break;
    case 22:
      FPS = FPS - 1;
      myNex.writeStr("fps.txt", String(FPS));                       //Motion control FPS add
      Tps = (MSeqTime * FPS);
      myNex.writeStr("SMC_counter.txt", String(Tps));
      delay(20);

      break;
  }
  SC = 1;
  PanTiltPlay = 1;
  SendNextionValues();
  SC = 0;
}
//********************************************sequencer************************************************************
void trigger12() {                            //Nextion K1 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 12;
  ++P1;
  switch (P1) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      P1 = 0;                             //Reset InPoint press back to 0
      break;
  }
}
void trigger13() {                            //Nextion K2 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 13;
  ++P2;
  switch (P2) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      P2 = 0;                             //Reset InPoint press back to 0
      break;
  }
}
void trigger14() {                            //Nextion K3 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 14;
  ++P3;
  switch (P3) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      P3 = 0;                               //Reset InPoint press back to 0
      break;
  }
}
void trigger15() {                         //Nextion K4 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 15;
  ++P4;
  switch (P4) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      P4 = 0;                               //Reset InPoint press back to 0
      break;
  }
}
void trigger16() {                            //Nextion K5 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 16;
  ++P5;
  switch (P5) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      P5 = 0;                              //Reset InPoint press back to 0
      break;
  }
}
void trigger17() {                            //Nextion K6 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 17;
  ++P6;
  switch (P6) {
    case 1:                                //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                //Second Press: Record position
      SendNextionValues();
      P6 = 0;                             //Reset InPoint press back to 0
      break;
  }
}
void trigger18() {                         //Nextion Clear key
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 18;
  String k4String = myNex.readStr("k4t.txt");
  if (k4String.toInt() == 0) {
    s4 = 0;
    s5 = 0;
    s6 = 0;
    P4 = 0;
    P5 = 0;
    P6 = 0;
    SendNextionValues();
  } else {
    String k5String = myNex.readStr("k5t.txt");
    if (k5String.toInt() == 0) {
      s5 = 0;
      s6 = 0;
      P5 = 0;
      P6 = 0;
      SendNextionValues();
    }
    else {
      String k6String = myNex.readStr("k5t.txt");
      if (k6String.toInt() == 0) {
        s6 = 0;
        P6 = 0;
        SendNextionValues();
      }
    }
  }
}
void trigger19() {                                                             //Nextion sequencer play
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 19;
  Nextion_play = 2;
  SendNextionValues();
  delay(200);
  digitalWrite(LED, HIGH);                                                   //Turn on LED
  myNex.writeNum("b5.pic", 7);                                                //Turn play red
  Nextion_play = 0;                                                          //Reset the button
}

void trigger20() {                                                            // Sequencer Up arrow
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();

  switch (Nextion_Func) {
    case 4:                                                                   //+Bounce Value Func
      if (Bounce <= 10 && Bounce != 500) {
        Bounce = Bounce + 1;                                                  // increase  by 1
        myNex.writeStr("t4.txt", String(Bounce));                             // display value in t4 Bounce box on Nextion
        myNex.writeStr("Bounce.txt", String(Bounce));
      } else {
        Bounce = 500;                                                          //If greater than ten assume infinity 500 bring it down again to 10
        myNex.writeStr("t4.txt", "Inf");
        myNex.writeStr("Bounce.txt", "Inf");
      }
      break;
    case 12:                                                                  //K1 Set
      if (SE == 0) {                                                          //If speed selected
        if (s1 <= 95) {
          s1 = s1 + 5;
          myNex.writeStr("k1t.txt", String(s1));
        }
      } else {
        if (a1 <= 2) {
          a1 = a1 + 1;
          myNex.writeStr("a1.txt", String(a1));
        }
      }

      break;
    case 13:                                                                  //K2 Set
      if (SE == 0) {                                                          //If speed selected
        if (s2 <= 95) {
          s2 = s2 + 5;
          myNex.writeStr("k2t.txt", String(s2));
        }
      } else {
        if (a2 <= 2) {
          a2 = a2 + 1;
          myNex.writeStr("a2.txt", String(a2));
        }
      }

      break;
    case 14:                                                                  //K3 Set
      if (SE == 0) {                                                          //If speed selected
        if (s3 <= 95) {
          s3 = s3 + 5;
          myNex.writeStr("k3t.txt", String(s3));
        }
      } else {
        if (a3 <= 2) {
          a3 = a3 + 1;
          myNex.writeStr("a3.txt", String(a3));
        }
      }

      break;
    case 15:                                                                  //K4 Set
      if (SE == 0) {                                                          //If speed selected
        if (s4 <= 95) {
          s4 = s4 + 5;
          myNex.writeStr("k4t.txt", String(s4));
        }
      } else {
        if (a4 <= 2) {
          a4 = a4 + 1;
          myNex.writeStr("a4.txt", String(a4));
        }
      }

      break;
    case 16:                                                                  //K5 Set
      if (SE == 0) {                                                          //If speed selected
        if (s5 <= 95) {
          s5 = s5 + 5;
          myNex.writeStr("k5t.txt", String(s5));
          break;
        }
      } else {
        if (a5 <= 2) {
          a5 = a5 + 1;
          myNex.writeStr("a5.txt", String(a5));
          break;
        }
      }

    case 17:                                                                  //K6 Set
      if (SE == 0) {                                                          //If speed selected
        if (s6 <= 95) {
          s6 = s6 + 5;
          myNex.writeStr("k6t.txt", String(s6));
        }
      } else {
        if (a6 <= 2) {
          a6 = a6 + 1;
          myNex.writeStr("a6.txt", String(a6));
        }
      }

      break;
  }

  SendNextionValues();
}
void trigger21() {                                                            // Sequencer Down arrow
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  switch (Nextion_Func) {

    case 4:
      if (Bounce <= 10 && Bounce != 500 && Bounce != 0) {
        Bounce = Bounce - 1;                                                  // increase  by 1
        myNex.writeStr("t4.txt", String(Bounce));                             // display value in t4 Bounce box on Nextion
        myNex.writeStr("Bounce.txt", String(Bounce));
      } else {
        Bounce = 10;                                                          //If greater than ten assume infinity 500 bring it down again to 10
        myNex.writeStr("t4.txt", String(Bounce));
        myNex.writeStr("Bounce.txt", String(Bounce));
      }
      break;

    case 12:                                                                  //K1 Set
      if (SE == 0) {                                                          //If speed selected
        if (s1 >= 5) {
          s1 = s1 - 5;
          myNex.writeStr("k1t.txt", String(s1));
        }
      } else {
        if (a1 > 0) {
          a1 = a1 - 1;
          myNex.writeStr("a1.txt", String(a1));
        }
      }

      break;
    case 13:                                                                  //K2 Set
      if (SE == 0) {                                                          //If speed selected
        if (s2 >= 5) {
          s2 = s2 - 5;
          myNex.writeStr("k2t.txt", String(s2));
        }
      } else {
        if (a2 > 0) {
          a2 = a2 - 1;
          myNex.writeStr("a2.txt", String(a2));
        }
      }

      break;
    case 14:                                                                  //K3 Set
      if (SE == 0) {                                                          //If speed selected
        if (s3 >= 5) {
          s3 = s3 - 5;
          myNex.writeStr("k3t.txt", String(s3));
        }
      } else {
        if (a3 > 0) {
          a3 = a3 - 1;
          myNex.writeStr("a3.txt", String(a3));
        }
      }

      break;
    case 15:                                                                  //K4 Set
      if (SE == 0) {                                                          //If speed selected
        if (s4 >= 5) {
          s4 = s4 - 5;
          myNex.writeStr("k4t.txt", String(s4));
        }
      } else {
        if (a4 > 0) {
          a4 = a4 - 1;
          myNex.writeStr("a4.txt", String(a4));
        }
      }

      break;
    case 16:                                                                  //K5 Set
      if (SE == 0) {                                                          //If speed selected
        if (s5 >= 5) {
          s5 = s5 - 5;
          myNex.writeStr("k5t.txt", String(s5));
        }
      } else {
        if (a5 > 0) {
          a5 = a5 - 1;
          myNex.writeStr("a5.txt", String(a5));
        }
      }

      break;
    case 17:                                                                  //K6 Set
      if (SE == 0) {                                                          //If speed selected
        if (s6 >= 5) {
          s6 = s6 - 5;
          myNex.writeStr("k6t.txt", String(s6));
        }
      } else {
        if (a6 > 0) {
          a6 = a6 - 1;
          myNex.writeStr("a6.txt", String(a6));
        }
      }

      break;
  }
  SendNextionValues();
}

void trigger22() {                                                            //Sequencwer Ease setup arrow keys flip between speed and Easy value for each frame
  if (SE == 0) {                                                              // 0=speed
    SE = 1;
  } else {                                                                    //1=Acceloration or ease
    SE = 0;
  }
}

void trigger23() {                                                            // Sequencer add key
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  delay(20);
  String k4String = myNex.readStr("k4t.txt");
  s4 = k4String.toInt();
  String k5String = myNex.readStr("k5t.txt");
  s5 = k5String.toInt();
  String k6String = myNex.readStr("k6t.txt");
  s6 = k6String.toInt();
  SendNextionValues();
}

//***********************PTZ CAMERA SETUPS*******************************************************************//
void trigger24() {                            //PTZ cam1
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_Cam = 1;

  if ( Cam1_RecordBut == 1) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
  SendNextionValues();
}

void trigger25() {                            //PTZ cam2
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_Cam = 2;

  if ( Cam2_RecordBut == 1) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
  SendNextionValues();
}
void trigger26() {                            //PTZ cam3
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_Cam = 3;

  if ( Cam3_RecordBut == 1) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
  SendNextionValues();
}

void trigger27() {                                                  //PTZ Pos1 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_Pose = 1;
  Serial.printf("\nPTZ_Pose set to  %d \n", PTZ_Pose);
  SendNextionValues();
  delay(20);
  PTZ_Pose = 0;
}

void trigger28() {                                                  //PTZ Pos2 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_Pose = 2;
  Serial.printf("\nPTZ_Pose set to  %d \n", PTZ_Pose);
  SendNextionValues();
  delay(20);
  PTZ_Pose = 0;
}

void trigger29() {                                                  //PTZ Pos3 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_Pose = 3;
  Serial.printf("\nPTZ_Pose set to  %d \n", PTZ_Pose);
  SendNextionValues();
  delay(20);
  PTZ_Pose = 0;
}

void trigger30() {                                                  //PTZ Pos4 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_Pose = 4;
  Serial.printf("\nPTZ_Pose set to  %d \n", PTZ_Pose);
  SendNextionValues();
  delay(20);
  PTZ_Pose = 0;
}

void trigger31() {                                                  //PTZ Pos5 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_Pose = 5;
  Serial.printf("\nPTZ_Pose set to  %d \n", PTZ_Pose);
  SendNextionValues();
  delay(20);
  PTZ_Pose = 0;
}

void trigger32() {                                                  //PTZ Pos6 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_Pose = 6;
  Serial.printf("\nPTZ_Pose set to  %d \n", PTZ_Pose);
  SendNextionValues();
  delay(20);
  PTZ_Pose = 0;
}
//*******************************************************SET HARDWARE CAMERA ID*************************//
void trigger33() {                                                  //PTZ cam1 ID set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_ID = 1;
  SendNextionValues();
  delay(20);
  PTZ_ID = 0;
}

void trigger34() {                                                  //PTZ cam2 ID set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_ID = 2;
  SendNextionValues();
  PTZ_ID = 0;
}

void trigger35() {                                                  //PTZ cam3 ID set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_ID = 3;
  SendNextionValues();
  PTZ_ID = 0;
}

void trigger36() {                                                  //PTZ Active true
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();

  PTZ_Active = 1;

  SendNextionValues();
}

void trigger37() {                                                  //Home button
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_Active = 0;
  PTZ_Cam = 4;
  digitalWrite(LED, LOW);
  lastRecordBut = 0;
  RecordBut = 0;
  SendNextionValues();
}

void trigger38() {                                                  //PTZ Save Pose
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_SaveP = 1;
  SendNextionValues();
  PTZ_SaveP = 0;
}

void trigger39() {                                                  //Stop motion Play command
  Nextion_Wake();
  digitalWrite(LED, HIGH);
  SM = 1;
  SendNextionValues();
  delay(50);
  SM = 0;
}

void trigger40() {                                                  //Stop motion cancel command
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  SC = 1;
  SendNextionValues();

}
void trigger41() {                                                  //Nextion Stop Motion SeqTime
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Func = 20;

}

void trigger42() {                                                  // Stop Motion Shutter open time
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 21;
}

void trigger43() {                                                  // Stop Motion FPS 2B
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  Nextion_Func = 22;

}

void trigger44() {                                                  // Clear all keys back to 0 2C
  //Clear all keyframes back to 0
  clrK = 1;
  SendNextionValues();
  clrK = 0;
}

void trigger45() {                                                   //Set inital values for Stopmotion

  // TpsD = 1000;
  SendNextionValues();
  stopMactive = 1;
}

void trigger46() {                                                   //clockwise focus/Zoom limit set- hex 2E
  switch (PTZ_Cam) {
    case 1:
      LM = 1;
      SendNextionValues();
      Serial.println("\nLM set to 1:\n");
      break;
    case 2:
      LM = 3;
      SendNextionValues();
      Serial.println("\nLM set to 3:\n");
      break;
    case 3:
      LM = 5;
      SendNextionValues();
      Serial.println("\nLM set to 5:\n");
      break;
  }
}
void trigger47() {                                                   //counter clockwise focus/Zoom limit set - hex 2F
  switch (PTZ_Cam) {
    case 1:
      LM = 2;
      SendNextionValues();
      Serial.println("\nLM set to 2:\n");
      break;
    case 2:
      LM = 4;
      SendNextionValues();
      Serial.println("\nLM set to 4:\n");
      break;
    case 3:
      LM = 6;
      SendNextionValues();
      Serial.println("\nLM set to 6:\n");
      break;
  }
}
void trigger48() {                                                   //Finished - hex 30
  LM = 0;
  SendNextionValues();
  Serial.println("\nLM set to: 0\n");
}

void trigger49() {                                                   //Camera 0 set ID hex 31
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");
  Nextion_Wake();
  PTZ_ID = 4;
  SendNextionValues();
  PTZ_ID = 0;
}

void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength)
{
  snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}



//*************************************ESP_NOW READ INCOMMING VALUES*************************//
void receiveCallback(const uint8_t *macAddr, const uint8_t *incomingData,  int Len) {
  // only allow a maximum of 250 characters in the message + a null terminating byte
  char buffer[ESP_NOW_MAX_DATA_LEN + 1];
  // int msgLen = min(ESP_NOW_MAX_DATA_LEN, dataLen);

  memcpy(&incomingValues, incomingData, sizeof(incomingValues));
  //memcpy(&incomingValues, Data, sizeof(incomingValues));
  //Read all current Nextion values

  Snd = int(incomingValues.Snd);                //Senders Ident 1=Controller 2=Slider 3=PanTilt 4=TurnT 5=Jib

  switch (Snd) {                                //Only Change the Device state if the message was sent by the device
    case 2:
      Sld = int(incomingValues.Sld);            //Slider available 1 or 0
      break;
    case 3:
      PT = int(incomingValues.PT);               //Pan Tilt available 1 or 0
      break;
    case 4:
      TT = int(incomingValues.TT);               //Turntable available 1 or 0
      break;
    case 5:
      JB = int(incomingValues.JB);                 //Jib available 1 or 0
      break;
  }


  Ease_Value = int(incomingValues.Ez);          // Acceloration control
  Bounce = int(incomingValues.Bo);              // Number of bounces
  Crono_time = int(incomingValues.Cro);         //Time in sec of system move
  //Nextion_Func = int(incomingValues.Fnc);       //Current Func of nextion display
  Tps = int(incomingValues.Tps);                //Timlapse frames
  TpsD = int(incomingValues.TpsD);              //Timplapse delay or shutter open time in Bulb mode
  TpsM = int(incomingValues.TpsM);              //Timelapse move comand used to trigger next move from nextion
  in_position = long(incomingValues.In);        //InPoint
  out_position = long(incomingValues.Out);      //OutPoint
  Nextion_play = int(incomingValues.Play);      //playbutton pressed on nextion
  But_Com = int(incomingValues.But);            //playbutton pressed on Slave
  InP = int(incomingValues.InP);                //InPoint button state 0-2
  OutP = int(incomingValues.OutP);              //OutPoint button state 0-2
  SMC = int(incomingValues.SMC);                //StopMotion counter
  PTZ_Cam = int(incomingValues.CA);

  myNex.writeStr("bounce.txt", String(Bounce));    //Bounce counter
  myNex.writeStr("out.txt", String(out_position));
  myNex.writeStr("in.txt", String(in_position));
  myNex.writeStr("tlps.txt", String(Tps));
  myNex.writeStr("ease .txt", String(Ease_Value));
  //myNex.writeStr("mess.txt", String(mess));


  //  strncpy(buffer, (const char *)data, msgLen);
  // make sure we are null terminated
  // buffer[msgLen] = 0;
  // format the mac address
  char macStr[18];
  formatMacAddress(macAddr, macStr, 18);
  // debug log the message to the serial port
  //  Serial.printf("Received message from: %s - %s\n", macStr, buffer);
  //  Serial.print("Last Packet Size: ");
  //  Serial.println (sizeof(incomingValues));
  // Put here what you want to do with the received data
  //  if (strcmp("on", buffer) == 0)
  //  {
  //    ledOn = true;
  //  }
  //  else
  //  {
  //    ledOn = false;
  //  }
  //  digitalWrite(2, ledOn);
}

// callback when data is sent
void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status)
{
  char macStr[18];
  formatMacAddress(macAddr, macStr, 18);
  //  Serial.print("Last Packet Sent to: ");
  //  Serial.println(macStr);
  //  Serial.print("Last Packet Send Status: ");
  //  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
//*******************************ESP_NOW Send VALUES****************************//
void SendNextionValues() {
  broadcast ("");
  NextionValues.Snd = 1;                    //Tells the controller who sent the message 1=Controller 2=Slider 3=PanTilt 4=Turntable 5=Jib
  NextionValues.Ez = Ease_Value;            // Acceloration control
  NextionValues.Bo = Bounce;                // Number of bounces
  NextionValues.Cro = Crono_time;           // Time in sec of system move
  //NextionValues.Fnc = Nextion_Func;         // Current Func of nextion display
  NextionValues.Tps = Tps;                  // Timlapse frames
  NextionValues.TpsD = TpsD;                // Timplapse delay or shutter open time in Bulb mode
  NextionValues.TpsM = TpsM;                // Timelapse move command
  NextionValues.In = in_position;           // InPoint step position value
  NextionValues.Out = out_position;         // OutPoint Step position value
  NextionValues.Play = Nextion_play;        // play comand
  NextionValues.But = But_Com;              // playbutton pressed on Slave
  NextionValues.InP = InP;                  // InPoint button state 0-2
  NextionValues.OutP = OutP;                // OutPoint button state 0-2
  NextionValues.Sld = Sld;                  // Slider present 0-1
  NextionValues.PT = PT;                    // Pan Tilt present 0-1
  NextionValues.TT = TT;                    // Turntable present 0-1
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
  NextionValues.a1 = a1;                    // Acceloration values
  NextionValues.a2 = a2;
  NextionValues.a3 = a3;
  NextionValues.a4 = a4;
  NextionValues.a5 = a5;
  NextionValues.a6 = a6;
  //PTZ variables
  NextionValues.Ma = masterA;              //Master acceloration pot

  NextionValues.Ts = Tilt_J_Speed;         //Joystick Speed
  NextionValues.Ps = Pan_J_Speed;
  NextionValues.Fs = Foc_J_Speed;
  NextionValues.Zs = Zoom_J_Speed;
  NextionValues.Ss = Slid_J_Speed;


  NextionValues.ID = PTZ_ID ;
  NextionValues.CA = PTZ_Cam;
  NextionValues.SP = PTZ_SaveP;
  NextionValues.Pz = PTZ_Pose;
  NextionValues.SM = SM;
  NextionValues.SC = SC;
  NextionValues.Rc = RecordBut;                 //Play button state for PTZ
  NextionValues.LM = LM;                        //Focus/Zoom Limits set 1-6
  NextionValues.Jb = usejoy;
  NextionValues.clrK = clrK;
  //Serial.print("\na1=");
  //Serial.print(NextionValues.a1);

  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(broadcastAddress, (uint8_t *) &NextionValues, sizeof(NextionValues));
}

void UpdateSEQ() {
  Nextion_Wake();
  delay(800);
  myNex.writeStr("k1t.txt", String(s1));
  myNex.writeStr("k2t.txt", String(s2));
  myNex.writeStr("k3t.txt", String(s3));
  myNex.writeStr("k4t.txt", String(s4));
  myNex.writeStr("k5t.txt", String(s5));
  myNex.writeStr("k6t.txt", String(s6));
}

//*************************************** Joystick contro s********************************************
void ReadAnalog() {                                  //Read joystick positions
  tiltS = analogRead(tiltS_PIN);
  panS = analogRead(panS_PIN);
  focusS = analogRead(focusS_PIN);
  zoomS = analogRead(zoomS_PIN);
  slidS = analogRead(slidS_PIN);

  masterA = analogRead(masterA_PIN);
  masterA = constrain(masterA, 900, 4000);




  tiltS = (tiltS - tilt_AVG);
  panS = (panS - pan_AVG);
  focusS = (focusS - foc_AVG);
  zoomS = (zoomS - zoom_AVG);
  slidS = (slidS - slid_AVG);

  //Serial.println(masterA);          //GPIO33

  //**Tilt Control**
  if ( tiltS > 300) {                               //Tilt moving forwards send update
    Tilt_J_Speed = (tiltS);
    sendjoy = true;
  }
  if (tiltS < -300) {                             //Tilt moving Backwards send update
    Tilt_J_Speed = (tiltS);
    sendjoy = true;
  }
  //**Pan Control**
  if ( panS > 200) {                               //Pan moving forwards send update
    Pan_J_Speed = (panS / 4);
    sendjoy = true;
  }
  if (panS < -300) {                             //Pan moving Backwards send update
    Pan_J_Speed = (panS / 2) ;
    sendjoy = true;
  }
  //**Focus Control**
  if ( focusS > 400) {                               //Focus moving forwards send update
    Foc_J_Speed = (focusS);
    sendjoy = true;
  }
  if (focusS < -400) {                                //Focus moving Backwards send update
    Foc_J_Speed = (focusS) ;
    sendjoy = true;
  }

  if (slidS > 300) {                            //Slider moving forwards send update
    Slid_J_Speed = (slidS - 100);
    sendjoy = true;
  }
  if (slidS < -300) {                         //Slider moving Backwards send update
    Slid_J_Speed = (slidS - 200) ;
    sendjoy = true;
  }
  //**Zoom Control**
  //  if (zoomS > 400) {                              //Zoom moving forwards send update
  //    Serial.println("ZoomStepper move forward");
  //
  //    Zoom_J_Speed = (zoomS);
  //    Serial.println(Zoom_J_Speed);
  //    sendjoy = true;
  //  }
  //  if (zoomS < -400) {                              //Zoom moving Backwards send update
  //    Serial.println("ZoomStepper move Backwards");
  //    Zoom_J_Speed = (zoomS);
  //    Serial.println(Zoom_J_Speed);
  //    sendjoy = true;
  //  }
  //**Slider Control**

  //*********Wright out all the above values here******************
  if (sendjoy) {
    SendNextionValues();                             //Send all values
    sendjoy = false;
    joypass = true;
  } else {                                           //send a final stopmove
    if (joypass) {
      Slid_J_Speed = 0;
      Pan_J_Speed = 0;
      Tilt_J_Speed = 0;
      Foc_J_Speed = 0;
      Zoom_J_Speed = 0;
      SendNextionValues();                             //Send all values
      //Serial.println("slid_Accel");
      //Serial.println(slidA);
      sendjoy = false;
      joypass = false;
    }
  }
}
void InitialValues() {

  //Set the values to zero before averaging
  int temp_tilt = 0;
  int temp_pan = 0;
  int temp_foc = 0;
  int temp_zoom = 0;
  int temp_slid = 0;

  for (int i = 0; i < 100; i++) {
    temp_tilt += analogRead(tiltS_PIN);
    delay(10); //allowing a little time between two readings
  }
  for (int i = 0; i < 100; i++) {
    temp_pan += analogRead(panS_PIN);
    delay(10); //allowing a little time between two readings
  }
  for (int i = 0; i < 100; i++) {
    temp_slid += analogRead(slidS_PIN);
    delay(10); //allowing a little time between two readings
  }
  for (int i = 0; i < 100; i++) {
    temp_foc += analogRead(focusS_PIN);
    delay(10); //allowing a little time between two readings
  }
  for (int i = 0; i < 100; i++) {
    temp_zoom += analogRead(zoomS_PIN);
    delay(10); //allowing a little time between two readings
  }


  tilt_AVG = temp_tilt / 100;
  pan_AVG = temp_pan / 100;
  slid_AVG = temp_slid / 100;
  foc_AVG = temp_foc / 100;
  zoom_AVG = temp_zoom / 100;



  //  Serial.println("tilt_Average");
  //  Serial.println(tilt_AVG);
  //  Serial.println("Pan_Average");
  //  Serial.println(pan_AVG);
  //  Serial.println("slid_Average");
  //  Serial.println(slid_AVG);
  //  Serial.println("slid_Accel");
  //  Serial.println(slidA);
}
