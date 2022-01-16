//DBNextionESP32 V2.53
//* Digital Bird Motion Control System Nextion Remote firmware for Adafruit HUZZAH32-ESP32 Feather Board
//This software should be installed on the remote dispaly Adafruit HUZZAH32-ESP32.//
//Written by Colin Henderson April 2021
//V1.03 Update extends life of battery by dimming the Nextion when not in use. Pressing any button will restore full brightness

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

const int NexBat = 35;                     //Battery monitoring pin



//Variables
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
String DBFirmwareVersion = "DBv2.53";
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
int TpsD = 2000;
int Buttonset = 0;
int timelapse_projected = 0;
int getEncoder = 0;
int InP = 0;
int OutP = 0;
int P1;
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
int TT;                                              //Is the turntable present 1 or 0
int mess;




//Structure example to send data
//Must match the receiver structure
//Structure to send WIFI data. Must match the receiver structure
//Structure to send WIFI data. Must match the receiver structure

//Structure to send WIFI data. Must match the receiver structure currently 219bytes of 250 max fo ESP-Now
typedef struct struct_message {
  int Ez;       //6                                   //Ease Value
  int Bo;                                             //Number of bounces
  int TT;                                             //Is the turntable present 1 or 0
  int PT;                                             //Is the turntable present 1 or 0 2 PanTilt Bounce trigger prevents cumulative speed errors
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
  int Fnc;                                                //Nection Function
  int Tps;                                                //Timlapse frames required
  long Out;                                               //Slider Out position in steps
  int But;     
  int InP;                                                //Inpoint button press
  int Sld;                                                //Is the Slider  present 1 or 0
  int OutP;     //8                                       //Out point button press
  int TpsD;                                               //Timlapse Delay in seconds    
  int TpsM;                                               //Timlapse shutter time in seconds
  int Play;                                               //Play button press
  //int mess;
} struct_message;

struct_message NextionValues;     // Create a struct_message to Hold outgoing values
struct_message incomingValues;    // Create a struct_message to hold incoming values from TiltPan and Turntable

int WIFIOUT[8];                   //set up 5 element array for sendinf values to pantilt
#define RXD2 16                   //Hardware Serial2 on ESP32 Dev (must also be a common earth between nextion and esp32)
#define TXD2 17
EasyNex myNex(Serial2);           // Create an object of EasyNex class with the name < myNex >
#define Nextion_Dim_Time 30
hw_timer_t * timer = NULL;


void IRAM_ATTR Nextion_Dim() {
  String dims = "dims=" + String(5);  // set display brightness to 100%
  myNex.writeStr(dims.c_str());

}
void Nextion_Wake() {
  String dims = "dims=" + String(100);  // set display brightness to 100%
  myNex.writeStr(dims.c_str());
}


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


  myNex.writeStr("t9.txt", String(DBFirmwareVersion));            // Show current firmware version on display
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
  NexBatV = analogRead(NexBat);
  NexBatV = ((NexBatV / 4095) * 2) * 3.3;



  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");

}

void loop() {
  //*************************************Listen for Nextion LCD comands*******************************************
  NexBatV = analogRead(NexBat);
  NexBatV = ((NexBatV / 4095) * 2) * 3.3;


  myNex.NextionListen();   // nextion_message.NextionListen(); //check for message from Nextion touchscreen

  //*************************************Listen for slave device button comands*****************************************
  if (But_Com != 0) {                                   //If a button comand has been received from a slave

    switch (But_Com) {

      case 1:                                            //Play request from slave
        Nextion_play = 1;
        But_Com = 0;                                     //Reset the Slave button request
        SendNextionValues();                             //Send out the play command
        Nextion_play = 0;                                //Reset play
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
        delay(5); 
        myNex.writeNum("rewind.val", 0);                 //zero the nextion timer
        delay(5); 
        myNex.writeNum("TimerStop.val", 1);              // timer Start
        But_Com = 0;
        break;
      case 5:
        delay(5); 
        myNex.writeNum("TimerStop.val", 0);              //Stop nextion timer
        But_Com = 0;
        break;
      case 6:
        delay(5); 
        myNex.writeStr("t4.txt", String(Bounce));        //update Bounce counter main
        delay(5); 
        myNex.writeStr("Bounce.txt", String(Bounce));    //update Bounce counter SEQ
        But_Com = 0;
        break;
      case 7:
        delay(5); 
        myNex.writeStr("t5.txt", String(Tps));           //update timlapse counter
        delay(5);
        if (Tps == 0) {
          crono_seconds = 10;
          Crono_time = 10;
          SendNextionValues();
          delay(5);
          myNex.writeStr("t2.txt", String(crono_seconds));
          But_Com = 0;
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
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 1;
  SendNextionValues();
}

void trigger3() {                             // Nextion m2 Sec
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 2;
  SendNextionValues();
}

void trigger4() {                             //Nextion m3 Ease
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 3;
  SendNextionValues();
}

void trigger5() {                             //Nextion m4 Bounce
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 4;
  SendNextionValues();
}

void trigger6() {                            //Nextion m5 Tps
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Func = 5;
  SendNextionValues();
}

void trigger7() {                            //Nextion m6 InPoint
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 6;
  ++InP;
  switch (InP) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      InP = 0;                          //Reset InPoint press back to 0
      break;
  }
}

void trigger8() {                             // Nextion m7 OutPoint

  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 7;
  ++OutP;
  switch (OutP) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      OutP = 0;                            //Reset InPoint press back to 0
      break;
  }
}

void trigger9() {                             // Nextion m7 Play
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_play = 1;
  SendNextionValues();
  delay(200);
  Nextion_play = 0;                           //Reset the button
}


void trigger10() {                                                             // Nextion Up arrow
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");                   // Show current firmware version on display
  Nextion_Wake();
  //  if (Nextion_Func == 0 ) {  // + time Hours on nextion
  //    if (crono_hours >= 0 && crono_hours <= 23) {                               //limit hours that can be set between 0 & 24
  //      crono_hours = crono_hours + 1;                                           // increase by 1
  //      myNex.writeStr("t0.txt", String(crono_hours));
  //      Crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
  //    }
  //  }
  if (Nextion_Func == 1 ) {                                                   // + time Minuts on nextion
    if (crono_minutes >= 0 && crono_hours <= 59) {                             //limit Minuits that can be set between 0 & 60
      crono_minutes = crono_minutes + 1;                                       // increase by 1
      myNex.writeStr("t1.txt", String(crono_minutes));
      Crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
    }
  }
  if (Nextion_Func == 2 ) {  // + time Seconds on nextion
    if (crono_seconds >= 0 && crono_seconds <= 59) {                           //limit seconds that can be set between 0 & 60
      crono_seconds = crono_seconds + 1;                                       // increase by 1
      myNex.writeStr("t2.txt", String(crono_seconds));
      Crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
    }
    if (Tps > 0) {
      TpsD = (crono_seconds * 1000);
    }
  }
  if (Nextion_Func == 3 ) {  // + Ease value
    if (Ease_Value >= 0 && Ease_Value <= 2) {                                 //limit Ease that can be set between 0 & 3
      Ease_Value = Ease_Value + 1;                                            // increase  by 1
      myNex.writeStr("t3.txt", String(Ease_Value));
    }
  }
  if (Nextion_Func == 4 ) {                                                  //+Bounce Value Func
    if (Bounce <= 10 && Bounce != 500) {
      Bounce = Bounce + 1;                                                    // increase  by 1
      myNex.writeStr("t4.txt", String(Bounce));                               // display value in t4 Bounce box on Nextion
      myNex.writeStr("Bounce.txt", String(Bounce));
    } else {
      Bounce = 500;                                                           //If greater than ten assume infinity 500 bring it down again to 10
      myNex.writeStr("t4.txt", "Inf");
      myNex.writeStr("Bounce.txt", "Inf");
    }
  }
  if (Nextion_Func == 5 ) {                                                   // + Tps on nextion
    if (Tps >= 0 ) {
      Tps = Tps + 25;                                                        // increase Tps by 25
      myNex.writeStr("t5.txt", String(Tps));                                  // display seconds in t2 box on Nextion
      if (Tps != 0) {
        timelapse_projected = (5 + (Tps * 2.501));
        crono_seconds = 2;
        myNex.writeStr("t2.txt", "2");   //set seconds deay at 2
        myNex.writeStr("t1.txt", "0");   //set min at 0
        myNex.writeStr("t0.txt", "0");   //set hours at 0
        myNex.writeStr("t8.txt", String(timelapse_projected));
      }
    }
  }
  PanTiltPlay = 1;
  SendNextionValues();
}

void trigger11() {                                                            // Nextion Down arrow
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");                 // Show current firmware version on display
  Nextion_Wake();
  if (Nextion_Func == 0 ) {  // + time Hours on nextion
    if (crono_hours >= 1) {                                                   //limit hours that can be set between 0 & 24
      crono_hours = crono_hours - 1;                                          // increase by 1
      myNex.writeStr("t0.txt", String(crono_hours));
      Crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
    }
  }
  if (Nextion_Func == 1 ) {                                                  // - time Minuts on nextion
    if (crono_minutes >= 1) {                                                 //limit Minuits that can be set between 0 & 60
      crono_minutes = crono_minutes - 1;
      myNex.writeStr("t1.txt", String(crono_minutes));
      Crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
    }
  }
  if (Nextion_Func == 2 ) {  // + time Seconds on nextion
    if (crono_seconds >= 1) {                                                 //limit seconds that can be set between 0 & 60
      crono_seconds = crono_seconds - 1;
      myNex.writeStr("t2.txt", String(crono_seconds));
      Crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
    }
  }
  if (Nextion_Func == 3 ) {  // + Ease value
    if (Ease_Value >= 1) {                                                    //limit Ease that can be set between 0 & 3
      Ease_Value = Ease_Value - 1;                                            // increase  by 1
      myNex.writeStr("t3.txt", String(Ease_Value));
    }
  }
  if (Nextion_Func == 4 ) {                                                  //-Bounce Value Func
    if (Bounce <= 10 && Bounce != 500 && Bounce != 0) {
      Bounce = Bounce - 1;                                                   
      myNex.writeStr("t4.txt", String(Bounce));                               // display value in t4 Bounce box on Nextion
      myNex.writeStr("Bounce.txt", String(Bounce));
    } else {
      Bounce = 10;                                                            //If greater than ten assume infinity 500 bring it down again to 10
      myNex.writeStr("t4.txt", String(Bounce));
      myNex.writeStr("Bounce.txt", String(Bounce));
    }
  }
  if (Nextion_Func == 5 ) {                                                  // + Tps on nextion
    if (Tps > 0 ) {
      Tps = Tps - 25;                                                       // increase Tps by 25
      myNex.writeStr("t5.txt", String(Tps));                                 // display seconds in t2 box on Nextion
      if (Tps == 0) {
        timelapse_projected = 0;
        crono_seconds = 10;
        Crono_time = 10;
        myNex.writeStr("t8.txt", String(timelapse_projected));
        myNex.writeStr("t2.txt", "10");   //set seconds deay at 2
        myNex.writeStr("t1.txt", "0");   //set min at 0
        myNex.writeStr("t0.txt", "0");   //set hours at 0

      }
    }
  }
  PanTiltPlay = 1;
  SendNextionValues();
}

//********************************************sequencer************************************************************
void trigger12() {                            //Nextion K1 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
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
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
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
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 14;
  ++P3;
  switch (P3) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      P3 = 0;                             //Reset InPoint press back to 0
      break;
  }
}
void trigger15() {                            //Nextion K4 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 15;
  ++P4;
  switch (P4) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      P4 = 0;                             //Reset InPoint press back to 0
      break;
  }
}
void trigger16() {                            //Nextion K5 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 16;
  ++P5;
  switch (P5) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      P5 = 0;                             //Reset InPoint press back to 0
      break;
  }
}
void trigger17() {                            //Nextion K6 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 17;
  ++P6;
  switch (P6) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      P6 = 0;                             //Reset InPoint press back to 0
      break;
  }
}
void trigger18() {                         //Nextion Clear key
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
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
void trigger19() {                            //Nextion sequencer play
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 19;
  Nextion_play = 2;
  SendNextionValues();
  delay(200);
  Nextion_play = 0;                           //Reset the button
}

void trigger20() {                                                            // Sequencer Up arrow
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");                  // Show current firmware version on display
  Nextion_Wake();

  switch (Nextion_Func) {
    case 4:                                                                   //+Bounce Value Func
      if (Bounce <= 10 && Bounce != 500) {
        Bounce = Bounce + 1;                                                    // increase  by 1
        myNex.writeStr("t4.txt", String(Bounce));                               // display value in t4 Bounce box on Nextion
        myNex.writeStr("Bounce.txt", String(Bounce));
      } else {
        Bounce = 500;                                                            //If greater than ten assume infinity 500 bring it down again to 10
        myNex.writeStr("t4.txt", "Inf");
        myNex.writeStr("Bounce.txt", "Inf");
      }
      break;
    case 12:                                                                  //K1 Set
      if (SE == 0) {                                                                //If speed selected
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
      if (SE == 0) {                                                                //If speed selected
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
      if (SE == 0) {                                                                //If speed selected
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
      if (SE == 0) {                                                                //If speed selected
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
      if (SE == 0) {                                                                //If speed selected
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
      if (SE == 0) {                                                                //If speed selected
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
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");                  // Show current firmware version on display
  Nextion_Wake();
  switch (Nextion_Func) {

    case 4:
      if (Bounce <= 10 && Bounce != 500 && Bounce != 0) {
        Bounce = Bounce - 1;                                                    // increase  by 1
        myNex.writeStr("t4.txt", String(Bounce));                               // display value in t4 Bounce box on Nextion
        myNex.writeStr("Bounce.txt", String(Bounce));
      } else {
        Bounce = 10;                                                            //If greater than ten assume infinity 500 bring it down again to 10
        myNex.writeStr("t4.txt", String(Bounce));
        myNex.writeStr("Bounce.txt", String(Bounce));
      }
      break;

    case 12:                                                                  //K1 Set
      if (SE == 0) {                                                                //If speed selected
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
      if (SE == 0) {                                                                //If speed selected
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
      if (SE == 0) {                                                                //If speed selected
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
      if (SE == 0) {                                                                //If speed selected
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
      if (SE == 0) {                                                                //If speed selected
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
      if (SE == 0) {                                                                //If speed selected
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
  if (SE == 0) {                                                                // 0=speed
    SE = 1;
  } else {                                                                       //1=Acceloration or ease
    SE = 0;
  }
}

void trigger23() {                                                            // Sequencer add key
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");                  // Show current firmware version on display
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


void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength)
{
  snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}




void receiveCallback(const uint8_t *macAddr, const uint8_t *incomingData,  int Len) {
  // only allow a maximum of 250 characters in the message + a null terminating byte
  char buffer[ESP_NOW_MAX_DATA_LEN + 1];
  // int msgLen = min(ESP_NOW_MAX_DATA_LEN, dataLen);

  memcpy(&incomingValues, incomingData, sizeof(incomingValues));
  //memcpy(&incomingValues, Data, sizeof(incomingValues));
  //Read all current Nextion values

  

  Ease_Value = int(incomingValues.Ez);          // Acceloration control
  Bounce = int(incomingValues.Bo);                // Number of bounces
  Crono_time = int(incomingValues.Cro);         //Time in sec of system move
  Nextion_Func = int(incomingValues.Fnc);        //Current Func of nextion display
  Tps = int(incomingValues.Tps);                //Timlapse frames
  TpsD = int(incomingValues.TpsD);              //Timplapse delay or shutter open time in Bulb mode
  TpsM = int(incomingValues.TpsM);              //Timelapse move comand used to trigger next move from nextion
  in_position = long(incomingValues.In);          //InPoint
  out_position = long(incomingValues.Out);        //OutPoint
  Nextion_play = int(incomingValues.Play);        //playbutton pressed on nextion
  But_Com = int(incomingValues.But);           //playbutton pressed on Slave
  InP = int(incomingValues.InP);                  //InPoint button state 0-2
  OutP = int(incomingValues.OutP);                //OutPoint button state 0-2
  Sld = int(incomingValues.Sld);            //Slider available 1 or 0
  PT = int(incomingValues.PT);                    //Pan Tilt available 1 or 0
  TT = int(incomingValues.TT);              //Turntable available 1 or 0
  


  myNex.writeStr("t4.txt", String(Bounce));       //Bounce counter
  myNex.writeStr("t7.txt", String(out_position));
  myNex.writeStr("t6.txt", String(in_position));
  myNex.writeStr("t5.txt", String(Tps));
  myNex.writeStr("t3.txt", String(Ease_Value));
  //myNex.writeStr("mess.txt", String(mess));


  //  strncpy(buffer, (const char *)data, msgLen);
  // make sure we are null terminated
  // buffer[msgLen] = 0;
  // format the mac address
  char macStr[18];
  formatMacAddress(macAddr, macStr, 18);
  // debug log the message to the serial port
  Serial.printf("Received message from: %s - %s\n", macStr, buffer);

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
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


//Setup ESP_NOW connection
void broadcast(const String &message)
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

void SendNextionValues() {
  broadcast ("");
  NextionValues.Ez = Ease_Value;            // Acceloration control
  NextionValues.Bo = Bounce;                  // Number of bounces
  NextionValues.Cro = Crono_time;           // Time in sec of system move
  NextionValues.Fnc = Nextion_Func;          // Current Func of nextion display
  NextionValues.Tps = Tps;                  // Timlapse frames
  NextionValues.TpsD = TpsD;                // Timplapse delay or shutter open time in Bulb mode
  NextionValues.TpsM = TpsM;                // Timelapse move command
  NextionValues.In = in_position;             // InPoint step position value
  NextionValues.Out = out_position;           // OutPoint Step position value
  NextionValues.Play = Nextion_play;          // play comand
  NextionValues.But = But_Com;             // playbutton pressed on Slave
  NextionValues.InP = InP;                    // InPoint button state 0-2
  NextionValues.OutP = OutP;                  // OutPoint button state 0-2
  NextionValues.Sld = Sld;              // Slider present 0-1
  NextionValues.PT = PT;                      // Pan Tilt present 0-1
  NextionValues.TT = TT;                // Turntable present 0-1
  NextionValues.s1 = s1;                    // Sequencer key speed
  NextionValues.s2 = s2;                    // Sequencer key speed
  NextionValues.s3 = s3;                    // Sequencer key speed
  NextionValues.s4 = s4;                    // Sequencer key speed
  NextionValues.s5 = s5;                    // Sequencer key speed
  NextionValues.s6 = s6;                    // Sequencer key speed
  NextionValues.P1 = P1;                // k1 set button 0-2
  NextionValues.P2 = P2;                // k2 set button 0-2
  NextionValues.P3 = P3;                // k3 set button 0-2
  NextionValues.P4 = P4;                // k4 set button 0-2
  NextionValues.P5 = P5;                // k5 set button 0-2
  NextionValues.P6 = P6;                // k6 set button 0-2
  NextionValues.a1 = a1;                // Acceloration values
  NextionValues.a2 = a2;
  NextionValues.a3 = a3;
  NextionValues.a4 = a4;
  NextionValues.a5 = a5;
  NextionValues.a6 = a6;
  Serial.print("\na1=");
  Serial.print(NextionValues.a1);

  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(broadcastAddress, (uint8_t *) &NextionValues, sizeof(NextionValues));
}

void UpdateSEQ(){
 Nextion_Wake();
delay(800);
myNex.writeStr("k1t.txt", String(s1));
myNex.writeStr("k2t.txt", String(s2));
myNex.writeStr("k3t.txt", String(s3));
myNex.writeStr("k4t.txt", String(s4));
myNex.writeStr("k5t.txt", String(s5));
myNex.writeStr("k6t.txt", String(s6));
}
