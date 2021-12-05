//DBNextionESP32 V2.01
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
String DBFirmwareVersion = "DBv2.00";
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
int Tlps = 0;
int Tlps_countdown = 0;
int Tlps_step_dist = 0;
int TlpsD = 2000;
int Buttonset = 0;
int timelapse_projected = 0;
int getEncoder = 0;
int InP = 0;
int OutP = 0;
int k1P;
int k2P;
int k3P;
int k4P;
int k5P;
int k6P;
int k1s = 50;
int k2s = 50;
int k3s = 50;
int k4s = 0;
int k5s = 0;
int k6s = 0;
int k1a = 0;
int k2a = 0;
int k3a = 0;
int k4a = 0;
int k5a = 0;
int k6a = 0;
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
int TlpsM;

int incomingEase;
int incomingBo;
int incomingCrono;
int incomingFunc;
int incomingTlps;
int incomingTlpsD;
int incomingPlay;
long incomingIn;
long incomingOut;
int incomingButCom;
int incommingInP;
int incommingOutP;
int incommingTlpsM;

int Slider = 0;                                            //Is the Slider  present 1 or 0
int PT;                                                 //Is the Pan Tilt present 1 0 0
int TurnT;                                              //Is the turntable present 1 or 0
int mess;




//Structure example to send data
//Must match the receiver structure
//Structure to send WIFI data. Must match the receiver structure
//Structure to send WIFI data. Must match the receiver structure
typedef struct struct_message {
  int Ease;
  int Bo;
  int Crono;
  int Func;
  int Tlps;
  int TlpsD;
  int Play;
  long In;
  long Out;
  int ButCom;
  int InP;
  int OutP;
  int TlpsM;                                              //Timlapse shutter time in seconds
  int Slider;                                             //Is the Slider  present 1 or 0
  int TurnT;                                              //Is the turntable present 1 or 0
  int PT;                                                 //Is the Pan Tilt present 1 0 0
  int k1P;
  int k2P;
  int k3P;
  int k4P;
  int k5P;
  int k6P;
  int k1s;
  int k2s;
  int k3s;
  int k4s;
  int k5s;
  int k6s;
  int k1a;
  int k2a;
  int k3a;
  int k4a;
  int k5a;
  int k6a;
  int mess;
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
        myNex.writeStr("t5.txt", String(Tlps));           //update timlapse counter
        delay(5);
        if (Tlps == 0) {
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

void trigger6() {                            //Nextion m5 Tlps
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
    if (Tlps > 0) {
      TlpsD = (crono_seconds * 1000);
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
  if (Nextion_Func == 5 ) {                                                   // + Tlps on nextion
    if (Tlps >= 0 ) {
      Tlps = Tlps + 25;                                                        // increase Tlps by 25
      myNex.writeStr("t5.txt", String(Tlps));                                  // display seconds in t2 box on Nextion
      if (Tlps != 0) {
        timelapse_projected = (5 + (Tlps * 2.501));
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
  if (Nextion_Func == 5 ) {                                                  // + Tlps on nextion
    if (Tlps > 0 ) {
      Tlps = Tlps - 25;                                                       // increase Tlps by 25
      myNex.writeStr("t5.txt", String(Tlps));                                 // display seconds in t2 box on Nextion
      if (Tlps == 0) {
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
  ++k1P;
  switch (k1P) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      k1P = 0;                             //Reset InPoint press back to 0
      break;
  }
}
void trigger13() {                            //Nextion K2 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 13;
  ++k2P;
  switch (k2P) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      k2P = 0;                             //Reset InPoint press back to 0
      break;
  }
}
void trigger14() {                            //Nextion K3 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 14;
  ++k3P;
  switch (k3P) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      k3P = 0;                             //Reset InPoint press back to 0
      break;
  }
}
void trigger15() {                            //Nextion K4 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 15;
  ++k4P;
  switch (k4P) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      k4P = 0;                             //Reset InPoint press back to 0
      break;
  }
}
void trigger16() {                            //Nextion K5 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 16;
  ++k5P;
  switch (k5P) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      k5P = 0;                             //Reset InPoint press back to 0
      break;
  }
}
void trigger17() {                            //Nextion K6 Set
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 17;
  ++k6P;
  switch (k6P) {
    case 1:                                 //First Press: Request a new InPoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      k6P = 0;                             //Reset InPoint press back to 0
      break;
  }
}
void trigger18() {                         //Nextion Clear key
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV) + "v");        // Show current firmware version on display
  Nextion_Wake();
  Nextion_Func = 18;
  String k4String = myNex.readStr("k4t.txt");
  if (k4String.toInt() == 0) {
    k4s = 0;
    k5s = 0;
    k6s = 0;
    k4P = 0;
    k5P = 0;
    k6P = 0;
    SendNextionValues();
  } else {
    String k5String = myNex.readStr("k5t.txt");
    if (k5String.toInt() == 0) {
      k5s = 0;
      k6s = 0;
      k5P = 0;
      k6P = 0;
      SendNextionValues();
    }
    else {
      String k6String = myNex.readStr("k5t.txt");
      if (k6String.toInt() == 0) {
        k6s = 0;
        k6P = 0;
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
        if (k1s <= 95) {
          k1s = k1s + 5;
          myNex.writeStr("k1t.txt", String(k1s));
        }
      } else {
        if (k1a <= 2) {
          k1a = k1a + 1;
          myNex.writeStr("k1a.txt", String(k1a));
        }
      }

      break;
    case 13:                                                                  //K2 Set
      if (SE == 0) {                                                                //If speed selected
        if (k2s <= 95) {
          k2s = k2s + 5;
          myNex.writeStr("k2t.txt", String(k2s));
        }
      } else {
        if (k2a <= 2) {
          k2a = k2a + 1;
          myNex.writeStr("k2a.txt", String(k2a));
        }
      }

      break;
    case 14:                                                                  //K3 Set
      if (SE == 0) {                                                                //If speed selected
        if (k3s <= 95) {
          k3s = k3s + 5;
          myNex.writeStr("k3t.txt", String(k3s));
        }
      } else {
        if (k3a <= 2) {
          k3a = k3a + 1;
          myNex.writeStr("k3a.txt", String(k3a));
        }
      }

      break;
    case 15:                                                                  //K4 Set
      if (SE == 0) {                                                                //If speed selected
        if (k4s <= 95) {
          k4s = k4s + 5;
          myNex.writeStr("k4t.txt", String(k4s));
        }
      } else {
        if (k4a <= 2) {
          k4a = k4a + 1;
          myNex.writeStr("k4a.txt", String(k4a));
        }
      }

      break;
    case 16:                                                                  //K5 Set
      if (SE == 0) {                                                                //If speed selected
        if (k5s <= 95) {
          k5s = k5s + 5;
          myNex.writeStr("k5t.txt", String(k5s));
          break;
        }
      } else {
        if (k5a <= 2) {
          k5a = k5a + 1;
          myNex.writeStr("k5a.txt", String(k5a));
          break;
        }
      }

    case 17:                                                                  //K6 Set
      if (SE == 0) {                                                                //If speed selected
        if (k6s <= 95) {
          k6s = k6s + 5;
          myNex.writeStr("k6t.txt", String(k6s));
        }
      } else {
        if (k6a <= 2) {
          k6a = k6a + 1;
          myNex.writeStr("k6a.txt", String(k6a));
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
        if (k1s >= 5) {
          k1s = k1s - 5;
          myNex.writeStr("k1t.txt", String(k1s));
        }
      } else {
        if (k1a > 0) {
          k1a = k1a - 1;
          myNex.writeStr("k1a.txt", String(k1a));
        }
      }

      break;
    case 13:                                                                  //K2 Set
      if (SE == 0) {                                                                //If speed selected
        if (k2s >= 5) {
          k2s = k2s - 5;
          myNex.writeStr("k2t.txt", String(k2s));
        }
      } else {
        if (k2a > 0) {
          k2a = k2a - 1;
          myNex.writeStr("k2a.txt", String(k2a));
        }
      }

      break;
    case 14:                                                                  //K3 Set
      if (SE == 0) {                                                                //If speed selected
        if (k3s >= 5) {
          k3s = k3s - 5;
          myNex.writeStr("k3t.txt", String(k3s));
        }
      } else {
        if (k3a > 0) {
          k3a = k3a - 1;
          myNex.writeStr("k3a.txt", String(k3a));
        }
      }

      break;
    case 15:                                                                  //K4 Set
      if (SE == 0) {                                                                //If speed selected
        if (k4s >= 5) {
          k4s = k4s - 5;
          myNex.writeStr("k4t.txt", String(k4s));
        }
      } else {
        if (k4a > 0) {
          k4a = k4a - 1;
          myNex.writeStr("k4a.txt", String(k4a));
        }
      }

      break;
    case 16:                                                                  //K5 Set
      if (SE == 0) {                                                                //If speed selected
        if (k5s >= 5) {
          k5s = k5s - 5;
          myNex.writeStr("k5t.txt", String(k5s));
        }
      } else {
        if (k5a > 0) {
          k5a = k5a - 1;
          myNex.writeStr("k5a.txt", String(k5a));
        }
      }

      break;
    case 17:                                                                  //K6 Set
      if (SE == 0) {                                                                //If speed selected
        if (k6s >= 5) {
          k6s = k6s - 5;
          myNex.writeStr("k6t.txt", String(k6s));
        }
      } else {
        if (k6a > 0) {
          k6a = k6a - 1;
          myNex.writeStr("k6a.txt", String(k6a));
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
  k4s = k4String.toInt();
  String k5String = myNex.readStr("k5t.txt");
  k5s = k5String.toInt();
  String k6String = myNex.readStr("k6t.txt");
  k6s = k6String.toInt();
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

  //  Serial.println("INCOMING READINGS");
  //  Serial.print("Ease: ");
  //  Serial.print(incomingValues.Ease);
  //  Serial.print("\nBounce: ");
  //  Serial.print(incomingValues.Bo);
  //  Serial.print("\nCronoTime: ");
  //  Serial.print(incomingValues.Crono);
  //  Serial.print("\nNextion Func: ");
  //  Serial.print(incomingValues.Func);
  //  Serial.print("\nTimelapse: ");
  //  Serial.print(incomingValues.Tlps);
  //  Serial.print("\nTlps Delay: ");
  //  Serial.print(incomingValues.TlpsD);
  //  Serial.print("\nIn point set");
  //  Serial.print(incomingValues.In);
  //  Serial.print("\nOut Point set: ");
  //  Serial.print(incomingValues.Out);
  //  Serial.print("\nNextion Func: ");
  //  Serial.print(incomingValues.Func);
  //  Serial.print("\nPlay: ");
  //  Serial.print(incomingValues.Play);
  //  Serial.print("\nBut_com: ");
  //  Serial.print(incomingValues.ButCom);
  //  Serial.print("\nInP: ");
  //  Serial.print(incomingValues.InP);
  //  Serial.print("\nOutP: ");
  //  Serial.print(incomingValues.OutP);
  //  Serial.print("\nSlider: ");
  //  Serial.print(incomingValues.Slider);
  //  Serial.print("\nPanTilt Head: ");
  //  Serial.print(incomingValues.PT);
  //  Serial.print("\nTurntable: ");
  //  Serial.print(incomingValues.TurnT);

  Ease_Value = int(incomingValues.Ease);          // Acceloration control
  Bounce = int(incomingValues.Bo);                // Number of bounces
  Crono_time = int(incomingValues.Crono);         //Time in sec of system move
  Nextion_Func = int(incomingValues.Func);        //Current Func of nextion display
  Tlps = int(incomingValues.Tlps);                //Timlapse frames
  TlpsD = int(incomingValues.TlpsD);              //Timplapse delay or shutter open time in Bulb mode
  TlpsM = int(incomingValues.TlpsM);              //Timelapse move comand used to trigger next move from nextion
  in_position = long(incomingValues.In);          //InPoint
  out_position = long(incomingValues.Out);        //OutPoint
  Nextion_play = int(incomingValues.Play);        //playbutton pressed on nextion
  But_Com = int(incomingValues.ButCom);           //playbutton pressed on Slave
  InP = int(incomingValues.InP);                  //InPoint button state 0-2
  OutP = int(incomingValues.OutP);                //OutPoint button state 0-2
  Slider = int(incomingValues.Slider);            //Slider available 1 or 0
  PT = int(incomingValues.PT);                    //Pan Tilt available 1 or 0
  TurnT = int(incomingValues.TurnT);              //Turntable available 1 or 0
  mess = int(incomingValues.mess);             //Message from client


  myNex.writeStr("t4.txt", String(Bounce));       //Bounce counter
  myNex.writeStr("t7.txt", String(out_position));
  myNex.writeStr("t6.txt", String(in_position));
  myNex.writeStr("t5.txt", String(Tlps));
  myNex.writeStr("t3.txt", String(Ease_Value));
  myNex.writeStr("mess.txt", String(mess));


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
  NextionValues.Ease = Ease_Value;            // Acceloration control
  NextionValues.Bo = Bounce;                  // Number of bounces
  NextionValues.Crono = Crono_time;           // Time in sec of system move
  NextionValues.Func = Nextion_Func;          // Current Func of nextion display
  NextionValues.Tlps = Tlps;                  // Timlapse frames
  NextionValues.TlpsD = TlpsD;                // Timplapse delay or shutter open time in Bulb mode
  NextionValues.TlpsM = TlpsM;                // Timelapse move command
  NextionValues.In = in_position;             // InPoint step position value
  NextionValues.Out = out_position;           // OutPoint Step position value
  NextionValues.Play = Nextion_play;          // play comand
  NextionValues.ButCom = But_Com;             // playbutton pressed on Slave
  NextionValues.InP = InP;                    // InPoint button state 0-2
  NextionValues.OutP = OutP;                  // OutPoint button state 0-2
  NextionValues.Slider = Slider;              // Slider present 0-1
  NextionValues.PT = PT;                      // Pan Tilt present 0-1
  NextionValues.TurnT = TurnT;                // Turntable present 0-1
  NextionValues.k1s = k1s;                    // Sequencer key speed
  NextionValues.k2s = k2s;                    // Sequencer key speed
  NextionValues.k3s = k3s;                    // Sequencer key speed
  NextionValues.k4s = k4s;                    // Sequencer key speed
  NextionValues.k5s = k5s;                    // Sequencer key speed
  NextionValues.k6s = k6s;                    // Sequencer key speed
  NextionValues.k1P = k1P;                // k1 set button 0-2
  NextionValues.k2P = k2P;                // k2 set button 0-2
  NextionValues.k3P = k3P;                // k3 set button 0-2
  NextionValues.k4P = k4P;                // k4 set button 0-2
  NextionValues.k5P = k5P;                // k5 set button 0-2
  NextionValues.k6P = k6P;                // k6 set button 0-2
  NextionValues.k1a = k1a;                // Acceloration values
  NextionValues.k2a = k2a;
  NextionValues.k3a = k3a;
  NextionValues.k4a = k4a;
  NextionValues.k5a = k5a;
  NextionValues.k6a = k6a;
  Serial.print("\nk1a=");
  Serial.print(NextionValues.k1a);

  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(broadcastAddress, (uint8_t *) &NextionValues, sizeof(NextionValues));
}

void UpdateSEQ(){
 Nextion_Wake();
delay(800);
myNex.writeStr("k1t.txt", String(k1s));
myNex.writeStr("k2t.txt", String(k2s));
myNex.writeStr("k3t.txt", String(k3s));
myNex.writeStr("k4t.txt", String(k4s));
myNex.writeStr("k5t.txt", String(k5s));
myNex.writeStr("k6t.txt", String(k6s));
}
