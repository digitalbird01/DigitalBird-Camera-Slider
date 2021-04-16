//DBNextionESP32 V0.02
//* Digital Bird Motion Control System Nextion Remote firmware for Adafruit HUZZAH32-ESP32 Feather Board
//This software should be installed on the remote dispaly Adafruit HUZZAH32-ESP32.//

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

long set_speed = 0;
long in_position = 0;                       // variable to hold IN position for slider
long out_position = 0;                      // variable to hold OUT position for slider
long travel_dist = 0;
long step_speed = 0;                        // default travel speed between IN and OUT points
int Crono_time = 10;
int crono_hours = 0;
int crono_minutes = 0;
int crono_seconds = 10;
String DBFirmwareVersion = "DBv1.00";
String nextion_message;                   // variable to hold received message for Nextion LCD
int move_left = 0;                        // variable to detect move left on Nextion LCD
int move_right = 0;                       // variable to confirm move right on Nextion LCD
int start_cycle = 0;                      // variable to confirm start of travel between IN and OUT points
long Home_position = 0;                    // Variable to st home position on slider
int Ease_Value = 0;                       // Variable to collect Ease_Value from nextion screen value range 1-3
int ease_time = 0;                        // Extra time allowed for ease in out calculation
int move_finished = 1;                    // Used to check if move is completed
long initial_homing = -1;                 // Used to Home Stepper at startup
int nextion_Func;
int Bounce = 0;
int Tlps = 0;
int Tlps_countdown = 0;
int Tlps_step_dist = 0;
int TlpsD = 2000;
int Buttonset = 0;
int timelapse_projected = 0;
int getEncoder = 0;
int inpress = 0;
int outpress = 0;
int TiltPanComand = 0;
int playbuttoncounter = 1;
String dataTx;
String wifimessage;
int PanTiltPlay;
int PanTiltINpoint;
int PanTiltOUTpoint;
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
int incommingInpress;
int incommingOutpress;
int incommingTlpsM;

int Slider =0;                                             //Is the Slider  present 1 or 0
int PT;                                                 //Is the Pan Tilt present 1 0 0
int TurnT;                                              //Is the turntable present 1 or 0





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
  int Inpress;
  int Outpress;
  int TlpsM;                                              //Timlapse shutter time in seconds
  int Slider;                                             //Is the Slider  present 1 or 0
  int TurnT;                                              //Is the turntable present 1 or 0
  int PT;                                                 //Is the Pan Tilt present 1 0 0
} struct_message;


struct_message NextionValues;     // Create a struct_message to Hold outgoing values
struct_message incomingValues;    // Create a struct_message to hold incoming values from TiltPan and Turntable

int WIFIOUT[8];                   //set up 5 element array for sendinf values to pantilt
#define RXD2 16                   //Hardware Serial2 on ESP32 Dev (must also be a common earth between nextion and esp32)
#define TXD2 17
EasyNex myNex(Serial2);           // Create an object of EasyNex class with the name < myNex >




void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("\nRunning DigitalBird Nextion ESP32 controler\n");


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
 NexBatV = ((NexBatV / 4095)*2)*3.3;

 
 
  myNex.writeStr("t10.txt", "Bat:" + String(NexBatV)+"v");

}

void loop() {
  //*************************************Listen for Nextion LCD comands*******************************************
  NexBatV = analogRead(NexBat);
  NexBatV = ((NexBatV / 4095)*2)*3.3;


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
      case 2:                                            //Inpoint set request from slave
        trigger7();                                      //Same as if pressed on nextion
        But_Com = 0;
        break;
      case 3:                                            //Outpoint set request from slave
        trigger8();                                      //Same as if pressed on nextion
        But_Com = 0;
        break;
      case 4:
        myNex.writeNum("rewind.val", 0);                //zero the nextion timer
        myNex.writeNum("TimerStop.val", 1);             // timer Start
        But_Com = 0;
        break;
      case 5:
        myNex.writeNum("TimerStop.val", 0);              //Stop nextion timer
        But_Com = 0;
        break;
      case 6:
        myNex.writeStr("t4.txt", String(Bounce));        //update timlapse counter
        But_Com = 0;
        break;
      case 7:
        myNex.writeStr("t5.txt", String(Tlps)); //update timlapse counter
        But_Com = 0;
        break;
    }
  }



} //end loop




//***************************Nextion Button Press Functions*****************************//

void trigger1() {                              //Nextion m0 Hours
   myNex.writeStr("t10.txt", "Bat:" + String(NexBatV)+"v");
  Nextion_Func = 0;
  SendNextionValues();

}
void trigger2() {                             //Nextion m1 Min
   myNex.writeStr("t10.txt", "Bat:" + String(NexBatV)+"v");          // Show current firmware version on display
  Nextion_Func = 1;
  SendNextionValues();
}

void trigger3() {                             // Nextion m2 Sec
   myNex.writeStr("t10.txt", "Bat:" + String(NexBatV)+"v");          // Show current firmware version on display
  Nextion_Func = 2;
  SendNextionValues();
}

void trigger4() {                             //Nextion m3 Ease
   myNex.writeStr("t10.txt", "Bat:" + String(NexBatV)+"v");          // Show current firmware version on display
  Nextion_Func = 3;
  SendNextionValues();
}

void trigger5() {                             //Nextion m4 Bounce
   myNex.writeStr("t10.txt", "Bat:" + String(NexBatV)+"v");          // Show current firmware version on display
  Nextion_Func = 4;
  SendNextionValues();
}

void trigger6() {                            //Nextion m5 Tlps
   myNex.writeStr("t10.txt", "Bat:" + String(NexBatV)+"v");          // Show current firmware version on display
  Nextion_Func = 5;
  SendNextionValues();
}

void trigger7() {                            //Nextion m6 Inpoint
   myNex.writeStr("t10.txt", "Bat:" + String(NexBatV)+"v");          // Show current firmware version on display
  Nextion_Func = 6;
  ++inpress;
  switch (inpress) {
    case 1:                                 //First Press: Request a new inpoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      inpress = 0;                          //Reset Inpoint press back to 0
      break;
  }
}

void trigger8() {                             // Nextion m7 OutPoint
 
   myNex.writeStr("t10.txt", "Bat:" + String(NexBatV)+"v");          // Show current firmware version on display
  Nextion_Func = 7;
  ++outpress;
  switch (outpress) {
    case 1:                                 //First Press: Request a new inpoint
      SendNextionValues();
      break;
    case 2:                                 //Second Press: Record position
      SendNextionValues();
      outpress = 0;                     //Reset Inpoint press back to 0
      break;
  }
}

void trigger9() {                             // Nextion m7 Play
   myNex.writeStr("t10.txt", "Bat:" + String(NexBatV)+"v");          // Show current firmware version on display
  Nextion_play = 1;
  SendNextionValues();
  delay(200);
  Nextion_play = 0;                           //Reset the button
}


void trigger10() {                                                             // Nextion Up arrow
   myNex.writeStr("t10.txt", "Bat:" + String(NexBatV)+"v");          // Show current firmware version on display
  if (Nextion_Func == 0 ) {  // + time Hours on nextion
    if (crono_hours >= 0 && crono_hours <= 23) {                               //limit hours that can be set between 0 & 24
      crono_hours = crono_hours + 1;                                           // increase by 1
      myNex.writeStr("t0.txt", String(crono_hours));
      Crono_time = ((crono_hours * 3600) + (crono_minutes * 60) + crono_seconds);
    }
  }
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
  if (Nextion_Func == 4 ) {                                                   //+Bounce Value Function
    if (Bounce >= 0 && Bounce <= 9 ) {
      Bounce = Bounce + 1;                                                     // increase  by 1
      myNex.writeStr("t4.txt", String(Bounce));                                // display value in t4 Bounce box on Nextion
    } else {
      Bounce = 500;                                                            //If greater than ten assume infinity 500
      myNex.writeStr("t4.txt", String(Bounce));
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
   myNex.writeStr("t10.txt", "Bat:" + String(NexBatV)+"v");          // Show current firmware version on display
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
  if (Nextion_Func == 4 ) {                                                  //+Bounce Value Func
    if (Bounce >= 1 && Bounce != 500) {
      Bounce = Bounce - 1;                                                    // increase  by 1
      myNex.writeStr("t4.txt", String(Bounce));                               // display value in t4 Bounce box on Nextion
    } else {
      Bounce = 10;                                                            //If greater than ten assume infinity 500 bring it down again to 10
      myNex.writeStr("t4.txt", String(Bounce));
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
//***End find Func****


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

  Serial.println("INCOMING READINGS");
  Serial.print("Ease: ");
  Serial.print(incomingValues.Ease);
  Serial.print("\nBounce: ");
  Serial.print(incomingValues.Bo);
  Serial.print("\nCronoTime: ");
  Serial.print(incomingValues.Crono);
  Serial.print("\nNextion Func: ");
  Serial.print(incomingValues.Func);
  Serial.print("\nTimelapse: ");
  Serial.print(incomingValues.Tlps);
  Serial.print("\nTlps Delay: ");
  Serial.print(incomingValues.TlpsD);
  Serial.print("\nIn point set");
  Serial.print(incomingValues.In);
  Serial.print("\nOut Point set: ");
  Serial.print(incomingValues.Out);
  Serial.print("\nNextion Func: ");
  Serial.print(incomingValues.Func);
  Serial.print("\nPlay: ");
  Serial.print(incomingValues.Play);
  Serial.print("\nBut_com: ");
  Serial.print(incomingValues.ButCom);
  Serial.print("\ninpress: ");
  Serial.print(incomingValues.Inpress);
  Serial.print("\noutpress: ");
  Serial.print(incomingValues.Outpress);
  Serial.print("\nSlider: ");
  Serial.print(incomingValues.Slider);
  Serial.print("\nPanTilt Head: ");
  Serial.print(incomingValues.PT);
  Serial.print("\nTurntable: ");
  Serial.print(incomingValues.TurnT);

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
  inpress = int(incomingValues.Inpress);          //Inpoint button state 0-2
  outpress = int(incomingValues.Outpress);        //Outpoint button state 0-2
  Slider = int(incomingValues.Slider);            //Slider available 1 or 0
  PT = int(incomingValues.PT);                    //Pan Tilt available 1 or 0
  TurnT = int(incomingValues.TurnT);              //Turntable available 1 or 0



  myNex.writeStr("t4.txt", String(Bounce));       //Bounce counter
  myNex.writeStr("t7.txt", String(out_position));
  myNex.writeStr("t6.txt", String(in_position));
  myNex.writeStr("t5.txt", String(Tlps));
  myNex.writeStr("t3.txt", String(Ease_Value));



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
  NextionValues.Inpress = inpress;            // Inpoint button state 0-2
  NextionValues.Outpress = outpress;          // Outpoint button state 0-2
  NextionValues.Slider = Slider;              // Slider present 0-1
  NextionValues.Slider = PT;                  // Pan Tilt present 0-1
  NextionValues.Slider = TurnT;               // Turntable present 0-1


  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(broadcastAddress, (uint8_t *) &NextionValues, sizeof(NextionValues));
}
