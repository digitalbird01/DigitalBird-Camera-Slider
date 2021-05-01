
/*Digital Bird Slider part of the Digital Bird Motion Control System Created by Colin Henderson 2021
  This code is design for the Digital Bird Camera slider parts of the Digital Bird Motion Control System.
  The code should be loaded up on your sliders ESP32-DevKitC-32D board mounted on the Digital Bird motherboard
*/
#include <FastAccelStepper.h>                 //FastAccelStepper library by gin66 https://github.com/gin66/FastAccelStepper
#include "esp_task_wdt.h"                     //Part of the ESP32 general library
#include <AS5600.h>                           //Library for reading data from the AS5600 encoder
#include <esp_now.h>                          //ESP-Now WIFI library                     
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <Wire.h>


// Setup Stepper pins
#define step1_pinSTEP 26                      // Pin Gpio 26 connected to STEP pin of Stepper Driver
#define step1_pinDIR  27                      // Pin Gpio 27 connected to DIR pin of Stepper Driver
#define StepD 13                              // Pin GPio 13 Activate Deactivate stepper






//Other pins
#define CAM A10                                //Pin Gpio 2/A10 Camera trigger
#define home_switch 15                         //Limit switch Gpio 15/A12


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;


/*
  ESP32 uses Hardware serial RX:16, TX:17
  Serial pins are defined in the ESPNexUpload.cpp file
*/


//AMS AS5600 Encoder setup   The encoder keeps track of the camera position without having to have the stepper driven
AS5600 encoder;



//Variables
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
int crono_seconds = 0;
String DBFirmwareVersion = "frm V:1.02";
String nextion_message;                   // variable to hold received message for Nextion LCD
int move_left = 0;                        // variable to detect move left on Nextion LCD
int move_right = 0;                       // variable to confirm move right on Nextion LCD
int start_cycle = 0;                      // variable to confirm start of travel between IN and OUT points
long Home_position = 0;                    // Variable to st home position on slider
int ease_InOut = 0;                       // Variable to collect Ease_InOut from nextion screen value range 1-3
int Ease_Value = 0;                       // Variable to hold actual Ease value used for acceleration calculation
int ease_time = 0;                        // Extra time allowed for ease in out calculation
int move_finished = 1;                    // Used to check if move is completed
long initial_homing = -1;                 // Used to Home Stepper at startup
int nextion_Func = 0;
int Bounce = 0;
int Tlps = 0;
int Tlps_countdown = 0;
int Tlps_step_dist = 0;
int TlpsD = 0;
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
long PanTiltINpoint;
long PanTiltOUTpoint;
int Nextion_Func;
int Nextion_play;
int But_Com;
int TlpsM;
int ResetEncoder = 0;
int Slider = 1;                                             //Is the Slider  present 1 or 0
int TurnT;                                              //Is the turntable present 1 or 0
int PT;                                             //Is the Pan Tilt present 1 or 0

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

struct_message NextionValues;                             //Define structure for sending values to remote Nextion Control
struct_message incomingValues;                            //Define structure for receiving values from remote Nextion control

int WIFIOUT[8];                                           //set up 5 element array for sendinf values to pantilt
#define RXD2 16                                           //Hardware Serial2 on ESP32 Dev (must also be a common earth between nextion and esp32)
#define TXD2 17

TaskHandle_t C1;
TaskHandle_t C2;

void setup() {
  //*************************************Setup a Tast for Specific core to run Encoder****************************
  xTaskCreatePinnedToCore(core1assignments, "Core_1", 10000, NULL, 2, &C1, 0);
  delay(100);
  //  xTaskCreatePinnedToCore(core2assignments, "Core_2", 10000, NULL, 1, &C2, 1);
  //  delay(100);

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
  Serial.println("\nRunning DigitalBird Slider ESP32 V1.0\n");
  //Pin Settups


  pinMode (CAM, OUTPUT);                                        //Camera Shutter


  pinMode(home_switch, INPUT_PULLUP);                           //Pin A1 pulled high for limit switches

  digitalWrite(StepD, LOW);                                     //Stepper Activation
  digitalWrite(CAM, LOW);                                       //Camera Shutter


  Max_Steps = (((Rail_Length - 43) * 58) / 2);                  // Setup for the rail length

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


  homeStepper();
  output = encoder.getPosition();
  lastOutput = output;
  E_position = output;
  SendNextionValues();
}




void loop() {
  //*************************************Action Nextion Comands*****************************************
  if (inpress != 0) {
    switch (inpress) {
      case 1:                                                  //First Press: Request a new inpoint
        INpointSet();
        break;
      case 2:                                                  //Second Press: Record position
        INpointSet();
        //SendNextionValues();
        inpress = 0;                                          //Reset Inpoint press back to 0
        break;
    }

  }
  if (outpress != 0) {

    switch (outpress) {
      case 1:                                                //First Press: Request a new inpoint
        OUTpointSet();
        break;
      case 2:                                                //Second Press: Record position
        OUTpointSet();
        //SendNextionValues();
        outpress = 0;                                        //Reset Inpoint press back to 0
        break;
    }
  }


  if  (Nextion_play == 1) {                                  //Play comand receved
    if (stepper1->getCurrentPosition() != in_position)  {    // Play Button action if not at atart)
      digitalWrite(StepD, LOW);
      delay(20);
      SetSpeed();
      delay(20);
      Start_1();
    } else {                                                 //System is at the start and ready to play
      SetSpeed();
      delay(20);
      if (Tlps == 0) {                                       //No timelapse
        digitalWrite(CAM, HIGH);                             //Start camera
        delay (200);
        digitalWrite(CAM, LOW);                              //Start camera
        Start_2();
      } else {                                               //Play Timelapse
        Start_4();
      }
    }




    Nextion_play = 0;                                        //Reset play to 0
    SendNextionValues();
  }

} //end loop






//*********************************************************INpoint set*************************************
void core1assignments( void * pvParameters ) {
  for (;;) {
    StartEncoder();
    vTaskDelay(10);
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

  S_position = ((E_position / 2.56));               //Ajust encoder to stepper values the number of steps eqiv
  //Serial.println(S_position);
}






void INpointSet() {
  // Buttonset = 1;
  //digitalWrite(StepD, LOW);
  delay(10);
  switch (inpress) {
    case 1:                                                       //First Press: Request a new OUTpoint
      if (stepper1->getCurrentPosition() != (in_position)) {     //Run to Out position
        stepper1->setSpeedInUs(1000);
        stepper1->setAcceleration(500);
        stepper1->moveTo(in_position);
        while (stepper1->isRunning()) {                          //delay until move complete
          delay(10);
        }
      }
      digitalWrite(StepD, HIGH);                                  //Power down the stepper for manual positioning
      delay(10);
      break;

    case 2:                                                      //Second inpoint press take the encoder values and set as inpoint
      in_position = (S_position);
      stepper1->setCurrentPosition(in_position);                 //Make sure the stepper knows where it is know
      inpress = 0;
      SendNextionValues();                                       //Update nextion with new inpoint values
      delay(10);
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
  switch (outpress) {
    case 1:                                                       //First Press: Request a new OUTpoint
      if (stepper1->getCurrentPosition() != (out_position)) {     //Run to Out position
        stepper1->setSpeedInUs(1000);
        stepper1->setAcceleration(500);
        stepper1->moveTo(out_position);
        while (stepper1->isRunning()) {                        //delay until move complete
          delay(10);
        }
      }
      digitalWrite(StepD, HIGH);                                  //Power down the stepper for manual positioning
      delay(10);
      break;

    case 2:                                                       //Second inpoint press take the encoder values and set as inpoint

      out_position = (S_position);
      stepper1->setCurrentPosition(out_position);                 //Make sure the stepper knows where it is know
      outpress = 0;
      Start_1();
      SendNextionValues();                                        //Update nextion with new inpoint values
      //delay(10);
      //Send the stepper back to In position
      break;
  }
  return;
}


//**********************************Moming at Start**********************************

void homeStepper() {
  while (digitalRead(home_switch) == HIGH) {            // Make the Stepper move CCW until the switch is activated
    stepper1->setSpeedInUs(1500);                         // the parameter is us/step larger values are slower!!!
    stepper1->setAcceleration(500);
    stepper1->moveTo(-20000);                             //"MoveTo" move to a position and stop,
  }
  if (digitalRead(home_switch) == LOW) {
    stepper1->forceStopAndNewPosition(5);                  //Stops dead

    delay(20);
    stepper1->move(300);
    while (stepper1->isRunning()) {                        //delay until move complete
      delay(10);
    }
    stepper1->setCurrentPosition(0);                       //Set this position as home 0
    ResetEncoder = 1;
    SendNextionValues();
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
  //  Serial.print("\nButton_com: ");
  //  Serial.print(incomingValues.ButCom);
  //  Serial.print("\ninpress: ");
  //  Serial.print(incomingValues.Inpress);
  //  Serial.print("\noutpress: ");
  //  Serial.print(incomingValues.Outpress);
  //  Serial.print("\nSlider: ");
  //  Serial.print(incomingValues.Slider);
  //  Serial.print("\nPanTilt Head: ");
  //  Serial.print(incomingValues.PT);
  //  Serial.print("\nTurntable: ");
  //  Serial.print(incomingValues.TurnT);

  ease_InOut = int(incomingValues.Ease);          //Acceleration control
  Bounce = int(incomingValues.Bo);                //Number of bounces
  Crono_time = int(incomingValues.Crono);         //Time in sec of system move
  Nextion_Func = int(incomingValues.Func);        //Current Func of nextion display
  Tlps = int(incomingValues.Tlps);                //Timlapse frames
  TlpsD = int(incomingValues.TlpsD);              //Timplapse delay or shutter open time in Bulb mode
  TlpsM = int(incomingValues.TlpsM);              //Timelapse move comand used to trigger next move from nextion
  //in_position = long(incomingValues.In);              //The slider is the originator so referance only
  //out_position = long(incomingValues.Out);            //The slider is the originator so referance only
  Nextion_play = int(incomingValues.Play);        //playbutton pressed on nextion
  But_Com = int(incomingValues.ButCom);           //playbutton pressed on Slave
  inpress = int(incomingValues.Inpress);          //Inpoint button state 0-2
  outpress = int(incomingValues.Outpress);        //Outpoint button state 0-2
  //Slider = int(incomingValues.Slider);                //The slider is the originator so here for referance only
  PT = int(incomingValues.PT);                    //Pan Tilt available 1 or 0
  TurnT = int(incomingValues.TurnT);              //Turntable available 1 or 0

  //  Serial.print("\noutpress set to: ");
  //  Serial.print(outpress);
  //  strncpy(buffer, (const char *)data, msgLen);
  // make sure we are null terminated
  // buffer[msgLen] = 0;
  // format the mac address
  char macStr[18];
  formatMacAddress(macAddr, macStr, 18);
  // debug log the message to the serial port
  Serial.printf("Received message from: %s - %s\n", macStr, buffer);

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

void SendNextionValues() {
  broadcast ("");
  NextionValues.Ease = ease_InOut;            // Acceloration control
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
  NextionValues.PT = PT;                      // Pan Tilt present 0-1
  NextionValues.TurnT = TurnT;                // Turntable present 0-1


  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(broadcastAddress, (uint8_t *) &NextionValues, sizeof(NextionValues));
}




//*********************************************ARM The SLIDER READY TO START**************************************************
void Start_1() {
  //  Arm the slider to start point ready for second play press
  digitalWrite(StepD, LOW);
  delay(100);

  stepper1->setSpeedInUs(500);
  stepper1->setAcceleration(900);
  stepper1->moveTo(in_position);
  while (stepper1->isRunning()) {                                 //delay until move complete (block)
    delay(10);
  }
  But_Com = 5;
  SendNextionValues();
}

//************************************************RUN THE SEQUENCE BASIC**************************************************************
void Start_2() {
  
  But_Com = 4;
  SendNextionValues();
  stepper1->setSpeedInUs(step_speed);
  stepper1->setAcceleration(Ease_Value);
  delay(15);                                               //Timed to start with PanTilt head
  stepper1->moveTo(out_position);
  while (stepper1->isRunning()) {                         //delay until move complete (block)
    delay(5);
  }

  if (Bounce >= 1) {
    Start_3();
  } else  {                                              // Count down number of bounces required
    But_Com = 5;
    SendNextionValues();
    Start_1();
  }
  digitalWrite(CAM, HIGH);                               //Stop camera
  delay (200);
  digitalWrite(CAM, LOW);
  return;
}

//******************************************RUN THE SEQUENCE with Bounce************************************************
void Start_3() {

  digitalWrite(StepD, LOW);
  stepper1->setSpeedInUs(step_speed);
  stepper1->setAcceleration(Ease_Value);
  delay(15);                                               //Timed to start with PanTilt head
  stepper1->moveTo(in_position);
  while (stepper1->isRunning()) {                         //delay until move complete (block)
    delay(5);
  }

  Bounce = Bounce - 1;
  But_Com = 6;
  SendNextionValues();                                  //Send current bounce counter back to nextion
  But_Com = 0;
  if (Bounce >= 1) {
    Start_2();
  } else {
    But_Com = 5;                                       //Tell the nextion to stop the timer
    SendNextionValues;
    But_Com = 0;
    Start_1();
  }
  return;
}

//**********************************************Timelapse*******************************************
void Start_4() {

  // TlpsD = (crono_seconds * 1000);                                      //use the timer seconds as delay time for timelapse

  Tlps_step_dist = travel_dist / Tlps;

  digitalWrite(StepD, LOW);
  step_speed = (travel_dist / 5);
  stepper1->setSpeedInUs(800);
  stepper1->setAcceleration(500);

  while (Tlps >= 1) {
    TlpsM = 4;                                                            //Trigger pantilt head timelapse move
    But_Com = 7;                                                          //Tell the nextion to update the frame counter
    SendNextionValues();
    stepper1->moveTo(stepper1->getCurrentPosition() + Tlps_step_dist);    //Current position plus one fps move
    while (stepper1->isRunning()) {                                       //delay until move complete (blocking)
      delay(5);
    }
    delay (1000);                                                         //Take a second to ensure camera is still
    digitalWrite(CAM, HIGH);                                              //Fire shutter
    delay (TlpsD);                                                        //Time the shutter is open for
    digitalWrite(CAM, LOW);                                               //Cose the shutter and move on
    delay (500);
    if (Tlps != 0) {
      Tlps = Tlps - 1;
    }
    // But_Com = 0;
  }
  But_Com = 5;                                                            //Tell the nextion to stop the timer
  SendNextionValues;
  But_Com = 0;
  //NextionLCD.setComponentValue("TimerStop", 0);
  //NextionLCD.setComponentText("t5", String(fps));
  Start_1();                                        //send the camera back to the start
}

//******Set stepper speed based on time distance and amount od ease InOut***********
void SetSpeed() {

  travel_dist = (out_position - in_position);              // Distance we have to go
  factor = (travel_dist / Crono_time);                     // fot the next three lines This formula should all be on one line but I cant get it to work!
  factor = (1 / factor);
  step_speed = abs(factor * 1000000);
  //step_speed = ((step_speed/100)*105);                   //Ajustment for PT

  switch (ease_InOut) {
    case 0:                                                //No ease
      Ease_Value = (2000);
      break;

    case 3:
      travel_dist = abs(out_position - in_position);                   // Distance we have to go in steps /3 for ramp at start and end
      factor = (1 / (travel_dist / (Crono_time * 12)));                // Distanse in steps/ (time in seconds/3) to account for ramp at start and end
      step_speed = abs(factor * 1000000);
      Ease_Value = (travel_dist / (Crono_time * 12));                  //Calculate the Acceloration value for the ramps total distance/3 time /3
      break;

    case 2:
      travel_dist = abs(out_position - in_position);                   // Distance we have to go in steps /3 for ramp at start and end
      factor = (1 / (travel_dist / (Crono_time * 12)));                // Distanse in steps/ (time in seconds/3) to account for ramp at start and end
      step_speed = abs(factor * 1000000);
      Ease_Value = (travel_dist / (Crono_time * 12));                  //Calculate the Acceloration value for the ramps total distance/3 time /3
      Ease_Value = ( Ease_Value * 2);
      break;

    case 1:
      travel_dist = abs(out_position - in_position);                   // Distance we have to go in steps /3 for ramp at start and end
      factor = (1 / (travel_dist / (Crono_time * 12)));                // Distanse in steps/ (time in seconds/3) to account for ramp at start and end
      step_speed = abs(factor * 1000000);
      Ease_Value = (travel_dist / (Crono_time * 12));                  //Calculate the Acceloration value for the ramps total distance/3 time /3
      Ease_Value = ( Ease_Value * 3);
      break;
  }

  return;
}





//*********************************************Error Reset***************************************//
//int ErrorReset() {
//  //NextionLCD.setComponentValue("TimerStop", 999);
//  //return;
//}
