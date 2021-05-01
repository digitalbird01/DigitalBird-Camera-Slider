
/*Digital Bird PanTilt Head part of the Digital Bird Motion Control System Created by Colin Henderson 2021
  This code is in the public domain...
  You can: copy it, use it, modify it, share it or just plain ignore it!
  Thx!
*/


#include <FastAccelStepper.h>
#include "esp_task_wdt.h"
#include <AS5600.h>

#include <esp_now.h>
#include <trigger.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <Wire.h>


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



FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;



//Variables

String DBFirmwareVersion = "frm V:1.00";      //Current firmware version
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
int Mount=0;                                   //Is the Mount function active 1 true 0 false

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


long Crono_time = 10;
int crono_hours = 0;
int crono_minutes = 0;
int crono_seconds = 0;


int move_left = 0;                        // variable to detect move left on Nextion LCD
int move_right = 0;                       // variable to confirm move right on Nextion LCD
int start_cycle = 0;                      // variable to confirm start of travel between IN and OUT points
long Home_position = 0;                    // Variable to st home position
int ease_InOut = 0;                       // Variable to collect Ease_InOut from nextion screen value range 1-3

int ease_time = 0;                        // Extra time allowed for ease in out calculation
int move_finished = 1;                    // Used to check if move is completed
long initial_homing = 0;                 // Used to Home Stepper at startup
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


int Slider;                                             //Is the Slider  present 1 or 0
int TurnT;                                              //Is the turntable present 1 or 0
int PT = 1;                                             //Is the Pan Tilt present 1 or 0

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


AS5600 encoder;                                           //AMS AS5600 Encoder setup   The encoder keeps track of the focus motor position on stepper3


int WIFIOUT[8];                                           //set up 5 element array for sendinf values to pantilt
//#define RXD2 16                                           //Hardware Serial2 on ESP32 Dev (must also be a common earth between nextion and esp32)
//#define TXD2 17

TaskHandle_t C1;
TaskHandle_t C2;



void setup() {
  //*************************************Setup a core to run Encoder****************************
  xTaskCreatePinnedToCore(core1assignments, "Core_1", 10000, NULL, 2, &C1, 0);
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

  //  pin setups


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
  
  InitialValues();

}//end setup


void loop() {
  //ADD Back IN******************Listen for mount/Dismount button comands*********************
  if (stepper1->isRunning() || stepper2->isRunning()) {
    // delay(1);
  } else {
    ReadAnalog(); //Joystick setup
    ReadMount();
  }






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


  if  (Nextion_play == 1) {                                     //Play comand receved
    if (stepper1->getCurrentPosition() != (TLTin_position) || stepper2->getCurrentPosition() != (PANin_position) ) {

      digitalWrite(StepFOC, LOW);
      delay(20);
      SetSpeed();
      delay(20);
      Start_1();
    } else {                                                   //System is at the start and ready to play
      SetSpeed();                                            // This causes reboot?????????????
      delay(20);
      if (Tlps == 0) {                                         //No timelapse
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
  //digitalWrite(StepFOC, HIGH);
  delay(10);
  switch (inpress) {
    case 1:                                                       //First Press: Request a new OUTpoint
      if (stepper1->getCurrentPosition() != (TLTin_position) || stepper2->getCurrentPosition() != (PANin_position) ) {   //Run to Out position
        stepper1->setSpeedInUs(200);
        stepper1->setAcceleration(3000);

        stepper2->setSpeedInUs(1000);
        stepper2->setAcceleration(500);

        stepper3->setSpeedInUs(1000);
        stepper3->setAcceleration(500);

        stepper1->moveTo(TLTin_position);
        delay(10);
        stepper2->moveTo(PANin_position);
        delay(10);
        stepper3->moveTo(FOCin_position);


        while (stepper1->isRunning()) {                               //delay until move complete
          delay(10);
        }
      }
      digitalWrite(StepFOC, HIGH);                                    //Power down the Focus stepper only for manual positioning
      delay(10);
      inpress = 0;
      break;

    case 2:                                                           //Second inpoint press take the encoder values and set as inpoint
      TLTin_position = stepper1->getCurrentPosition();                //Burn in positions
      PANin_position = stepper2->getCurrentPosition();
      FOCin_position = (S_position);                                  //Only the Focus motor has an encoder this is getting the value from the encoder
      inpress = 0;
      SendNextionValues();                                            //Update nextion with new inpoint values
      delay(10);
      Start_1();                                                      //Send the stepper back to In position
      //SetSpeed();
      break;
  }
  return;
}

//*********************************************************OUTpoint set*************************************
void OUTpointSet() {


  delay(10);
  switch (outpress) {
    case 1:                                                           //First Press: Request a new OUTpoint
      if (stepper1->getCurrentPosition() != (TLTout_position) || stepper2->getCurrentPosition() != (PANout_position) ) {

        stepper1->setSpeedInUs(200);                                 //Setup stepper speed and acceloration
        stepper1->setAcceleration(3000);
        stepper2->setSpeedInUs(1000);
        stepper2->setAcceleration(500);
        stepper3->setSpeedInUs(1000);
        stepper3->setAcceleration(500);

        stepper1->moveTo(TLTout_position);                            //Start moves
        delay(10);
        stepper2->moveTo(PANout_position);
        delay(10);
        stepper3->moveTo(FOCout_position);

        while (stepper1->isRunning()||stepper2->isRunning() ) {                               //delay until move complete
          delay(10);
        }
      }
      digitalWrite(StepFOC, HIGH);                                      //Power down the stepper for manual positioning
      delay(10);
      //Serial.println("leaving out press1");
      outpress = 0;
      break;

    case 2:                                                           //Second outpoint press take the encoder values and set as inpoint
      TLTout_position = stepper1->getCurrentPosition();                //Burn in positions
      Serial.println("TLTout_POsition=");
      Serial.println(TLTout_position);
      PANout_position = stepper2->getCurrentPosition();
      FOCout_position = (S_position);
      outpress = 0;
      Start_1();
      delay(10);
      SendNextionValues();                                            //Update nextion with new inpoint values

      //Send the stepper back to In position
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
  Slider = int(incomingValues.Slider);            //The slider is the originator so here for referance only
  //PT = int(incomingValues.PT);                  //Pan Tilt available not required by Pan Tilt
  TurnT = int(incomingValues.TurnT);              //Turntable available 1 or 0

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
  NextionValues.In = PANin_position;             // InPoint step position value
  NextionValues.Out = PANout_position;           // OutPoint Step position value
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
  Serial.print("Entering Start1 ");
  digitalWrite(StepFOC, LOW);

  delay(100);

  stepper1->setSpeedInUs(200);
  stepper1->setAcceleration(3000);
  stepper2->setSpeedInUs(500);
  stepper2->setAcceleration(700);
  stepper3->setSpeedInUs(500);
  stepper3->setAcceleration(700);
  stepper1->moveTo(TLTin_position);
  delay(10);
  stepper2->moveTo(PANin_position);
  delay(10);
  stepper3->moveTo(FOCin_position);

  while (stepper1->isRunning()) {                                  //delay until move complete (block)
    delay(10);
  }
  if (Slider == 0) {                                                //Only take charhe of the counting if no slider
    But_Com = 5;
    //Serial.print("Leaving Start1 ");
    SendNextionValues();
  }
}

//************************************************RUN THE SEQUENCE BASIC**************************************************************
void Start_2() {
  digitalWrite(StepFOC, LOW);                                       //Engage the focus stepper
  //Serial.print("Entering Start2 ");
  //Serial.print(outpress);
  if (Slider == 0) {                                                //Only take charhe of the counting if no slider
    But_Com = 4;
    SendNextionValues();
  }
  if (TLTtravel_dist != 0) {
    stepper1->setSpeedInUs(TLTstep_speed);                            //Setup speed and acceloration values
    stepper1->setAcceleration(TLTease_Value);
    stepper1->moveTo(TLTout_position);                                //Move the steppers
    delay(2);
  }
  if (PANtravel_dist != 0) {
    stepper2->setSpeedInUs(PANstep_speed);
    stepper2->setAcceleration(PANease_Value);
    stepper2->moveTo(PANout_position);
    delay(2);
  }
  if (FOCtravel_dist != 0) {
    stepper3->setSpeedInUs(FOCstep_speed);
    stepper3->setAcceleration(FOCease_Value);
    stepper3->moveTo(FOCout_position);
  }
  while (stepper2->isRunning() || stepper1->isRunning()  ) {           //delay until move complete (block)
    delay(2);
  }

  if (Bounce >= 1) {
    Start_3();
  } else  {                                                              // Count down number of bounces required
    if (Slider == 0) {                                                   //Only take charhe of the counting if no slider
      But_Com = 4;
      SendNextionValues();
    }
    Start_1();
  }
  digitalWrite(CAM, HIGH);                                              //Stop camera switch must be flipped
  delay (200);
  digitalWrite(CAM, LOW);
  return;
}

//******************************************RUN THE SEQUENCE with Bounce************************************************
void Start_3() {

  //digitalWrite(StepD, LOW);
  stepper1->setSpeedInUs(TLTstep_speed);                     //Setup speed and acceloration values
  stepper1->setAcceleration(TLTease_Value);
  stepper2->setSpeedInUs(PANstep_speed);
  stepper2->setAcceleration(PANease_Value);
  stepper3->setSpeedInUs(FOCstep_speed);
  stepper3->setAcceleration(FOCease_Value);

  stepper1->moveTo(TLTin_position);                      //Move the steppers
  delay(2);
  stepper2->moveTo(PANin_position);
  delay(2);
  stepper3->moveTo(FOCin_position);
  while (stepper1->isRunning() || stepper2->isRunning() || stepper3->isRunning()) {                       //delay until move complete (block)
    delay(2);
  }
  if (Slider == 0) {                                      //Only take charhe of the counting if no slider
    Bounce = Bounce - 1;
    But_Com = 6;
    SendNextionValues();                                  //Send current bounce counter back to nextion
    But_Com = 0;
  }
  if (Bounce >= 1) {
    Start_2();
  } else {
    if (Slider == 0) {
      But_Com = 5;                                       //Tell the nextion to stop the timer
      SendNextionValues;
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
  // TlpsD = (crono_seconds * 1000);                                      //use the timer seconds as delay time for timelapse

  TLTTlps_step_dist = TLTtravel_dist / Tlps;
  PANTlps_step_dist = PANtravel_dist / Tlps;
  FOCTlps_step_dist = FOCtravel_dist / Tlps;

  //digitalWrite(StepD, LOW);
  TLTstep_speed = (TLTtravel_dist / 5);
  PANstep_speed = (PANtravel_dist / 5);
  FOCstep_speed = (FOCtravel_dist / 5);


  stepper1->setSpeedInUs(200);
  stepper1->setAcceleration(1000);
  stepper2->setSpeedInUs(500);
  stepper2->setAcceleration(500);
  stepper3->setSpeedInUs(500);
  stepper3->setAcceleration(500);

  while (Tlps >= 1) {

    if (Slider == 1) {                                                         //If Slider is conected give slider control over timing

      if (TlpsM == 4) {                                                          //Trigger pantilt head timelapse move
        //Serial.println("Tlps Trigger from slider");

        TlpsM = 0;
        stepper1->moveTo(stepper1->getCurrentPosition() + TLTTlps_step_dist);    //Current position plus one fps move
        delay(5);
        stepper2->moveTo(stepper2->getCurrentPosition() + PANTlps_step_dist);    //Current position plus one fps move
        delay(5);
        stepper3->moveTo(stepper3->getCurrentPosition() + FOCTlps_step_dist);    //Current position plus one fps move

        while (stepper1->isRunning() || stepper2->isRunning() ) {                                        //delay until move complete (blocking)
          delay(5);
        }
        delay (1000);                                                           //Take a second to ensure camera is still
        digitalWrite(CAM, HIGH);                                                //Fire shutter
        delay (TlpsD);                                                          //Time the shutter is open for
        digitalWrite(CAM, LOW);                                                 //Cose the shutter and move on

        if (Tlps != 0) {
          Tlps = Tlps - 1;
        }

      }

    }

    if (Slider == 0) {                                                               //If Slider is not with us take control
      Serial.println("Tlps Trigger from PT");
      TlpsM = 4;                                                               //Trigger pantilt head timelapse move
      SendNextionValues();                                                     //Trigger pantilt head timelapse move
      stepper1->moveTo(stepper1->getCurrentPosition() + TLTTlps_step_dist);    //Current position plus one fps move
      delay(10);
      stepper2->moveTo(stepper2->getCurrentPosition() + PANTlps_step_dist);    //Current position plus one fps move
      delay(10);
      stepper3->moveTo(stepper3->getCurrentPosition() + FOCTlps_step_dist);    //Current position plus one fps move

      if (Tlps != 0) {
        Tlps = Tlps - 1;
      }
      But_Com = 7;                                                            //Tell the nextion to update the frame counter
      SendNextionValues;
      But_Com = 0;
      while (stepper1->isRunning()) {                                          //delay until move complete (blocking)
        delay(10);
      }
      delay (1000);                                                            //Take a second to ensure camera is still
      digitalWrite(CAM, HIGH);                                                 //Fire shutter
      delay (TlpsD);                                                           //Time the shutter is open for
      digitalWrite(CAM, LOW);                                                  //Cose the shutter and move on
      delay (500);

    }
  }



  But_Com = 5;                                                              //Tell the nextion to stop the timer
  SendNextionValues;
  But_Com = 0;

  Start_1();                                                               //send the camera back to the start
}




//******Set stepper speed based on time distance and amount od ease InOut***********
void SetSpeed() {

  //Serial.print("Enterring Setpeed");
  TLTtravel_dist = (TLTout_position - TLTin_position);                     // Distance we have to go
  factor = (TLTtravel_dist / Crono_time);                                  // fot the next three lines This formula should all be on one line but I cant get it to work!
  factor = (1 / factor);
  TLTstep_speed = abs(factor * 1000000);



  PANtravel_dist = (PANout_position - PANin_position);                     // Distance we have to go
  factor = (PANtravel_dist / Crono_time);                                  // fot the next three lines This formula should all be on one line but I cant get it to work!
  factor = (1 / factor);
  PANstep_speed = abs(factor * 1000000);

  FOCtravel_dist = (FOCout_position - FOCin_position);                     // Distance we have to go
  factor = (FOCtravel_dist / Crono_time);                                  // fot the next three lines This formula should all be on one line but I cant get it to work!
  factor = (1 / factor);
  FOCstep_speed = abs(factor * 1000000);


  switch (ease_InOut) {
    case 0:                                                                //No ease
      TLTease_Value = 4000;
      PANease_Value = 4000;
      FOCease_Value = 4000;

      break;

    case 3:
      PANtravel_dist = abs(PANout_position - PANin_position);                   // Distance we have to go in steps *3 for ramp at start and end
      factor = (1 / (PANtravel_dist / (Crono_time * 12)));                       // Distanse in steps/ (time in seconds*3) to account for ramp at start and end
      PANstep_speed = abs(factor * 1000000);
      PANease_Value = (PANtravel_dist / (Crono_time * 12));                      //Calculate the Acceloration value for the ramps total distance/ time *12


      TLTtravel_dist = abs(TLTout_position - TLTin_position);
      factor = (1 / (TLTtravel_dist / (Crono_time * 12)));
      TLTstep_speed = abs(factor * 1000000);
      TLTease_Value = (TLTtravel_dist / (Crono_time * 12));


      //      FOCtravel_dist = abs(FOCout_position - FOCin_position);
      //      if (FOCtravel_dist != 0) {
      //        factor = (1 / (FOCtravel_dist / (Crono_time * 3)));
      //        FOCstep_speed = abs(factor * 1000000);
      //        FOCease_Value = (FOCtravel_dist / (Crono_time * 3));
      //      } else {
      FOCstep_speed = 500;
      FOCease_Value = 500;
      //      }

      break;

    case 2:
      PANtravel_dist = abs(PANout_position - PANin_position);                   // Distance we have to go in steps *3 for ramp at start and end
      factor = (1 / (PANtravel_dist / (Crono_time * 12)));                       // Distanse in steps/ (time in seconds*3) to account for ramp at start and end
      PANstep_speed = abs(factor * 1000000);
      PANease_Value = (PANtravel_dist / (Crono_time * 12));                      //Calculate the Acceloration value for the ramps total distance/ time *12
      PANease_Value = ( PANease_Value * 2);

      TLTtravel_dist = abs(TLTout_position - TLTin_position);
      factor = (1 / (TLTtravel_dist / (Crono_time * 12)));
      TLTstep_speed = abs(factor * 1000000);
      TLTease_Value = (TLTtravel_dist / (Crono_time * 12));
      TLTease_Value = (TLTease_Value * 2);

      //      FOCtravel_dist = abs(FOCout_position - FOCin_position);
      //      if (FOCtravel_dist != 0) {
      //        factor = (1 / (FOCtravel_dist / (Crono_time * 3)));
      //        FOCstep_speed = abs(factor * 1000000);
      //        FOCease_Value = (FOCtravel_dist / (Crono_time * 3));
      //      } else {
      FOCstep_speed = 500;
      FOCease_Value = 500;
      //      }
      break;

    case 1:
      PANtravel_dist = abs(PANout_position - PANin_position);                   // Distance we have to go in steps *3 for ramp at start and end
      factor = (1 / (PANtravel_dist / (Crono_time * 12)));                       // Distanse in steps/ (time in seconds*3) to account for ramp at start and end
      PANstep_speed = abs(factor * 1000000);
      PANease_Value = (PANtravel_dist / (Crono_time * 12));                      //Calculate the Acceloration value for the ramps total distance/ time *12
      PANease_Value = ( PANease_Value * 3);

      TLTtravel_dist = abs(TLTout_position - TLTin_position);
      factor = (1 / (TLTtravel_dist / (Crono_time * 12)));
      TLTstep_speed = abs(factor * 1000000);
      TLTease_Value = (TLTtravel_dist / (Crono_time * 12));
      TLTease_Value = (TLTease_Value * 3);

      //      FOCtravel_dist = abs(FOCout_position - FOCin_position);
      //      if (FOCtravel_dist != 0) {
      //        factor = (1 / (FOCtravel_dist / (Crono_time * 3)));
      //        FOCstep_speed = abs(factor * 1000000);
      //        FOCease_Value = (FOCtravel_dist / (Crono_time * 3));
      //      } else {
      FOCstep_speed = 500;
      FOCease_Value = 500;
      //      }
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
    stepper1->setSpeedInUs(200);             //Larger value is slower!
    stepper1->setAcceleration (abs(TiltJoySpeed) * 2);

    if (TiltJoySpeed < 0) {
      stepper1->runBackward();
      Serial.println("TiltStepper is Backwords");
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
        stepper1->setAcceleration (abs(TiltJoySpeed));
        stepper1->applySpeedAcceleration();
        if (abs(tilt - tilt_AVG) < 250) {
          Serial.println("TiltStepper has been stoped");
          stepper1->forceStopAndNewPosition(stepper1->getCurrentPosition());
          //stepper1->stopMove();
          return;
        }
      }
    }
  }

  if (abs( PanJoySpeed) > 250) {
    stepper2->setSpeedInUs(4000);                             //Larger value is slower!
    stepper2->setAcceleration (abs(PanJoySpeed) / 6);
    if (PanJoySpeed < 0) {
      stepper2->runBackward();
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
  return;
}

void ReadMount() {                                         //Listen for PanTilt mount buttons and execute
  stepper2->setSpeedInUs(100);                             //Larger value is slower!
  stepper2->setAcceleration(4000);

  if (digitalRead(Mount_PIN) == LOW) {
    delay(100); // Debounce the button
    while  (digitalRead(Mount_PIN) == LOW) {
      Mount=1;
      stepper2-> runForward();
    }
  }
  if (digitalRead(Mount_PIN) == HIGH && Mount==1) {
    stepper2->forceStopAndNewPosition(0);                  //Stops dead
    Mount=0;
  }

  if (digitalRead(DisMount_PIN) == LOW) {
    delay(100); // Debounce the button
    while  (digitalRead(DisMount_PIN) == LOW) {
     Mount=1;
      stepper2-> runBackward();
    }
  }

  if (digitalRead(DisMount_PIN)== HIGH && Mount==1) {
    stepper2->forceStopAndNewPosition(0);                  //Stops dead
    Mount=0;
  }

}
