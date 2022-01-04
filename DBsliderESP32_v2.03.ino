#include <SD.h>

//#include <Boards.h>

#include <FastAccelStepper.h>
#include "esp_task_wdt.h"
#include <AS5600.h>

#include <esp_now.h>
#include <trigger.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <Wire.h>

#include "algorithm"

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

//Ps3 variables
#define SLIDE_ACCELERATION 8000
#define SLIDE_SPEED 2500
unsigned long previousMillis = 0;
const long debouncedelay = 100;
float slidespeed_current = 0;
float speedfactor = 1;
float accelerationfactor = 1;
int slide_is_moving;



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

//long Key1 = 2000;
//long Key2 = 4000;
//long Key3 = 6000;
//long Key4 = 8000;
//long Key5 = 10000;


long Rail_Length = 500;                     //Edit this to use any rail length you require in mm
long Max_Steps = 0;                         //Variable used to set the max number of steps for the given leangth of rail
long set_speed = 0;
long in_position = 0;                       // variable to hold IN position for slider
long out_position = 0;                      // variable to hold OUT position for slider
//long travel_dist = 0;
long step_speed = 0;                        // default travel speed between IN and OUT points
int Crono_time = 10;
int crono_hours = 0;
int crono_minutes = 0;
int crono_seconds = 0;
String DBFirmwareVersion = "frm V:32.00";
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
int TlpsM;

int ResetEncoder = 0;
int Slider = 1;                                             //Is the Slider  present 1 or 0
int TurnT;                                              //Is the turntable present 1 or 0
int PT;                                             //Is the Pan Tilt present 1 or 0
//Sequencer
int K1A;                                             //Actual interpolated Accel values
int K2A;
int K3A;
int K4A;
int K5A;
int K6A;

int k1a = 0;                                          //Nextion values for Accel 1-3
int k2a = 0;
int k3a = 0;
int k4a = 0;
int k5a = 0;
int k6a = 0;

int k1P;                                            //Nextion button state
int k2P;
int k3P;
int k4P;
int k5P;
int k6P;

int k1s = 50;                                       //Nextion speed as a percentage
int k2s = 50;
int k3s = 50;
int k4s = 0;
int k5s = 0;
int k6s = 0;

int k1s_speed;                                      //Actual interpolated speeds
int k2s_speed;
int k3s_speed;
int k4s_speed;
int k5s_speed;
int k6s_speed;
int k1_position;
int k2_position;
int k3_position;
int k4_position;
int k5_position;
int k6_position;
int SEQmin = 4000;                                        //Min slowest speed for sequencer larger number slower speed
int  SEQmoves = 2;
int Last_key;
int SEQ = 0;
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
int k1sH;
int k2sH;
int k3sH;
int k4sH;
int k5sH;
int k6sH;

int k1aH;                                         //Nextion values for Accel 1-3
int k2aH;
int k3aH;
int k4aH;
int k5aH;
int k6aH;

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
//int incomingEase;
//int incomingBo;
//int incomingCrono;
//int incomingFunc;
//int incomingTlps;
//int incomingTlpsD;
//int incomingPlay;
//long incomingIn;
//long incomingOut;
//int incomingButCom;
//int incommingInP;
//int incommingOutP;
//int incommingTlpsM;

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
  int PT;                                                 // PanTilt 0,1,2 1=present 2= Trigger for bounce 0= no pantilt
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
  //Is the Pan Tilt present 1 0 0
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

  xTaskCreatePinnedToCore(core1assignments, "Core_1", 10000, NULL, 2, &C1, 0);
  delay(100);
  //  xTaskCreatePinnedToCore(core2assignments, "Core_2", 10000, NULL, 1, &C2, 1);
  //  delay(100);


}

void loop() {




  //*************************************Action Nextion Comands*****************************************
  if (InP != 0) {
    switch (InP) {
      case 1:                                                  //First Press: Request a new inpoint
        INpointSet();
        break;
      case 2:                                                  //Second Press: Record position
        INpointSet();
        //SendNextionValues();
        InP = 0;                                          //Reset Inpoint press back to 0
        break;
    }

  }
  if (OutP != 0) {

    switch (OutP) {
      case 1:                                                //First Press: Request a new inpoint
        OUTpointSet();
        break;
      case 2:                                                //Second Press: Record position
        OUTpointSet();
        //SendNextionValues();
        OutP = 0;                                        //Reset Inpoint press back to 0
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
        Travel_dist = abs(out_position - in_position);
        Tlps_step_dist = (Travel_dist / Tlps);
        Start_4();
      }
    }
    Nextion_play = 0;                                        //Reset play to 0
    SendNextionValues();
  }

  //**************************************Listen for Sequencer commands**********************************
  if (k1P != 0) {
    switch (k1P) {
      case 1:                                                  //First Press: Request a new position
        K1set();
        break;
      case 2:                                                  //Second Press: Record position

        K1set();
        k1P = 0;                                           //Reset press back to 0
        break;
    }
  }
  if (k2P != 0) {
    switch (k2P) {
      case 1:
        K2set();
        break;
      case 2:
        K2set();
        k2P = 0;
        break;
    }
  }
  if (k3P != 0) {
    switch (k3P) {
      case 1:
        K3set();
        break;
      case 2:
        K3set();
        k3P = 0;
        break;
    }
  }
  if (k4P != 0) {
    switch (k4P) {
      case 1:
        K4set();
        break;
      case 2:
        K4set();
        k4P = 0;
        break;
    }
  }
  if (k5P != 0) {
    switch (k5P) {
      case 1:
        K5set();
        break;
      case 2:
        K5set();
        k5P = 0;
        break;
    }
  }
  if (k6P != 0) {
    switch (k6P) {
      case 1:
        K6set();
        break;
      case 2:
        K6set();
        k6P = 0;
        break;
    }
  }

  if  (Nextion_play == 2) {                                    //Sequencer Play comand receved
    if (stepper1->getCurrentPosition() != k1_position)  {    // Play Button action if not at atart)
      digitalWrite(StepD, LOW);
      delay(20);
      Start_1();                                              //Go to start
    } else {                                                 //System is at the start and ready to play
      delay(20);
      digitalWrite(CAM, HIGH);                             //Start camera
      delay (200);
      digitalWrite(CAM, LOW);                              //Start camera
      SEQ = 1;
      StorePositions();
      Start_5();

    }
    Nextion_play = 0;                                        //Reset play to 0
    SEQ = 0;
    SendNextionValues();
  }






} //end loop







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
      InP = 0;
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
  switch (OutP) {
    case 1:                                                       //First Press: Request a new OUTpoint
      if (stepper1->getCurrentPosition() != (out_position)) {     //Run to Out position
        stepper1->setSpeedInUs(1000);
        stepper1->setAcceleration(500);
        stepper1->moveTo(out_position);
        while (stepper1->isRunning()) {                          //delay until move complete
          delay(10);
        }
      }
      digitalWrite(StepD, HIGH);                                  //Power down the stepper for manual positioning
      delay(10);
      break;

    case 2:                                                       //Second inpoint press take the encoder values and set as inpoint
      out_position = (S_position);
      stepper1->setCurrentPosition(out_position);                 //Make sure the stepper knows where it is
      OutP = 0;
      delay(50);
      SendNextionValues();                                        //Update nextion with new inpoint values
      Start_1();                                                  //Send the stepper back to In position
      break;
  }
  return;
}

//*********************************************************K1set*************************************
void K1set() {

  delay(10);
  switch (k1P) {
    case 1:                                                      //First Press: Request a new position
      if (stepper1->getCurrentPosition() != (k1_position)) {    //Run to  position
        stepper1->setSpeedInUs(1000);
        stepper1->setAcceleration(500);
        stepper1->moveTo(k1_position);
        while (stepper1->isRunning()) {                          //delay until move complete
          delay(10);
        }
      }
      digitalWrite(StepD, HIGH);                                 //Power down the stepper for manual positioning
      delay(10);
      k1P = 0;
      break;

    case 2:                                                      //Second press take the encoder values and set as new position
      k1_position = (S_position);
      stepper1->setCurrentPosition(k1_position);                 //Make sure the stepper knows where it is know
      k1P = 0;
      SendNextionValues();                                       //Update nextion with new values
      digitalWrite(StepD, LOW);                                 //Power UP the stepper
      delay(10);

      break;
  }
  return;
}
//*********************************************************K2set*************************************
void K2set() {

  delay(10);
  switch (k2P) {
    case 1:
      if (stepper1->getCurrentPosition() != (k2_position)) {

        if (k2_position == 0) {

        } else {
          stepper1->setSpeedInUs(1000);
          stepper1->setAcceleration(500);
          stepper1->moveTo(k2_position);
          while (stepper1->isRunning()) {
            delay(10);
          }
        }
      }
      digitalWrite(StepD, HIGH);
      delay(10);
      k2P = 0;
      break;

    case 2:
      k2_position = (S_position);
      stepper1->setCurrentPosition(k2_position);
      k2P = 0;
      SendNextionValues();
      delay(10);
      digitalWrite(StepD, LOW);                                 //Power UP the stepper
      break;
  }
  return;
}
//*********************************************************K3set*************************************
void K3set() {

  delay(10);
  switch (k3P) {
    case 1:
      if (stepper1->getCurrentPosition() != (k3_position)) {
        if (k3_position == 0) {

        } else {
          stepper1->setSpeedInUs(1000);
          stepper1->setAcceleration(500);
          stepper1->moveTo(k3_position);
          while (stepper1->isRunning()) {
            delay(10);
          }
        }
      }
      delay(50);
      digitalWrite(StepD, HIGH);
      k3P = 0;
      break;

    case 2:
      k3_position = (S_position);
      stepper1->setCurrentPosition(k3_position);
      k3P = 0;
      digitalWrite(StepD, LOW);                                 //Power UP the stepper
      delay(100);
      SendNextionValues();
      break;
  }
  return;
}
//*********************************************************K4set*************************************
void K4set() {

  delay(10);
  switch (k4P) {
    case 1:
      if (stepper1->getCurrentPosition() != (k4_position)) {
        if (k4_position == 0) {
          //Do nothing
        } else {
          stepper1->setSpeedInUs(1000);
          stepper1->setAcceleration(500);
          stepper1->moveTo(k4_position);
          while (stepper1->isRunning()) {
            delay(10);
          }
        }
      }
      delay(50);
      digitalWrite(StepD, HIGH);
      k4P = 0;
      break;

    case 2:
      k4_position = (S_position);
      stepper1->setCurrentPosition(k4_position);
      k4P = 0;
      digitalWrite(StepD, LOW);                                 //Power UP the stepper
      delay(100);
      SendNextionValues();
      break;
  }
  return;
}
//*********************************************************K5set*************************************
void K5set() {


  delay(10);
  switch (k5P) {
    case 1:
      if (stepper1->getCurrentPosition() != (k5_position)) {
        if (k5_position == 0) {

        } else {
          stepper1->setSpeedInUs(1000);
          stepper1->setAcceleration(500);
          stepper1->moveTo(k5_position);
          while (stepper1->isRunning()) {
            delay(10);
          }
        }
      }
      delay(50);
      digitalWrite(StepD, HIGH);
      k5P = 0;
      break;

    case 2:
      k5_position = (S_position);
      stepper1->setCurrentPosition(k5_position);
      k5P = 0;
      digitalWrite(StepD, LOW);                                 //Power UP the stepper
      delay(100);
      SendNextionValues();
      break;
  }
  return;
}
//*********************************************************K6set*************************************
void K6set() {

  delay(10);
  switch (k6P) {
    case 1:
      if (stepper1->getCurrentPosition() != (k6_position)) {
        if (k6_position == 0) {

        } else {
          stepper1->setSpeedInUs(1000);
          stepper1->setAcceleration(500);
          stepper1->moveTo(k6_position);
          while (stepper1->isRunning()) {
            delay(10);
          }
        }
      }
      delay(50);
      digitalWrite(StepD, HIGH);
      k6P = 0;
      break;

    case 2:
      k6_position = (S_position);
      stepper1->setCurrentPosition(k6_position);
      k6P = 0;
      digitalWrite(StepD, LOW);                                 //Power UP the stepper
      delay(100);
      SendNextionValues();
      break;
  }
  return;
}
//**********************************Homing at Start**********************************

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
    StartEncoder();
    vTaskDelay(10);
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



  k1a = int(incomingValues.k1a);
  k2a = int(incomingValues.k2a);
  k3a = int(incomingValues.k3a);
  k4a = int(incomingValues.k4a);
  k5a = int(incomingValues.k5a);
  k6a = int(incomingValues.k6a);
  k1P = int(incomingValues.k1P);
  k2P = int(incomingValues.k2P);
  k3P = int(incomingValues.k3P);
  k4P = int(incomingValues.k4P);
  k5P = int(incomingValues.k5P);
  k6P = int(incomingValues.k6P);
  k1s = int(incomingValues.k1s);                   //Sequencer keys % speed

  k1s_speed = (float(100 - k1s) / 100) * SEQmin ;
  k2s = int(incomingValues.k2s);
  k2s_speed = (float(100 - k2s) / 100) * SEQmin ;
  k3s = int(incomingValues.k3s);
  k3s_speed = (float(100 - k3s) / 100) * SEQmin ;
  k4s = int(incomingValues.k4s);
  k4s_speed = (float(100 - k4s) / 100) * SEQmin ;
  k5s = int(incomingValues.k5s);
  k5s_speed = (float(100 - k5s) / 100) * SEQmin ;
  k6s = int(incomingValues.k6s);
  k6s_speed = (float(100 - k6s) / 100) * SEQmin ;

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
  InP = int(incomingValues.InP);          //Inpoint button state 0-2
  OutP = int(incomingValues.OutP);        //Outpoint button state 0-2
  //Slider = int(incomingValues.Slider);                //The slider is the originator so here for referance only
  PT = int(incomingValues.PT);                    //Pan Tilt available 1,2 or 0
  TurnT = int(incomingValues.TurnT);              //Turntable available 1 or 0

  //  Serial.print("\nOutP set to: ");
  //  Serial.print(OutP);

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
  NextionValues.InP = InP;                    // Inpoint button state 0-2
  NextionValues.OutP = OutP;                  // Outpoint button state 0-2
  NextionValues.Slider = Slider;              // Slider present 0-1
  //NextionValues.PT = PT;                      // Pan Tilt present 0-1,2
  //NextionValues.TurnT = TurnT;                // Turntable present 0-1
  NextionValues.k1s = k1s;                    // Sequencer key speed
  NextionValues.k2s = k2s;                    // Sequencer key speed
  NextionValues.k3s = k3s;                    // Sequencer key speed
  NextionValues.k4s = k4s;                    // Sequencer key speed
  NextionValues.k5s = k5s;                    // Sequencer key speed
  NextionValues.k6s = k6s;                    // Sequencer key speed
  NextionValues.k1P = k1P;                    // k1 set button 0-2
  NextionValues.k2P = k2P;                    // k2 set button 0-2
  NextionValues.k3P = k3P;                    // k3 set button 0-2
  NextionValues.k4P = k4P;                    // k4 set button 0-2
  NextionValues.k5P = k5P;                    // k5 set button 0-2
  NextionValues.k6P = k6P;                    // k6 set button 0-2
  NextionValues.k1a = k1a;                    // Acceloration values
  NextionValues.k2a = k2a;
  NextionValues.k3a = k3a;
  NextionValues.k4a = k4a;
  NextionValues.k5a = k5a;
  NextionValues.k6a = k6a;
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(broadcastAddress, (uint8_t *) &NextionValues, sizeof(NextionValues));
}




//*********************************************ARM The SLIDER READY TO START**************************************************
void Start_1() {
  //  Arm the slider to start point ready for second play press
  digitalWrite(StepD, LOW);
  delay(100);

  stepper1->setSpeedInUs(400);
  stepper1->setAcceleration(1000);

  if (SEQ == 0) {
    stepper1->moveTo(in_position);
    delay(10);
  }
  if (SEQ == 1) {
    stepper1->moveTo(k1_position);
    delay(10);

  }
  while (stepper1->isRunning() ) {                              //delay until move complete (block)
    delay(10);
  }

  But_Com = 5;
  //Serial.print("Leaving Start1 ");
  delay(100);
  SendNextionValues();
  But_Com = 0;

}

//************************************************RUN THE SEQUENCE BASIC**************************************************************
void Start_2() {
  if (PT > 0) {
    delay(200);
  }
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
  if (PT > 0) {                                           //If PanTitlt is onboard
    delay(500);
  }
  Bounce = Bounce - 1;
  But_Com = 6;
  SendNextionValues();                                  //Send current bounce counter back to nextion
  digitalWrite(StepD, LOW);
  stepper1->setSpeedInUs(step_speed);
  stepper1->setAcceleration(Ease_Value);
  stepper1->moveTo(in_position);
  while (stepper1->isRunning()) {                         //delay until move complete (block)
    delay(10);
  }


  But_Com = 0;
  if (Bounce >= 1) {
    if (PT == 0) {                                        //If No PanTilt head is available just play
      Start_2();
    } else {
      while (PT < 2) {                                    //Else weight for the pantilt head to signal 2 befor starting next bounce move
        delay(10);
      }
      PT = 1;
      SendNextionValues;
      Start_2();
    }
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
  //  Travel_dist = (out_position - in_position);
  //  Tlps_step_dist = Travel_dist / Tlps;
  //Serial.println(stepper1->getCurrentPosition() + Tlps_step_dist);
  digitalWrite(StepD, LOW);
  step_speed = (Travel_dist / 5);
  stepper1->setSpeedInUs(500);
  stepper1->setAcceleration(500);

  while (Tlps >= 1) {
    TlpsM = 4;                                                            //Trigger pantilt head timelapse move
    SendNextionValues();
    stepper1->moveTo(stepper1->getCurrentPosition() + Tlps_step_dist);    //Current position plus one fps move
    while (stepper1->isRunning()) {                                       //delay until move complete (blocking)
      delay(10);
    }
    delay (1000);                                                         //Take a second to ensure camera is still
    digitalWrite(CAM, HIGH);                                              //Fire shutter
    delay (TlpsD);                                                        //Time the shutter is open for
    digitalWrite(CAM, LOW);                                               //Cose the shutter and move on
    delay (500);
    Tlps = Tlps - 1;
    But_Com = 7;                                                          //Tell the nextion to update the frame counter
    SendNextionValues;
    But_Com = 0;
  }
  But_Com = 5;                                                            //Tell the nextion to stop the timer
  SendNextionValues;
  But_Com = 0;
  //NextionLCD.setComponentValue("TimerStop", 0);
  //NextionLCD.setComponentText("t5", String(fps));
  Start_1();                                        //send the camera back to the start
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

  k1s = k1sH;
  k2s = k2sH;
  k3s = k3sH;
  k4s = k4sH;
  k5s = k5sH;
  k6s = k6sH;

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

      k1s = k3sH;
      k2s = k2sH;
      k3s = k1sH;

      K1A = K3AH;
      K2A = K2AH;
      K3A = K1AH;
      break;
    case 4:                                                                                   //No ease
      k1_position = k4_positionH;
      k2_position = k3_positionH;
      k3_position = k2_positionH;
      k4_position = k1_positionH;

      k1s = k4sH;
      k2s = k3sH;
      k3s = k2sH;
      k4s = k1sH;

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


      k1s = k5sH;
      k2s = k4sH;
      k3s = k3sH;
      k4s = k2sH;
      k5s = k1sH;

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

      k1s = k6sH;
      k2s = k5sH;
      k3s = k4sH;
      k4s = k3sH;
      k5s = k2sH;
      k6s = k1sH;

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

  k1sH = k1s;
  k2sH = k2s;
  k3sH = k3s;
  k4sH = k4s;
  k5sH = k5s;
  k6sH = k6s;

  K1AH = K1A;
  K2AH = K2A;
  K3AH = K3A;
  K4AH = K4A;
  K5AH = K5A;
  K6AH = K6A;
}



void Start_5() {
  //Establish how many keys are set
  if (k1s != 0) {
    LastMove = 1;
  }
  if (k2s != 0) {
    LastMove = 2;
  }
  if (k3s != 0) {
    LastMove = 3;
  }
  if (k4s != 0) {
    LastMove = 4;
  }
  if (k5s != 0) {
    LastMove = 5;
  }
  if (k6s != 0) {
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

  k1s_speed = 0;
  k2s_speed = 0;
  k3s_speed = 0;
  k4s_speed = 0;
  k5s_speed = 0;
  k6s_speed = 0;

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
  delay(100);
  KeyB = k2_position;
  KeyA = k1_position;
  Find_KeyDist();

  if (KeyDist != 0 && k2s != 0) {
    Key_Crono_time = int(float(100 / k1s) * Seq_Time);                                         //Function to find out the distance between key frames under any + - condition
    factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
    factor = (1 / factor);
    k1s_speed = abs(factor * 1000000);
    Stp_1_active = 1;
    Stp_active = 1;

    switch (k1a) {
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
    stepper1->setSpeedInUs(k1s_speed);


    switch (move1_Dest) {                                                      //Pick up how far the motor should move before it either stopes or changes direction
      case 2:
        stepper1->moveTo(k2_position);
        break;
      case 3:
        k1s_speed = k1s_speed + ((k1s_speed / 100) * 9);                        //remember the higher the speed value the slower it moves!
        stepper1->setSpeedInUs(k1s_speed);
        stepper1->moveTo(k3_position);
        break;
      case 4:
        k1s_speed = k1s_speed + ((k1s_speed / 100) * 9);
        stepper1->setSpeedInUs(k1s_speed);
        stepper1->moveTo(k4_position);
        break;
      case 5:
        k1s_speed = k1s_speed + ((k1s_speed / 100) * 9);
        stepper1->setSpeedInUs(k1s_speed);
        stepper1->moveTo(k5_position);
        break;
      case 6:
        k1s_speed = k1s_speed + ((k1s_speed / 100) * 9);
        stepper1->setSpeedInUs(k1s_speed);
        stepper1->moveTo(k6_position);
        break;
    }
    Base_accel =  K1A;
    Last_accel = K1A;
    Last_speed = k1s_speed;
    delay(1500);                                                              //Delay telling the PT the slider is moving again in case the PT is behind Quick fix!
    Slider = 2;                                                               //Tell the pantilthead the slider is moving
    SendNextionValues();
    delay(10);
    ActiveMove = 1;
  } else {
    Stp_1_active = 0;
    Slider = 1;                                                               //Tell the pantilthead the slider is not moving
    SendNextionValues();
  }
  //************************PT move*****************************************************
  if (PT == 2 ) {
    ActiveMove = 1;
    Stp_active = 1;
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
      Slider = 1;                                                                  //Tell the pantilthead the slider is finished its move
      SendNextionValues();
      delay(100);
      Stp_1_active = 0;

    }

    if (Stp_1_active == 0 && PT != 2 ) {                     //Final test to see if all steppers have finished move
      Slider = 1;                                                                                      //Tell the pantilthead the slider is finished its move
      SendNextionValues();
      delay(100);
      Stp_active = 0;

    }
  }

  //**********************************************************************************************************
  //****************************************************move2-3***********************************************
  //**********************************************************************************************************


  KeyB = k3_position;
  KeyA = k2_position;
  Find_KeyDist();

  if (KeyDist != 0 && k3s != 0) {

    Key_Crono_time = int(float(100 / k2s) * Seq_Time);                                         //Function to find out the distance between key frames under any + - condition
    factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
    factor = (1 / factor);
    k2s_speed = abs(factor * 1000000);
    Stp_1_active = 1;
    Stp_active = 1;

    switch (k2a) {
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

    if (stepper1->isRunning() == true && move1_Dest > 2 && move2_Rst != 1 ) {     //If the stepper is running on from last move
      stepper1->setAcceleration(K2A);
      stepper1->setSpeedInUs(k2s_speed);
      stepper1-> applySpeedAcceleration();
      delay(10);
    } else {
      stepper1->setAcceleration(K2A);                                             //If starting fresh Max accel to catch up on turnarround
      stepper1->setSpeedInUs(k2s_speed);

      switch (move2_Dest) {                                                       //Pick up how far the motor should move before it either stopes or changes direction
        case 3:
          stepper1->moveTo(k3_position);
          break;
        case 4:
          k2s_speed = k2s_speed + ((k2s_speed / 100) * 9);
          stepper1->setSpeedInUs(k2s_speed);
          stepper1->moveTo(k4_position);
          break;
        case 5:
          k2s_speed = k2s_speed + ((k2s_speed / 100) * 9);
          stepper1->setSpeedInUs(k2s_speed);
          stepper1->moveTo(k5_position);
          break;
        case 6:
          k2s_speed = k2s_speed + ((k2s_speed / 100) * 9);
          stepper1->setSpeedInUs(k2s_speed);
          stepper1->moveTo(k6_position);
          break;
      }
      delay(1500);                                                               //Beta Patch delay slider is moving slightly faster than PanTilt!
      Slider = 2;                                                               //Tell the pantilthead the slider is moving
      SendNextionValues();
      delay(10);
    }
    Base_accel = K2A;
    Last_accel = K2A;
    Last_speed = k2s_speed;
    ActiveMove = 2;
  } else {
    Stp_1_active = 0;
    Slider = 1;                                                               //Tell the pantilthead the slider is moving
    SendNextionValues();
  }

  //************************PT move*****************************************************
  if (PT == 2 ) {
    ActiveMove = 1;
    Stp_active = 1;
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
      Slider = 1;                                                                                      //Tell the pantilthead the slider is finished its move
      SendNextionValues();
      delay(100);
      Stp_1_active = 0;
    }

    if (Stp_1_active == 0 && PT != 2 ) {                                                            //Final test to see if all steppers have finished move
      Slider = 1;                                                                                      //Tell the pantilthead the slider is finished its move
      SendNextionValues();
      delay(100);
      Stp_active = 0;
    }
  }

  //**********************************************************************************************************
  //****************************************************move3-4***********************************************
  //**********************************************************************************************************
  if (k4s != 0) {

    //************************Tlt move*****************************************************
    KeyB = k4_position;
    KeyA = k3_position;
    Find_KeyDist();

    if (KeyDist != 0 && k4s != 0) {
      Key_Crono_time = int(float(100 / k3s) * Seq_Time);                                              //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                                  //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      k3s_speed = abs(factor * 1000000);
      Stp_1_active = 1;
      Stp_active = 1;


      switch (k3a) {
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
        k3s_speed = k3s_speed + ((k3s_speed / 100) * 9);
        stepper1->setSpeedInUs(k3s_speed);
        stepper1-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper1->setAcceleration(K3A);                                                              //If starting fresh Max accel to catch up on turnarround
        stepper1->setSpeedInUs(k3s_speed);

        switch (move3_Dest) {                                                                    //Pick up how far the motor should move before it either stopes or changes direction

          case 4:
            //k3s_speed = k3s_speed + ((k3s_speed / 100) * 9);
            stepper1->setSpeedInUs(k3s_speed);
            stepper1->moveTo(k4_position);
            break;
          case 5:
            k3s_speed = k3s_speed + ((k3s_speed / 100) * 9);
            stepper1->setSpeedInUs(k3s_speed);
            stepper1->moveTo(k5_position);
            break;
          case 6:
            k3s_speed = k3s_speed + ((k3s_speed / 100) * 9);
            stepper1->setSpeedInUs(k3s_speed);
            stepper1->moveTo(k6_position);
            break;
        }
        delay(1000);
        Slider = 2;                                                               //Tell the pantilthead the slider is moving
        SendNextionValues();
        delay(10);
      }
      Base_accel = K3A;
      Last_accel = K3A;
      Last_speed = k3s_speed;
      ActiveMove = 3;
    } else {
      Slider = 1;                                                               //Tell the pantilthead the slider is not moving
      SendNextionValues();
      Stp_1_active = 0;
    }

    //************************PT move*****************************************************
    if (PT == 2 ) {
      ActiveMove = 1;
      Stp_active = 1;
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
        Slider = 1;                                                                                      //Tell the pantilthead the slider is finished its move
        SendNextionValues();
        Stp_1_active = 0;
      }

      if (Stp_1_active == 0 && PT != 2  ) {                             //Final test to see if all steppers have finished move
        Slider = 1;                                                                                      //Tell the pantilthead the slider is finished its move
        SendNextionValues();
        delay(100);
        Stp_active = 0;
      }

    }
  }

  //**********************************************************************************************************
  //****************************************************move4-5***********************************************
  //**********************************************************************************************************
  if (k5s != 0) {

    KeyB = k5_position;
    KeyA = k4_position;
    Find_KeyDist();

    if (KeyDist != 0 && k5s != 0) {
      Key_Crono_time = int(float(100 / k4s) * Seq_Time);                                     //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      k4s_speed = abs(factor * 1000000);
      Stp_1_active = 1;
      Stp_active = 1;

      switch (k4a) {
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
        k4s_speed = k4s_speed + ((k4s_speed / 100) * 9);
        stepper1->setSpeedInUs(k4s_speed);
        stepper1-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper1->setAcceleration(K4A);                                                              //If starting fresh Max accel to catch up on turnarround
        stepper1->setSpeedInUs(k4s_speed);

        switch (move4_Dest) {                                                                     //Pick up how far the motor should move before it either stopes or changes direction
          case 5:
            stepper1->moveTo(k5_position);
            break;
          case 6:
            k4s_speed = k4s_speed + ((k4s_speed / 100) * 9);
            stepper1->setSpeedInUs(k4s_speed);
            stepper1->moveTo(k6_position);
            break;
        }
        delay(1000);
        Slider = 2;                                                               //Tell the pantilthead the slider is moving
        SendNextionValues();
        delay(10);
      }
      Base_accel = K4A;
      Last_accel = K4A;
      Last_speed = k4s_speed;
      ActiveMove = 4;
    } else {
      Stp_1_active = 0;
    }

    //************************PT move*****************************************************
    if (PT == 2 ) {
      ActiveMove = 1;
      Stp_active = 1;
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
        Slider = 1;                                                                                      //Tell the pantilthead the slider is finished its move
        SendNextionValues();
        Stp_1_active = 0;
      }

      if (Stp_1_active == 0 && PT != 2 ) {                                                                   //Final test to see if all steppers have finished move
        Slider = 1;                                                                                      //Tell the pantilthead the slider is finished its move
        SendNextionValues();
        delay(100);
        Stp_active = 0;
      }
    }
  }

  //**********************************************************************************************************
  //****************************************************move5-6***********************************************
  //**********************************************************************************************************
  if (k6s != 0) {



    KeyB = k6_position;
    KeyA = k5_position;
    Find_KeyDist();
    if (KeyDist == 0 ) {
      KeyDist = -10;
    }
    if (KeyDist != 0 && k6s != 0) {
      Key_Crono_time = int(float(100 / k5s) * Seq_Time);                                     //Function to find out the distance between key frames under any + - condition
      factor = abs(KeyDist) / float(Key_Crono_time);                                              //Speed required for cost time in the middle of the move
      factor = (1 / factor);
      k5s_speed = abs(factor * 1000000);
      Stp_1_active = 1;
      Stp_active = 1;

      switch (k5a) {
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
        k5s_speed = k5s_speed + ((k5s_speed / 100) * 9);
        stepper1->setSpeedInUs(k5s_speed);
        stepper1-> applySpeedAcceleration();
        delay(10);
      } else {
        stepper1->setAcceleration(K5A);                                                  //If starting fresh Max accel to catch up on turnarround
        stepper1->setSpeedInUs(k5s_speed);
        stepper1->moveTo(k6_position);

        delay(1000);
        Slider = 2;                                                               //Tell the pantilthead the slider is moving
        SendNextionValues();
      }
      Base_accel = K5A;
      Last_accel = K5A;
      Last_speed = k5s_speed;
      ActiveMove = 5;
    } else {
      Stp_1_active = 0;
    }
    //************************PT move*****************************************************
    if (PT == 2 ) {
      ActiveMove = 1;
      Stp_active = 1;
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
        Slider = 1;                                                                                      //Tell the pantilthead the slider is finished its move
        SendNextionValues();
        Stp_1_active = 0;
      }
      if (Stp_1_active == 0 && PT != 2) {       //Final test to see if all steppers have finished move
        Slider = 1;                                                                                      //Tell the pantilthead the slider is finished its move
        SendNextionValues();
        delay(100);
        Stp_active = 0;
      }

    }
  }
  //End Game for Sequencer moves
  if (Bounce >= 1) {
    BounceActive = 1;                                                         //Tell the system that bounce is active
    Bounce = Bounce - 1;
    But_Com = 6;
    delay(300);                                                               //This delay put into give the PT head a chance on final bounce
    SendNextionValues();

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
      BounceActive = 0;
      Start_6();
    } else {
      But_Com = 5;                                                                //Stop the timer
      SendNextionValues();
      But_Com = 0;
      delay(10);
      But_Com = 6;                                                                //Update Nextion bounce
      Slider = 1;                                                                 //Set slider to normal
      SendNextionValues();
      But_Com = 0;
      BounceReturn = 1;                                                            //Forward move restored
      delay(10);
      digitalWrite(CAM, HIGH);                                                     //Stop camera
      delay (200);
      digitalWrite(CAM, LOW);
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
  Travel_dist = (out_position - in_position);                     // Distance we have to go
  factor = (Travel_dist / Crono_time);                                  // fot the next three lines This formula should all be on one line but I cant get it to work!
  factor = (1 / factor);
  step_speed = abs(factor * 1000000);

  switch (ease_InOut) {
    case 0:                                                                //No ease
      Ease_Value = 4000;
      break;

    case 3:
      Travel_dist = abs(out_position - in_position);
      factor = (1 / (Travel_dist / (Crono_time * 12)));
      step_speed = abs(factor * 1000000);
      Ease_Value = (Travel_dist / (Crono_time * 12));
      break;

    case 2:
      Travel_dist = abs(out_position - in_position);
      factor = (1 / (Travel_dist / (Crono_time * 12)));
      step_speed = abs(factor * 1000000);
      Ease_Value = (Travel_dist / (Crono_time * 12));
      Ease_Value = (Ease_Value * 2);
      break;

    case 1:
      Travel_dist = abs(out_position - in_position);
      factor = (1 / (Travel_dist / (Crono_time * 12)));
      step_speed = abs(factor * 1000000);
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
  if (k2s != 0) {
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

  switch (move1_Dest) {                                //Find the full move start if move one only took us to 2 then 2-3 must be a restart for a change in direction


    case 2:
      move2_Dest = 2;
      //*********************k1-k2**************************
      if (move2_Dest == 2 && k3s != 0) {
        if (k2_position > k1_position) {                      //check direction if + or -
          k1_k2_Dir = 1;
        }
        if (k2_position < k1_position) {
          k1_k2_Dir = -1;
        }
        if (k2_position == k1_position) {
          k1_k2_Dir = 0;
        }

        //*********************k2-k3**************************
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
        if ((k2_k3_Dir != 0) && (k1_k2_Dir == k2_k3_Dir)) {                                                                 // if no direction change
          move2_Dest = 3;
        } else {
          move2_Dest = 3;
          move3_Rst = 1;
        }
      }
      //*********************k3-k4**************************
      if (move2_Dest == 3 && k4s != 0) {
        if (k4_position > k3_position) {                      //check direction if + or -
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
      if (move2_Dest == 4 && k5s != 0) {
        if (k5_position > k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                        // if no direction change
          move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (move2_Dest == 5 && k6s != 0) {
        if (k6_position > k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          move2_Dest = 6;
        }
      }
      break;
    case 3:
      move2_Dest = 3;


      //***********k2-k3 move*************************
      if (move2_Dest == 3 && k4s != 0) {
        if (k3_position > k2_position) {                      //check direction if + or -
          k2_k3_Dir = 1;
        }
        if (k3_position < k2_position) {
          k2_k3_Dir = -1;
        }
        if (k3_position == k2_position) {
          k2_k3_Dir = 0;
        }

        //***********k3-k4 move*************************
        if (k4_position > k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (k4_position < k3_position) {
          k3_k4_Dir = -1;
        }
        if (k4_position == k3_position) {
          k3_k4_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {                        // if no direction change
          move2_Dest = 4;
        }
      }
      //***********k4-k5 move*************************
      if (move2_Dest == 4 && k5s != 0) {
        if (k5_position > k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                        // if no direction change
          move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (move2_Dest == 5 && k6s != 0) {

        if (k6_position > k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          move2_Dest = 6;
        }
      }
      break;
    case 4:
      move2_Dest = 4;

      //***********k3-k4 move*************************

      if (move2_Dest == 4 && k5s != 0) {
        if (k4_position > k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (k4_position < k3_position) {
          k3_k4_Dir = -1;
        }
        if (k4_position == k3_position) {
          k3_k4_Dir = 0;
        }
        //***********k4-k5 move*************************
        if (k5_position > k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          move2_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (move2_Dest == 5 && k6s != 0) {
        if (k6_position > k5_position) {                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          move2_Dest = 6;
        }
      }
      break;

    case 5:
      move2_Dest = 5;
      //***********k4-k5 move*************************

      if (move2_Dest == 5 && k6s != 0) {
        if (k5_position > k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }

        //***********k5-k6 move*************************
        if (k6_position > k5_position) {                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
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

  switch (move2_Dest) {                                                        //Find the full move start

    case 3:

      move3_Dest = 3;
      //***********k2-k3 move*************************
      if (move3_Dest == 3 && k4s != 0) {
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
        if ((k3_k4_Dir != 0) && (k2_k3_Dir == k3_k4_Dir)) {                          // if no direction change
          move3_Dest = 4;
        } else {
          move3_Dest = 4;
          move4_Rst = 1;
        }
      }
      //***********k4-k5 move*************************
      if (move3_Dest == 4 && k5s != 0) {
        if (k5_position > k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          move3_Dest = 5;
        }
      }
      //***********Test for moving on 3-6
      if (move3_Dest == 5 && k6s != 0) {
        if (k6_position > k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = 0;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          move3_Dest = 6;
        }
      }
      break;
    case 4:
      move3_Dest = 4;

      //***********k3-k4 move*************************
      if (move3_Dest == 4 && k5s != 0) {
        if (k4_position > k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (k4_position < k3_position) {
          k3_k4_Dir = -1;
        }
        if (k4_position == k3_position) {
          k3_k4_Dir = 0;
        }

        //***********k4-k5 move*************************

        if (k5_position > k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          move3_Dest = 5;
        }
      }
      //***********k5-k6 move*************************
      if (move3_Dest == 5 && k6s != 0) {
        if (k6_position > k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          move3_Dest = 6;
        }
      }
      break;
    case 5:
      move3_Dest = 5;
      //***********k4-k5 move*************************
      if (move3_Dest == 5 && k6s != 0) {
        if (k5_position > k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************
        if (k6_position > k5_position) {                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
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

  switch (move3_Dest) {                                //Find the full move start


    case 4:

      move4_Dest = 4;


      //***********k3-k4 move*************************is it going on to 6
      if (move4_Dest == 4 && k5s != 0) {
        if (k4_position > k3_position) {                      //check direction if + or -
          k3_k4_Dir = 1;
        }
        if (k4_position < k3_position) {
          k3_k4_Dir = -1;
        }
        if (k4_position == k3_position) {
          k3_k4_Dir = 0;
        }
        //***********k4-k5 move*************************is it going on to 6

        if (k5_position > k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k4_k5_Dir != 0) && (k3_k4_Dir == k4_k5_Dir)) {                          // if no direction change
          move4_Dest = 5;
        } else {
          move4_Dest = 5;
          move5_Rst = 1;
        }
      }
      //***********k5-k6 move*************************
      if (move4_Dest == 5 && k6s != 0) {
        if (k6_position > k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          move4_Dest = 6;
        }
      }
      break;
    case 5:
      move4_Rst = 1;
      move4_Dest = 5;
      //***********k4-k5 move*************************is it going on to 6
      if (move4_Dest == 5 && k6s != 0) {
        if (k5_position > k4_position) {                      //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************

        if (k6_position > k5_position) {                       //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
          move4_Dest = 6;
        }
      }
      break;
  }
  //********************* Move5 plan***********************************

  switch (move4_Dest) {                                                       //Find the full move start

    case 5:

      move4_Dest = 5;

      //***********k4-k5 move*************************is it going on to 6
      if (move5_Dest == 5 && k6s != 0) {
        if (k5_position > k4_position) {                                    //check direction if + or -
          k4_k5_Dir = 1;
        }
        if (k5_position < k4_position) {
          k4_k5_Dir = -1;
        }
        if (k5_position == k4_position) {
          k4_k5_Dir = 0;
        }
        //***********k5-k6 move*************************is it going on to 6

        if (k6_position > k5_position) {                                      //check direction if + or -
          k5_k6_Dir = 1;
        }
        if (k6_position < k5_position) {
          k5_k6_Dir = -1;
        }
        if (k6_position == k5_position) {
          k5_k6_Dir = 0;
        }
        //***************look for direction change**********************
        if ((k5_k6_Dir != 0) && (k4_k5_Dir == k5_k6_Dir)) {                          // if no direction change
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

//*********************************************Error Reset***************************************//
//int ErrorReset() {
//  //NextionLCD.setComponentValue("TimerStop", 999);
//  //return;
//}
