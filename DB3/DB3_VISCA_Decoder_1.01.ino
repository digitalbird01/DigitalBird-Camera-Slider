/*

   Digital Bird Visca Decoder- POE ethernet UDP to UART export v1.0 Sept 2023
change Log:
1) Preset speed added to viscacommand set allowing you to modify the speed of preset moves 
2) Bug Fix where IP was not always being received correctly from the upper head CPU at startup
*/

#define DEBUG_ETHERNET_WEBSERVER_PORT       Serial
// Debug Level from 0 to 4
#define _ETHERNET_WEBSERVER_LOGLEVEL_       3
#include <WebServer_WT32_ETH01.h>
#include <HardwareSerial.h>
HardwareSerial UARTport(2); // use UART2
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFiClient.h>
#include "sntp.h"


// Select the IP address according to your local network
IPAddress myIP(192, 168, 137, 80);
IPAddress myGW(192, 168, 137, 1);
IPAddress mySN(255, 255, 255, 0);
IPAddress myDNS(192, 168, 137, 1);   //Adress to send out on

char timeServer[]         = "time.nist.gov";  // NTP server
unsigned int localPort    = 1259;             // local port to listen for UDP packets

const int NTP_PACKET_SIZE = 48;       // NTP timestamp is in the first 48 bytes of the message
const int UDP_TIMEOUT     = 10;     // timeout in miliseconds to wait for an UDP packet to arrive

byte packetBuffer[NTP_PACKET_SIZE];   // buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

// send an NTP request to the time server at the given address


//Example return codes
byte acknowledge[3] = {0x90, 0x41, 0xFF}; // Return for accept command
byte acceptC[3] = {0x90, 0x38, 0xFF}; // Return for accept command

//Required for inital camera setup
byte PTposition[12] = {0x90, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF}; //Pan Tilt current position reply 11
byte ZoomPosition[7] = {0x90, 0x50, 0x00, 0x00, 0x00, 0x00, 0xFF}; //Zoom position reply = Zoom x1
byte FocusPosition[7] = {0x90, 0x50, 0x00, 0x00, 0x00, 0x00, 0xFF}; //Focus position reply = infinity
byte Confirm[3] = {0x90, 0x42, 0xFF}; //Accept comands return
byte complete[3] = {0x90, 0x51, 0xFF}; // Return code for Command Complete

const byte numChars = 4;       //For uart reception
char receivedChars[numChars];   // An array to store the received data from UART

boolean newData = false;
int Tally;
int Pose;
long TAngle;
long ZAngle;
long FAngle;
long PAngle;
char P1[2];
char P2[2];
char P3[2];
char P4[2];
char P5[2];
char P6[2];
char P7[2];
char P8[2];
char P9[2];
int ViscaPacketSize;
int ViscaMove;
int ViscaPanSpeed;
int ViscaTiltSpeed;
int ViscaFocusSpeed;
int ViscaZoomSpeed;
int Speed;                                  //Generic holder for speed
int PanSpeed;
int TiltSpeed;
int FocusSpeed;
int ZoomSpeed;
int PoseNumber;
int PoseSpeed;
const byte bufferSize = 11;
char serialBuffer[bufferSize];
byte bufferIndex = 0;
char EOL = '\n';
bool hasData = false;

int IP1 = 0;
int IP2 = 0;
int IP3 = 0;
int IP4 = 0;
int IPGW = 0;
int UDP = 0;
int ViscaComand;
int stopcheck;                                   // For checking when a STOP command is received (0)
int checksum;                                    // For Calculating Checksum. Sum of the payload bytes (bytes 1 through 8) in the message
int ByteNumber;                                  // Counter for reading the serial buffer
const char * returnIP;
int returnPort;
byte byteReceived[9];
int request = 0;
char *payloadString;
unsigned long entry;
int IPCheck;

String NumHex4;
String NumHex5;
String NumHex6;
String NumHex7;
String NumHex8;
String NumHex9;
String NumHex10;
String NumHex11;
String NumHex12;
String NumHex13;
String NumHex14;

int PAngleIn;
int TAngleIn;
int ZAngleIn;
int FAngleIn;
void setup() {




  Serial.begin(115200);
  UARTport.begin(9600, SERIAL_8N1, 14, 15);

  while (IP1 != 192) {
    GetIP();
  }
  // Select the IP address according to your local network
  IPAddress myIP(IP1, IP2, IP3, IP4);
  IPAddress myGW(IP1, IP2, IP3, IPGW);
  IPAddress myDNS(IP1, IP2, IP3, IPGW);
  IPAddress mySN(255, 255, 255, 0);
Serial.print("\nIP address found from head set to " + String(IP1));
Serial.print("\nIP address found from head set to " + String(IP2));
Serial.print("\nIP address found from head set to " + String(IP3));
Serial.print("\nIP address found from head set to " + String(IP4));
  while (!Serial);


  Serial.print("\nStarting UdpNTPClient on " + String(ARDUINO_BOARD));
  Serial.println(" with " + String(SHIELD_TYPE));
  Serial.println(WEBSERVER_WT32_ETH01_VERSION);


  WT32_ETH01_onEvent();                                   // To be called before ETH.begin()
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
  ETH.config(myIP, myGW, mySN, myDNS);
  WT32_ETH01_waitForConnect();
  Udp.begin(localPort);




}

void loop() {
  //if (IPCheck==0){
  // listenForUART();
  //}

  // wait for a reply for UDP_TIMEOUT miliseconds
  unsigned long startMs = millis();
  while (!Udp.available() && (millis() - startMs) < UDP_TIMEOUT) {}




  int packetSize = Udp.parsePacket();                     // if there's data available, read a packet
  if (packetSize) {

    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    Serial.println();
    ViscaPacketSize = packetSize;


    Serial.print(", Last Paket count: ");
    Serial.println (ViscaPacketSize);
    Serial.print(packetBuffer[0]);
    Serial.print(", ");
    Serial.print(packetBuffer[1]);
    Serial.print(", ");
    Serial.print(packetBuffer[2]);
    Serial.print(", ");
    Serial.print(packetBuffer[3]);
    Serial.print(", ");
    Serial.print(packetBuffer[4]);
    Serial.print(", ");
    Serial.print(packetBuffer[5]);
    Serial.print(", ");
    Serial.print(packetBuffer[6]);
    Serial.print(", ");
    Serial.print(packetBuffer[7]);
    Serial.print(", ");
    Serial.print(packetBuffer[8]);
    Serial.println("");
    Serial.print (packetBuffer[9]);
    Serial.print(", ");
    Serial.print(packetBuffer[10]);
    Serial.print(", ");
    Serial.print(packetBuffer[11]);
    Serial.print(", ");
    Serial.print(packetBuffer[12]);
    Serial.print(", ");
    Serial.print(packetBuffer[13]);
    Serial.print(", ");
    Serial.print(packetBuffer[14]);
    Serial.print(", ");
    Serial.print(packetBuffer[15]);
    Serial.print(", ");
    Serial.print(packetBuffer[16]);
    Serial.print(", ");
    Serial.print(packetBuffer[17]);
    Serial.println("");




    //*********************Establish a DB move code fron the two packets before FF using Decimal Values********************
    if (ViscaPacketSize == 5) {

      ViscaComand = packetBuffer[2] * 10 + (packetBuffer[3]);
    }
    if (ViscaPacketSize == 6) {
      ViscaComand = (packetBuffer[3] * 100 + (packetBuffer[4]));
      if ((packetBuffer[1] + packetBuffer[2] + packetBuffer[3]) == 14) {
        ViscaComand = 14;
        Tally = packetBuffer[4];
      }
    if ((packetBuffer[1] + packetBuffer[2] + packetBuffer[3]) == 8) {
        ViscaComand = 137;
        PoseSpeed = packetBuffer[4];
      }
    
    }

    if (ViscaPacketSize == 7) {
      if (ViscaComand = (packetBuffer[2] == 4)) {
        ViscaComand = (packetBuffer[2] * 10 + (packetBuffer[4]));
        PoseNumber = (packetBuffer[5]);
      } else {
        ViscaComand = packetBuffer[2];
      };
      if (ViscaComand == 40 || ViscaComand == 41 || ViscaComand == 42) {                      //If Command is a pose find out the pose requested
        Pose = (packetBuffer[5]);
      }

    }
    if (ViscaPacketSize == 8) {
      ViscaComand = (packetBuffer[2] * 10 + packetBuffer[4]);
    }
    if (ViscaPacketSize == 9) {
      ViscaComand = (packetBuffer[6] * 10 + packetBuffer[7]);
      PanSpeed = ((packetBuffer[4]));
      TiltSpeed = ((packetBuffer[5]));
      if (packetBuffer[3] == 71) {
        ViscaComand = 71;
      }
    }

    //For OBS padded commands
    if (ViscaPacketSize == 13) {
      ViscaComand = (packetBuffer[10] * 10 + packetBuffer[11]);       // All 13 comands except the absolute position commands
      if (packetBuffer[1] + packetBuffer[2] + packetBuffer[3] == 76) { //Then Abolute zoom/Focus move requested eg vMix position
        ViscaComand = 76;
        NumHex4 = String(packetBuffer[4], HEX);
        NumHex5 = String(packetBuffer[5], HEX);
        NumHex6 = String(packetBuffer[6], HEX);
        NumHex7 = String(packetBuffer[7], HEX);
        NumHex8 = String(packetBuffer[8], HEX);
        NumHex9 = String(packetBuffer[9], HEX);
        NumHex10 = String(packetBuffer[10], HEX);
        NumHex11  = String(packetBuffer[11], HEX);
      }
      if (packetBuffer[1] + packetBuffer[2] + packetBuffer[3] == 77) { //Then Abolute Focus move requested eg vMix position
        ViscaComand = 77;
      }
    }
    if (ViscaPacketSize == 14) {
      ViscaComand = (packetBuffer[11] * 100 + packetBuffer[12]);

    }
    if (ViscaPacketSize == 15) {
      if (ViscaComand = (packetBuffer[10] == 4)) {
        ViscaComand = (packetBuffer[10] * 10 + packetBuffer[12]);
        PoseNumber = (packetBuffer[13]);
      } else {
        ViscaComand = packetBuffer[10];
      }
      if (ViscaComand == 40 || 41 || 42) {                          //If Command is a pose find out the pose requested
        Pose = (packetBuffer[13]);
      }
      if (packetBuffer[1] + packetBuffer[2] + packetBuffer[3] == 9) { //Then Abolute move requested eg vMix position
        ViscaComand = 9;
        PanSpeed = packetBuffer[6];
        TiltSpeed = packetBuffer[5];
        NumHex7 = String(packetBuffer[7], HEX);
        NumHex8 = String(packetBuffer[8], HEX);
        NumHex9 = String(packetBuffer[9], HEX);
        NumHex10 = String(packetBuffer[10], HEX);
        NumHex11 = String(packetBuffer[11], HEX);
        NumHex12 = String(packetBuffer[12], HEX);
        NumHex13 = String(packetBuffer[13], HEX);
        NumHex14  = String(packetBuffer[14], HEX);
      }

    }


    if (ViscaPacketSize == 16) {
      ViscaComand = (packetBuffer[10] * 10 + packetBuffer[12]);
    }
    if (ViscaPacketSize == 17) {
      ViscaComand = (packetBuffer[14] * 10) + packetBuffer[15];
      PanSpeed = ((packetBuffer[12]));
      TiltSpeed = ((packetBuffer[13]));
    }


    for ( int i = 0; i < ViscaPacketSize;  ++i ) {               //Clear the buffer
      packetBuffer[i] = (char)0;


    }

 

    //*******************Visca comands*******************

    //Udp.beginPacket(myGW, 1259);
    //Visca Returns requested at setup
    //                        if (ViscaComand == 78) {                 //Pant Tilt position request
    //                          Udp.write(PTposition, 11);
    //                          Serial.println ("PT position requested");
    //                          delay(200);
    //                        }
    if (ViscaComand == 111) {                                         //Zoom Position request
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(ZoomPosition, 7);
      Udp.endPacket();
      Serial.println ("Zoom position requested");

      //delay(200);
    }
    if (ViscaComand == 112) {                                         //Focus position request
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(FocusPosition, 7);
      Udp.endPacket();
      Serial.println ("Focus position requested");
      //delay(200);
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(Confirm, 3);
      Udp.endPacket();
      Serial.println ("Confirm sent");
    }





  }
  char buffer[50];
  switch (ViscaComand)  {

    case 3802: Serial.println ("AKA");
      Udp.beginPacket(myGW, UDP);
      Udp.write(Confirm, 3);
      Udp.endPacket();
      Serial.println ("Confirm sent");
      break;

    case 14: Serial.println ("Tally light command:");
      sprintf(buffer, "%d,%d\n", ViscaComand, Tally);
      UARTport.print(buffer);
      Serial.print (Tally); break;
    
    case 76: Serial.println ("Zoom/Focus Absolute position move request");
      ReadInZoomFocus();
      Serial.print("PanSpeed: ");
      Serial.println(PanSpeed);
      Serial.print("TiltSpeed: ");
      Serial.println(TiltSpeed);
      Serial.print("PAngleIn: ");
      Serial.println(PAngleIn);
      Serial.print("TAngleIn: ");
      Serial.println(TAngleIn);
      Serial.print("FAngleIn: ");
      Serial.println(FAngleIn);
      Serial.print("ZAngleIn: ");
      Serial.println(ZAngleIn);
      Serial.println("Sending vMix Absolute position request to PTZ head");
      ViscaComand = 9;
      sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d\n", ViscaComand, PanSpeed, TiltSpeed, PAngleIn, TAngleIn, FAngleIn, ZAngleIn );
      UARTport.print(buffer);
      break;

    case 77: Serial.println ("Focus Absolute position move request");
      //Read in the Focus request here
      Udp.beginPacket(myGW, 1259);
      Udp.write(Confirm, 3);
      Udp.endPacket();
      Serial.println ("Confirm sent");
      break;

    case 9: Serial.println ("PT Absolute position move request from vMix");
      ReadInPTpos();
            sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d\n", ViscaComand, PanSpeed, TiltSpeed, PAngleIn, TAngleIn, FAngleIn, ZAngleIn );
            UARTport.print(buffer); break;
      break;

    case 5603: Serial.println("Manual focus");
      Udp.beginPacket(myGW, 1259);
      Udp.write(Confirm, 3);
      Udp.endPacket();
      Serial.println ("Confirm sent");
      break;

    case 71: ("Send focus home");
      Udp.beginPacket(myGW, 1259);
      Udp.write(Confirm, 3);
      Udp.endPacket(); break;

    case 78:  Serial.println ("PT position requested1");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer);                //Send request to PTZ head
      int Request;
      if (Request = 0) {
        Serial.println ("CHECKING FOR UART reply");
        listenForUART();
                Serial.print("PAngle: ");
                Serial.println(PAngle);
                Serial.print("TAngle: ");
                Serial.println(TAngle);
                Serial.print("ZAngle: ");
                Serial.println(ZAngle);
                Serial.print("FAngle: ");
                Serial.println(FAngle);
                  Serial.println(PTposition[0]);
                  Serial.println(PTposition[1]);
                  Serial.println(PTposition[10]);

        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(PTposition, 11);
        Udp.endPacket();
        //delay(200);
        Request = 2;
      } else {
        //PTposition[2] = 0x00; PTposition[3] = 0x05; PTposition[4] = 0x02; PTposition[5] = 0x0E; PTposition[6] = 0x0F; break;
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(PTposition, 11);
        Udp.endPacket();
        //delay(200);
        Request = 0;
        Serial.println(PTposition[0] );
        Serial.println(PTposition[1]);
        Serial.println(PTposition[2]);
        Serial.println(PTposition[3]);
        Serial.println(PTposition[4]);
        Serial.println(PTposition[5]);
        Serial.println(PTposition[6]);
        Serial.println(PTposition[7]);
        Serial.println(PTposition[8]);
        Serial.println(PTposition[9]);
        Serial.println(PTposition[10]);
        Serial.println(PTposition[11]);
      }
      break;




    case 64: Serial.print(", PT Home ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;




    case 33: Serial.print(", PT Stop ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer);
      delay(20);
      listenForUART();
      Serial.print("IP4 ");
      Serial.print(IP4);
      break;

    case 700: Serial.print(", Zoom Stop ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer);
      delay(20);
      listenForUART();
      break;

    case 800: Serial.print(", Focus Stop 1 ");
      Serial.print(ViscaComand);
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer);
      delay(20);
      listenForUART();
      break;
    case 13: Serial.print(", Pan Left ");
      sprintf(buffer, "%d,%d,%d\n", ViscaComand, PanSpeed, TiltSpeed );
      UARTport.print(buffer); break;


    case 23: Serial.print(", Pan Right ");
      sprintf(buffer, "%d,%d,%d\n", ViscaComand, PanSpeed, TiltSpeed );
      UARTport.print(buffer); break;

    case 31: Serial.print(", Tilt Up ");
      sprintf(buffer, "%d,%d,%d\n", ViscaComand, PanSpeed, TiltSpeed );
      UARTport.print(buffer); break;

    case 32: Serial.print(", Tilt Down ");
      sprintf(buffer, "%d,%d,%d\n", ViscaComand, PanSpeed, TiltSpeed );
      UARTport.print(buffer); break;

    case 11: Serial.print(", Up Left ");
      sprintf(buffer, "%d,%d,%d\n", ViscaComand, PanSpeed, TiltSpeed );
      UARTport.print(buffer); break;

    case 21: Serial.print(", Up Right ");
      sprintf(buffer, "%d,%d,%d\n", ViscaComand, PanSpeed, TiltSpeed );
      UARTport.print(buffer); break;

    case 12: Serial.print(", Down Left ");
      sprintf(buffer, "%d,%d,%d\n", ViscaComand, PanSpeed, TiltSpeed );
      UARTport.print(buffer); break;

    case 22: Serial.print(", Down Right ");
      sprintf(buffer, "%d,%d,%d\n", ViscaComand, PanSpeed, TiltSpeed );
      UARTport.print(buffer); break;

    case 748: Serial.print(", Zoom out p1  ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 749: Serial.print(", Zoom out p2  ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 750: Serial.print(", Zoom out p3  ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 751: Serial.print(", Zoom out p4  ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 752: Serial.print(", Zoom out p5  ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 753: Serial.print(", Zoom out p6  ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 754: Serial.print(", Zoom out p7  ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 755: Serial.print(", Zoom out p8  ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 756: Serial.print(", Zoom out p9  ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 732: Serial.print(", Zoom in p1 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 733: Serial.print(", Zoom in p2 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 734: Serial.print(", Zoom in p3 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 735: Serial.print(", Zoom in p4 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 736: Serial.print(", Zoom in p5 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 737: Serial.print(", Zoom in p6 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 738: Serial.print(", Zoom in p7 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 739: Serial.print(", Zoom in p8 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 2402: Serial.print(", Focus Stop ");
      Serial.print(ViscaComand);
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer);
      delay(20);
      listenForUART();
      break;
    case 832: Serial.print(", Focus Near p1 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 833: Serial.print(", Focus Near p2 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 834: Serial.print(", Focus Near p3 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 835: Serial.print(", Focus Near p4 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 836: Serial.print(", Focus Near p5 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 837: Serial.print(", Focus Near p6 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 838: Serial.print(", Focus Near p7 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 839: Serial.print(", Focus Near p8 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 848: Serial.print(", Focus Far p1 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 849: Serial.print(", Focus Far p2 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 850: Serial.print(", Focus Far p3 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 851: Serial.print(", Focus Far p4 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 852: Serial.print(", Focus Far p5 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 853: Serial.print(", Focus Far p6 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 854: Serial.print(", Focus Far p7 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 855: Serial.print(", Focus Far p8 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 4901: Serial.print(", Ramp/EaseValue=1 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 4902: Serial.print(", Ramp/EaseValue=2 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 4903: Serial.print(", Ramp/EaseValue=3 ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 2: Serial.print(", Start Recore ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 3: Serial.print(", Stop Record ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 41: Serial.print(", Save Pose ");
      Serial.print(Pose);
      sprintf(buffer, "%d,%d\n", ViscaComand, Pose );
      UARTport.print(buffer); break;

    case 42: Serial.print(", Move to pose ");
      Serial.print(Pose);
      sprintf(buffer, "%d,%d\n", ViscaComand, Pose );
      UARTport.print(buffer); break;

    case 40: Serial.print(", Reset Pose do Defaults ");
      Serial.print(Pose);
      sprintf(buffer, "%d,%d\n", ViscaComand, Pose );
      UARTport.print(buffer); break;


    case 126: Serial.print(", Record Mode setting ");
      sprintf(buffer, "%d\n", ViscaComand );
      UARTport.print(buffer); break;

    case 137: Serial.print(", Pose speed ");
    sprintf(buffer, "%d,%d\n", ViscaComand,PoseSpeed );
      UARTport.print(buffer); break;

 
  
  
  }
  ViscaComand = 0;                                          //Clear the Comand ready to start fresh
}
void  listenForUART() {
  //Serial.println ("Listening");
  char buffer[30];
  int PP = 78;
  int TT = 79;
  int ZZ = 80;
  int FF = 81;
  int SYS1 = 665;
  int SYS2 = 666;
  int SYS3 = 667;
  int SYS4 = 668;
  int SYS5 = 669;

  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  sprintf(buffer, "%d\n", PP );//First Pass get PP
  UARTport.print(buffer);
  UARTport.flush();
  newData = false;
  delay(50);
  GetUARTdata();
  if (newData == true) {
    PAngle = atoi(receivedChars);
    PAngle = PAngle * 2;
    //PAngle = 10 * ((PAngle + 5) / 10);  //Round to nearest 10

    //    if (PAngle < 0) {
    //      PAngle = abs(PAngle) + 1000;
    //    }
    Serial.print("\n");
    Serial.print("PAngle: ");
    Serial.println(PAngle);
    if (PAngle < 0) {
      Serial.print("PAngle is negative ");
    } else {
      Serial.print("PAngle is Posative ");
    }
  }

  sprintf(buffer, "%d\n", TT );   //Second pass get TT
  delay(100);
  UARTport.print(buffer);
  UARTport.flush();
  newData = false;
  delay(50);
  GetUARTdata();
  if (newData == true) {
    TAngle = atoi(receivedChars) * 2;
    //TAngle = 10 * ((TAngle + 5) / 10);  //Round to nearest 10
    //    if (TAngle < 0) {
    //      TAngle = abs(TAngle) + 1000;
    //    }
    Serial.print("TAngle: ");
    Serial.println(TAngle);
  }

  sprintf(buffer, "%d\n", FF );     //Third pass get FF
  delay(100);
  UARTport.print(buffer);
  UARTport.flush();
  newData = false;
  delay(50);
  GetUARTdata();
  if (newData == true) {
    FAngle = atoi(receivedChars);
    FAngle = abs(FAngle);
    Serial.print("FAngle=: ");  //This gets past back to VISCA position save
    Serial.println(FAngle);
  }
 
 sprintf(buffer, "%d\n", ZZ );     //Forth pass get ZZ
  delay(100);
  UARTport.print(buffer);
  UARTport.flush();
  newData = false;
  delay(50);
  GetUARTdata();
  if (newData == true) {
    ZAngle = atoi(receivedChars);
     ZAngle = abs(ZAngle);
    Serial.print("ZAngle: ");
    Serial.println(ZAngle);
  }
  //Convert to uint8 for output


  if (PAngle > 0) {
    PAngle = PAngle * 117.95;     //divisions reduced from norm to make establisheing sign of value easier
  } else {
    PAngle = PAngle * 235.9;
  }

  TAngle = TAngle * 235.9;

  Serial.print("PAngle: ");
  Serial.println(PAngle);
  if (PAngle < 0) {
    Serial.print("PAngle is negative ");
  } else {
    Serial.print("PAngle is Posative ");
  }
  if (PAngle < 0) {           //If Pangle negative
    PTposition[2] = {0x0F};
    Serial.print("PAngle negative");
  } else {
    PTposition[2] = {0x00};
  }


  // Solve for Pan

  for (int8_t i = 8; i > 0; i--) {
    if (i - 2 > 2) {
      PTposition[i - 2] = PAngle % 16;          //Add it to the Array
    }
    PAngle = (PAngle - PTposition[i - 2]) / 16; //Process the next part

    if ( PTposition[2] == 0x0F) {                   //If We are processing a negative angle striff off the F for VISCA
      switch (PTposition[i - 2]) {
        case 0x00: PTposition[i - 2] = 0x0F; break;
        case 0xF1: PTposition[i - 2] = 0x01; break;
        case 0xF2: PTposition[i - 2] = 0x02; break;
        case 0xF3: PTposition[i - 2] = 0x03; break;
        case 0xF4: PTposition[i - 2] = 0x04; break;
        case 0xF5: PTposition[i - 2] = 0x05; break;
        case 0xF6: PTposition[i - 2] = 0x06; break;
        case 0xF7: PTposition[i - 2] = 0x07; break;
        case 0xF8: PTposition[i - 2] = 0x08; break;
        case 0xF9: PTposition[i - 2] = 0x09; break;
        case 0xFA: PTposition[i - 2] = 0x0A; break;
        case 0xFB: PTposition[i - 2] = 0x0B; break;
        case 0xFC: PTposition[i - 2] = 0x0C; break;
        case 0xFD: PTposition[i - 2] = 0x0D; break;
        case 0xFE: PTposition[i - 2] = 0x0E; break;
        case 0xFF: PTposition[i - 2] = 0x0F; break;
      }
    }
  }


  // Solve for Tilt   ****Tilt isstamping F on position2 Fix
  for (int8_t i = 12; i > 0; i--) {
    if (i - 2 > 6) {
      PTposition[i - 2] = TAngle % 16;
    }
    TAngle = (TAngle - PTposition[i - 2]) / 16;
    if ( TAngle < 0) {                  //If We are processing a negative angle striff off the F for VISCA
      switch (PTposition[i - 2]) {
        case 0xF1: PTposition[i - 2] = 0x01; break;
        case 0xF2: PTposition[i - 2] = 0x02; break;
        case 0xF3: PTposition[i - 2] = 0x03; break;
        case 0xF4: PTposition[i - 2] = 0x04; break;
        case 0xF5: PTposition[i - 2] = 0x05; break;
        case 0xF6: PTposition[i - 2] = 0x06; break;
        case 0xF7: PTposition[i - 2] = 0x07; break;
        case 0xF8: PTposition[i - 2] = 0x08; break;
        case 0xF9: PTposition[i - 2] = 0x09; break;
        case 0xFA: PTposition[i - 2] = 0x0A; break;
        case 0xFB: PTposition[i - 2] = 0x0B; break;
        case 0xFC: PTposition[i - 2] = 0x0C; break;
        case 0xFD: PTposition[i - 2] = 0x0D; break;
        case 0xFE: PTposition[i - 2] = 0x0E; break;
        case 0xFF: PTposition[i - 2] = 0x0F; break;
      }
    }

  }

  Serial.print("PTpos0: ");
  Serial.println(PTposition[0]);
  Serial.print("PTpos1: ");
  Serial.println(PTposition[1]);
  Serial.print("PTpos2: ");
  Serial.println(PTposition[2]);
  Serial.print("PTpos3: ");
  Serial.println(PTposition[3]);
  Serial.print("PTpos4: ");
  Serial.println(PTposition[4]);
  Serial.print("PTpos5: ");
  Serial.println(PTposition[5]);
  Serial.print("PTpos6: ");
  Serial.println(PTposition[6]);
  Serial.print("PTpos7: ");
  Serial.println(PTposition[7]);
  Serial.print("PTpos8: ");
  Serial.println(PTposition[8]);
  Serial.print("PTpos9: ");
  Serial.println(PTposition[9]);
  Serial.print("PTpos10: ");
  Serial.println(PTposition[10]);
  Serial.print("PTpos11: ");
  Serial.println(PTposition[11]); //Closing termonator

  // Solve for Focus

  FAngle = ((FAngle * 604.5) + 1000);            //Because the Focus range is 1000-61440

  for (int8_t i = 7; i > 0; i--) {
    if (i - 2 > 1) {
      FocusPosition[i - 2] = FAngle % 16;           //Add it to the Array
    }
    FAngle = (FAngle - FocusPosition[i - 2]) / 16;   //Process the next part
  }
  Serial.print("Fpos0: ");
  Serial.println(FocusPosition[0]);
  Serial.print("Fpos1: ");
  Serial.println(FocusPosition[1]);
  Serial.print("Fpos2: ");
  Serial.println(FocusPosition[2]);
  Serial.print("Fpos3: ");
  Serial.println(FocusPosition[3]);
  Serial.print("Fpos4: ");
  Serial.println(FocusPosition[4]);
  Serial.print("Fpos5: ");
  Serial.println(FocusPosition[5]);
  Serial.print("Fpos6 -Should be 255: ");
  Serial.println(FocusPosition[6]);




  // Solve for Zoom

  ZAngle = (ZAngle * 60);                       //Because the zoom range is 0-6000

  for (int8_t i = 7; i > 0; i--) {
    if (i - 2 > 1) {
      ZoomPosition[i - 2] = ZAngle % 16;           //Add it to the Array
    }
    ZAngle = (ZAngle - ZoomPosition[i - 2]) / 16;   //Process the next part
  }
  ZoomPosition[6] = {0xFF};
  Serial.print("ZPos0: ");
  Serial.println(ZoomPosition[0]);
  Serial.print("ZPos1: ");
  Serial.println(ZoomPosition[1]);
  Serial.print("ZPos2: ");
  Serial.println(ZoomPosition[2]);
  Serial.print("ZPos3: ");
  Serial.println(ZoomPosition[3]);
  Serial.print("ZPos4: ");
  Serial.println(ZoomPosition[4]);
  Serial.print("ZPos5: ");
  Serial.println(ZoomPosition[5]);
  Serial.print("ZPos6 should be 255: ");
  Serial.println(ZoomPosition[6]);




  //  switch (PAngle) {
  //    //Left
  //    case 0: PTposition[2] = 0x00; PTposition[3] = 0x00; PTposition[4] = 0x00; PTposition[5] = 0x00; PTposition[6] = 0x00; break;
  //    case 10: PTposition[2] = 0x00; PTposition[3] = 0x00; PTposition[4] = 0x09; PTposition[5] = 0x03; PTposition[6] = 0x07; break;
  //    case 20: PTposition[2] = 0x00; PTposition[3] = 0x01; PTposition[4] = 0x02; PTposition[5] = 0x06; PTposition[6] = 0x0E; break;
  //    case 30: PTposition[2] = 0x00; PTposition[3] = 0x01; PTposition[4] = 0x0B; PTposition[5] = 0x0A; PTposition[6] = 0x05; break;
  //    case 40: PTposition[2] = 0x00; PTposition[3] = 0x02; PTposition[4] = 0x04; PTposition[5] = 0x0D; PTposition[6] = 0x0C; break;
  //    case 50: PTposition[2] = 0x00; PTposition[3] = 0x02; PTposition[4] = 0x0E; PTposition[5] = 0x01; PTposition[6] = 0x03; break;
  //    case 60: PTposition[2] = 0x00; PTposition[3] = 0x03; PTposition[4] = 0x07; PTposition[5] = 0x04; PTposition[6] = 0x0A; break;
  //    case 70: PTposition[2] = 0x00; PTposition[3] = 0x04; PTposition[4] = 0x00; PTposition[5] = 0x08; PTposition[6] = 0x01; break;
  //    case 80: PTposition[2] = 0x00; PTposition[3] = 0x04; PTposition[4] = 0x09; PTposition[5] = 0x0B; PTposition[6] = 0x08; break;
  //    case 90: PTposition[2] = 0x00; PTposition[3] = 0x05; PTposition[4] = 0x02; PTposition[5] = 0x0E; PTposition[6] = 0x0F; break;
  //    case 100: PTposition[2] = 0x00; PTposition[3] = 0x05; PTposition[4] = 0x0C; PTposition[5] = 0x02; PTposition[6] = 0x06; break;
  //    case 110: PTposition[2] = 0x00; PTposition[3] = 0x00; PTposition[4] = 0x09; PTposition[5] = 0x03; PTposition[6] = 0x07; break;
  //    case 120: PTposition[2] = 0x00; PTposition[3] = 0x06; PTposition[4] = 0x0E; PTposition[5] = 0x09; PTposition[6] = 0x04; break;
  //    case 130: PTposition[2] = 0x00; PTposition[3] = 0x07; PTposition[4] = 0x07; PTposition[5] = 0x0C; PTposition[6] = 0x0B; break;
  //    case 140: PTposition[2] = 0x00; PTposition[3] = 0x08; PTposition[4] = 0x01; PTposition[5] = 0x00; PTposition[6] = 0x02; break;
  //    case 150: PTposition[2] = 0x00; PTposition[3] = 0x08; PTposition[4] = 0x0A; PTposition[5] = 0x03; PTposition[6] = 0x09; break;
  //    case 160: PTposition[2] = 0x00; PTposition[3] = 0x09; PTposition[4] = 0x03; PTposition[5] = 0x07; PTposition[6] = 0x00; break;
  //    case 169: PTposition[2] = 0x00; PTposition[3] = 0x09; PTposition[4] = 0x0B; PTposition[5] = 0x0B; PTposition[6] = 0x0B; break;
  //    case 170: PTposition[2] = 0x00; PTposition[3] = 0x09; PTposition[4] = 0x0C; PTposition[5] = 0x0A; PTposition[6] = 0x07; break;
  //    //    //Right
  //    case 1010: PTposition[2] = 0x0F; PTposition[3] = 0x0F; PTposition[4] = 0x06; PTposition[5] = 0x0C; PTposition[6] = 0x09; break;
  //    case 1020: PTposition[2] = 0x0F; PTposition[3] = 0x0E; PTposition[4] = 0x0D; PTposition[5] = 0x09; PTposition[6] = 0x02; break;
  //    case 1030: PTposition[2] = 0x0F; PTposition[3] = 0x0E; PTposition[4] = 0x04; PTposition[5] = 0x05; PTposition[6] = 0x0B; break;
  //    case 1040: PTposition[2] = 0x0F; PTposition[3] = 0x0D; PTposition[4] = 0x0B; PTposition[5] = 0x02; PTposition[6] = 0x04; break;
  //    case 1050: PTposition[2] = 0x0F; PTposition[3] = 0x0D; PTposition[4] = 0x01; PTposition[5] = 0x0E; PTposition[6] = 0x0D; break;
  //    case 1060: PTposition[2] = 0x0F; PTposition[3] = 0x0C; PTposition[4] = 0x08; PTposition[5] = 0x0B; PTposition[6] = 0x06; break;
  //    case 1070: PTposition[2] = 0x0F; PTposition[3] = 0x0B; PTposition[4] = 0x0F; PTposition[5] = 0x07; PTposition[6] = 0x0F; break;
  //    case 1080: PTposition[2] = 0x0F; PTposition[3] = 0x0B; PTposition[4] = 0x06; PTposition[5] = 0x04; PTposition[6] = 0x08; break;
  //    case 1090: PTposition[2] = 0x0F; PTposition[3] = 0x0A; PTposition[4] = 0x0D; PTposition[5] = 0x01; PTposition[6] = 0x01; break;
  //    case 1100: PTposition[2] = 0x0F; PTposition[3] = 0x0A; PTposition[4] = 0x03; PTposition[5] = 0x0D; PTposition[6] = 0x0A; break;
  //    case 1110: PTposition[2] = 0x0F; PTposition[3] = 0x09; PTposition[4] = 0x0A; PTposition[5] = 0x0A; PTposition[6] = 0x03; break;
  //    case 1120: PTposition[2] = 0x0F; PTposition[3] = 0x09; PTposition[4] = 0x01; PTposition[5] = 0x06; PTposition[6] = 0x0C; break;
  //    case 1130: PTposition[2] = 0x0F; PTposition[3] = 0x08; PTposition[4] = 0x08; PTposition[5] = 0x03; PTposition[6] = 0x05; break;
  //    case 1140: PTposition[2] = 0x0F; PTposition[3] = 0x07; PTposition[4] = 0x0E; PTposition[5] = 0x0F; PTposition[6] = 0x0E; break;
  //    case 1150: PTposition[2] = 0x0F; PTposition[3] = 0x07; PTposition[4] = 0x05; PTposition[5] = 0x0C; PTposition[6] = 0x07; break;
  //    case 1160: PTposition[2] = 0x0F; PTposition[3] = 0x06; PTposition[4] = 0x0C; PTposition[5] = 0x09; PTposition[6] = 0x00; break;
  //    case 1169: PTposition[2] = 0x0F; PTposition[3] = 0x06; PTposition[4] = 0x04; PTposition[5] = 0x04; PTposition[6] = 0x05; break;
  //    case 1170: PTposition[2] = 0x0F; PTposition[3] = 0x06; PTposition[4] = 0x03; PTposition[5] = 0x05; PTposition[6] = 0x09; break;
  //  }
  //
  //  switch (TAngle) {
  //    //    //UP
  //    case 0: PTposition[7] = 0x00; PTposition[8] = 0x00; PTposition[9] = 0x00; PTposition[10] = 0x00;  break;
  //    case 10: PTposition[7] = 0x00; PTposition[8] = 0x09; PTposition[9] = 0x03; PTposition[10] = 0x07; break;
  //    case 20: PTposition[7] = 0x01; PTposition[8] = 0x02; PTposition[9] = 0x06; PTposition[10] = 0x0E; break;
  //    case 30: PTposition[7] = 0x01; PTposition[8] = 0x0B; PTposition[9] = 0x0A; PTposition[10] = 0x05; break;
  //    case 40: PTposition[7] = 0x02; PTposition[8] = 0x04; PTposition[9] = 0x0D; PTposition[10] = 0x0C; break;
  //    case 50: PTposition[7] = 0x02; PTposition[8] = 0x0E; PTposition[9] = 0x01; PTposition[10] = 0x03; break;
  //    case 60: PTposition[7] = 0x03; PTposition[8] = 0x07; PTposition[9] = 0x04; PTposition[10] = 0x0A; break;
  //    case 70: PTposition[7] = 0x04; PTposition[8] = 0x00; PTposition[9] = 0x08; PTposition[10] = 0x01; break;
  //    case 80: PTposition[7] = 0x04; PTposition[8] = 0x09; PTposition[9] = 0x0B; PTposition[10] = 0x08; break;
  //    case 90: PTposition[7] = 0x05; PTposition[8] = 0x02; PTposition[9] = 0x0E; PTposition[10] = 0x0F; break;
  //    //    //Down
  //    case 1010: PTposition[7] = 0x0F; PTposition[8] = 0x06; PTposition[9] = 0x0C; PTposition[10] = 0x09; break;
  //    case 1020: PTposition[7] = 0x0E; PTposition[8] = 0x0D; PTposition[9] = 0x09; PTposition[10] = 0x02; break;
  //    case 1030: PTposition[7] = 0x0E; PTposition[8] = 0x04; PTposition[9] = 0x05; PTposition[10] = 0x0B; break;
  //
  //  }
  //
  //   switch (FAngle) {
  //      //    //UP
  //      case 0: PTposition[7] = 0x00; PTposition[8] = 0x00; PTposition[9] = 0x00; PTposition[10] = 0x00;  break;
  //      case 10: PTposition[7] = 0x00; PTposition[8] = 0x09; PTposition[9] = 0x03; PTposition[10] = 0x07; break;
  //      case 20: PTposition[7] = 0x01; PTposition[8] = 0x02; PTposition[9] = 0x06; PTposition[10] = 0x0E; break;
  //      case 30: PTposition[7] = 0x01; PTposition[8] = 0x0B; PTposition[9] = 0x0A; PTposition[10] = 0x05; break;
  //      case 40: PTposition[7] = 0x02; PTposition[8] = 0x04; PTposition[9] = 0x0D; PTposition[10] = 0x0C; break;
  //      case 50: PTposition[7] = 0x02; PTposition[8] = 0x0E; PTposition[9] = 0x01; PTposition[10] = 0x03; break;
  //      case 60: PTposition[7] = 0x03; PTposition[8] = 0x07; PTposition[9] = 0x04; PTposition[10] = 0x0A; break;
  //      case 70: PTposition[7] = 0x04; PTposition[8] = 0x00; PTposition[9] = 0x08; PTposition[10] = 0x01; break;
  //      case 80: PTposition[7] = 0x04; PTposition[8] = 0x09; PTposition[9] = 0x0B; PTposition[10] = 0x08; break;
  //      case 90: PTposition[7] = 0x05; PTposition[8] = 0x02; PTposition[9] = 0x0E; PTposition[10] = 0x0F; break;
  //      //    //Down
  //      case 1010: PTposition[7] = 0x0F; PTposition[8] = 0x06; PTposition[9] = 0x0C; PTposition[10] = 0x09; break;
  //      case 1020: PTposition[7] = 0x0E; PTposition[8] = 0x0D; PTposition[9] = 0x09; PTposition[10] = 0x02; break;
  //      case 1030: PTposition[7] = 0x0E; PTposition[8] = 0x04; PTposition[9] = 0x05; PTposition[10] = 0x0B; break;
  //
  // }





}


void GetUARTdata() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (UARTport.available() ) {
    rc = UARTport.read();
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx > numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}


//At startup get the current IP adress from rthe main processor
void  GetIP() {
  Serial.println ("Listening");
  char buffer[30];
  delay(500);
  int SYS1 = 665;
  int SYS2 = 666;
  int SYS3 = 667;
  int SYS4 = 668;
  int SYS5 = 669;
  int SYS6 = 670;
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  sprintf(buffer, "%d\n", SYS1 );     //Gget IP1
  UARTport.print(buffer);
  UARTport.flush();
  newData = false;
  delay(50);
  GetUARTdata();
  if (newData == true) {
    IP1 = atoi(receivedChars);

    Serial.print("IP1: ");
    Serial.println(IP1);
  }
  sprintf(buffer, "%d\n", SYS2 );     //Gget IP2
  UARTport.print(buffer);
  UARTport.flush();
  newData = false;
  delay(50);
  GetUARTdata();
  if (newData == true) {
    IP2 = atoi(receivedChars);

    Serial.print("IP2: ");
    Serial.println(IP2);
  }
  sprintf(buffer, "%d\n", SYS3 );     //Gget IP3
  UARTport.print(buffer);
  UARTport.flush();
  newData = false;
  delay(50);
  GetUARTdata();
  if (newData == true) {
    IP3 = atoi(receivedChars);

    Serial.print("IP3: ");
    Serial.println(IP3);
  }
  sprintf(buffer, "%d\n", SYS4 );     //Gget IP4
  UARTport.print(buffer);
  UARTport.flush();
  newData = false;
  delay(50);
  GetUARTdata();
  if (newData == true) {
    IP4 = atoi(receivedChars);

    Serial.print("IP4: ");
    Serial.println(IP4);
  }

  sprintf(buffer, "%d\n", SYS6 );     //Gget IPGW      the server gatway
  UARTport.print(buffer);
  UARTport.flush();
  newData = false;
  delay(50);
  GetUARTdata();
  if (newData == true) {
    IPGW = atoi(receivedChars);

    Serial.print("IPGW: ");
    Serial.println(IPGW);
  }

  sprintf(buffer, "%d\n", SYS5 );     //Gget UDPport
  UARTport.print(buffer);
  UARTport.flush();
  newData = false;
  delay(50);
  GetUARTdata();
  if (newData == true) {
    UDP = atoi(receivedChars);

    Serial.print("UDPport: ");
    Serial.println(UDP);
  }
}

void ReadInPTpos() {                             //Read in the Absolute position request PT only
  //Convert Pan back to Dec Degrees for input

  Serial.print("NumHex7: ");
  Serial.println(NumHex7);
  Serial.print("NumHex8: ");
  Serial.println(NumHex8);
  Serial.print("NumHex9: ");
  Serial.println(NumHex9);
  Serial.print("NumHex10: ");
  Serial.println(NumHex10);
  char buffer[30];
  sprintf(buffer, "0x%s%s%s%s", NumHex7, NumHex8, NumHex9, NumHex10);
  String Hstring = (buffer);
  Serial.println(Hstring);

  char H_array[7];
  Hstring.toCharArray(H_array, 7);
  int x;
  char *endptr;
  x = strtol(H_array, &endptr, 16);
  Serial.println(x, HEX);
  Serial.println(x, DEC);

  if ( x >= 23090) {                          //If the move is negatvie
    PAngleIn = (x - 23090);                   //Stored int16_t to provide a signed 2's value NOT REQUIRED
    PAngleIn = 180 - ((PAngleIn / 235.9) );
    PAngleIn = (PAngleIn - (PAngleIn * 2));   //Sign it as negative
    Serial.print("PAngleIn: ");
    Serial.println(PAngleIn);
  } else {
    PAngleIn = x;
    PAngleIn = ((PAngleIn / 117.95));

    Serial.print("PAngleIn: ");
    Serial.println(PAngleIn);
  }


  //Convert Tilt back to Dec Degrees for input

  Serial.print("NumHex11: ");
  Serial.println(NumHex11);
  Serial.print("NumHex12: ");
  Serial.println(NumHex12);
  Serial.print("NumHex13: ");
  Serial.println(NumHex13);
  Serial.print("NumHex14: ");
  Serial.println(NumHex14);
  //char buffer[30];
  sprintf(buffer, "0x%s%s%s%s", NumHex11, NumHex12, NumHex13, NumHex14); //Send the requested PT vMix position to the head
  Hstring = (buffer);
  Serial.println(Hstring);

  //char H_array[7];
  Hstring.toCharArray(H_array, 7);
  x;
  //char *endptr;
  x = strtol(H_array, &endptr, 16);
  Serial.println(x, HEX);
  Serial.println(x, DEC);
  TAngleIn = x;                                //Stored int16_t to provide a signed 2's value not required
  if (TAngleIn > 43000) {                      //Then the move is negative down
    TAngleIn = (x - 23090);                   //Stored int16_t to provide a signed 2's value
    TAngleIn = 180 - ((TAngleIn / 235.9) );
    TAngleIn = (TAngleIn - (TAngleIn * 2));     //Sign it as negative
    Serial.print("TAngleIn: ");
    Serial.println(TAngleIn);
  } else {
    TAngleIn = ((TAngleIn / 235.9));
    Serial.print("TAngleIn: ");
    Serial.println(TAngleIn);
  }
}
void ReadInZoomFocus() {                             //Read in the Absolute position request PT only
  //Convert Zoom back to % of Zoom

  Serial.print("NumHex4: ");
  Serial.println(NumHex4);
  Serial.print("NumHex5: ");
  Serial.println(NumHex5);
  Serial.print("NumHex6: ");
  Serial.println(NumHex6);
  Serial.print("NumHex7: ");
  Serial.println(NumHex7);
  char buffer[30];
  sprintf(buffer, "0x%s%s%s%s", NumHex4, NumHex5, NumHex6, NumHex7); //Send the vMix zoom request to the head
  String Hstring = (buffer);
  Serial.println(Hstring);

  char H_array[7];
  Hstring.toCharArray(H_array, 7);
  int x;
  char *endptr;
  x = strtol(H_array, &endptr, 16);
  Serial.println(x, HEX);
  Serial.println(x, DEC);

  ZAngleIn = x;
  ZAngleIn = ZAngleIn / 60;

  Serial.print("ZAngleIn: ");
  Serial.println(ZAngleIn);



  //Convert Focus back to % of focus for input

  Serial.print("NumHex8: ");
  Serial.println(NumHex8);
  Serial.print("NumHex9: ");
  Serial.println(NumHex9);
  Serial.print("NumHex10: ");
  Serial.println(NumHex10);
  Serial.print("NumHex11: ");
  Serial.println(NumHex11);
  //char buffer[30];
  sprintf(buffer, "0x%s%s%s%s", NumHex8, NumHex9, NumHex10, NumHex11);    //Send the focus position to the head
  Hstring = (buffer);
  Serial.println(Hstring);

  //char H_array[7];
  Hstring.toCharArray(H_array, 7);
  x;

  x = strtol(H_array, &endptr, 16);
  Serial.println(x, HEX);
  Serial.println(x, DEC);
  FAngleIn = x;                                //Stored int16_t to provide a signed 2's value not required
  FAngleIn = (FAngleIn - 1000) / 604.5;        //Convert bake to % of posible move


  Serial.print("FAngleIn: ");
  Serial.println(FAngleIn);
}
