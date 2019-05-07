/*                       CONTIMOBIL 
 
*/ 


//DEBUG 

#define DEBUG_RPMCADENCE 
//#define DEBUG_TBUF 
//#define DEBUG_RBUF 
//#define DEBUG_GEAR 
//#define DEBUG_BUTTON 

//DEBUG 

#include <mcp_can.h> 
#include <mcp_can_dfs.h> 
#include <SPI.h> 
#include <stdio.h> 
#include <math.h> 

// Demo: Dual CAN-BUS Drive-Unit Control Through eRocker 
// Written by: Crehul Vlad & Maior Teodor 
// Last Updated on: 14.09.2018 10:57 AM by Crehul Vlad 

#define btn_min 7 //eRocker Minus on Digital Pin 7 
#define btn_plus 6 //eRocker Plus on Digital Pin 6 
#define btn_M 5 //eRocker Menu on Digital Pin 5 
#define btn_pwr 4 //eRocker Power on Digital Pin 4 
#define btn_bec 3 //eRocker Lights on Digital Pin 3 

unsigned long rxID0, rxID1 ; //Recieve IDs 
unsigned long txID0 = 0x153; //Tr IDs 
byte len; //length 
byte rxBuf0[8], rxBuf1[8]; //Recieve buffer 0 & 1 
int gearchn1 = 1, gearchn2 = 5, gearchn3 = 15, spdchn = gearchn1; 
int gearstate = 0; 

byte txBuf0[] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //Tr buffer 
/* 0 0 0 0 0 0 1 1 --> DriveUnit On and L2?? Safety Off 
   # # # # # # # # --> 0 to 255 or 0x00 to 0xFF for second half of motor speed digits 
   # # # # # # # # --> 0 to 255 or 0x00 to 0xFF for first half of motor speed digits 
*/ 

char i, j; 

MCP_CAN CAN0(9);   // CAN0 interface usins CS on digital pin 9 
MCP_CAN CAN1(10);    // CAN1 interface using CS on digital pin 10 

bool PW = 0, update = 0, gear = 0; //Button state 
byte Speed1 = 0, Speed2 = 0; //Second half of digits & First half of digits 

int hextdec(int hexadecimal_number); 

void setup() 
{ 


  pinMode(btn_plus, INPUT_PULLUP); 
  pinMode(btn_min, INPUT_PULLUP); 
  pinMode(btn_M, INPUT_PULLUP); 
  pinMode(btn_bec, INPUT_PULLUP); 
  pinMode(btn_pwr, INPUT_PULLUP); 

  Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\nNEW\n\n\n\n\n\n\n\n\n\n\n\n\n"); 
  Serial.begin(74880); //Serial Baudrate 

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) { //CAN0 Initalization 
    Serial.print("CAN0: Init OK!\r\n\n\n"); 
    CAN0.setMode(MCP_NORMAL); 
  } else { 
    Serial.print("CAN0: Init Fail!!!\r\n"); 
    while (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK) { //If it fails keep trying until it works 
      Serial.print("Retrying...\n"); delay(300); 
    } Serial.print("CAN0: Init OK!\r\n\n\n"); 
  } 

  if (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) { //CAN1 Initalization 
    Serial.print("CAN1: Init OK!\r\n\n\n"); 
    CAN1.setMode(MCP_NORMAL); 
  } else  { 
    Serial.print("CAN1: Init Fail!!!\r\n"); 
    while (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK) { //If it fails keep trying until it works 
      Serial.print("Retrying...\n"); delay(300); 
    } Serial.print("CAN1: Init OK!\r\n\n\n"); 

  } 

  SPI.setClockDivider(SPI_CLOCK_DIV2); 



  txBuf0[1] = 0x00; txBuf0[2] = 0x00; 
  CAN0.sendMsgBuf(0x153, 0, 8, txBuf0); 
  CAN1.sendMsgBuf(0x153, 0, 8, txBuf0); 
} 

void loop() { 

  if (digitalRead(btn_pwr) == 0 && PW == 0) { //Power Button 
    txBuf0[1] = 0x00; txBuf0[2] = 0x00; //If turned OFF reset speed 
    Speed1 = 0; Speed2 = 0; 
    PW = 1; 
    Serial.println("ON"); 
    delay(1000); 
  } 
  if (digitalRead(btn_pwr) == 0 && PW == 1) { 
    txBuf0[1] = 0x00; txBuf0[2] = 0x00; //If turned OFF reset speed 
    Speed1 = 0; Speed2 = 0; 
    CAN0.sendMsgBuf(0x153, 0, 8, txBuf0); 
    CAN1.sendMsgBuf(0x153, 0, 8, txBuf0); 
    PW = 0; 
    Serial.println("OFF"); 
    delay(1000); 
  } 

  if (PW == 1) { 

    if (digitalRead(btn_M) == 0 && gear == 0) { 
      #ifdef DEBUG_BUTTON 
      Serial.println("M Button"); 
      #endif 
      gear = 1; 
      if (gearstate == 0 && gear == 1) { 
        gearstate = 1; 
        #ifdef DEBUG_GEAR 
        Serial.print("Gearstate: "); 
        Serial.print(gearstate); 
        Serial.println("     -  - Gear 2 - -      "); 
        #endif 
        spdchn = gearchn2; 
        gear = 0; 
      } else if (gearstate == 1 && gear == 1) { 
        gearstate = 2; 
        #ifdef DEBUG_GEAR 
        Serial.print("Gearstate: "); 
        Serial.print(gearstate); 
        Serial.println("     -  - Gear 3 - -      "); 
        #endif 
        spdchn = gearchn3; 
        gear = 0; 
      } else if (gearstate == 2 && gear == 1) { 
        gearstate = 0; 
        #ifdef DEBUG_GEAR 
        Serial.print("Gearstate: "); 
        Serial.print(gearstate); 
        Serial.println("     -  - Gear 1 - -      "); 
        #endif 
        spdchn = gearchn1; 
        gear = 0; 
      } 
      delay(500); 
    } 

    if (digitalRead(btn_plus) == 0) { //Speed Control for eRocker Alghoritm 
     #ifdef DEBUG_BUTTON 
      Serial.println("+ Button"); 
      #endif 
      if (Speed1 < 255) { 
        Speed1 += spdchn; 
        update = 1; 
      } else { 
        if (Speed1 == 255 && Speed2 < 16) { 
          Speed2 += 1; 
          Speed1 = 0; 
          update = 1; 
        } 
      } 
    } 
    if (digitalRead(btn_min) == 0) { 
      #ifdef DEBUG_BUTTON 
      Serial.println("- Button"); 
      #endif 
      if (Speed1 != 0 && Speed2 >= 0) { 
        Speed1 -= spdchn; 
        update = 1; 
      } else if (Speed1 == 0 && Speed2 > 0) { 
        Speed2 -= 1; 
        Speed1 = 255; 
        update = 0; 
      } 
    }//End of Alghoritm 

    txBuf0[1] = Speed1; 
    txBuf0[2] = Speed2; 

    byte sndStat0 = CAN1.sendMsgBuf(0x153, 0, 8, txBuf0); 
    #ifdef DEBUG_TBUF 
        if(sndStat0 == CAN_OK) 
          Serial.println("Message 0 Sent Successfully!"); 
        else 
          Serial.println("Error Sending Message 0..."); 
    #endif 

    byte sndStat1 = CAN0.sendMsgBuf(0x153, 0, 8, txBuf0); 
    #ifdef DEBUG_TBUF 
        if(sndStat1 == CAN_OK) 
          Serial.println("Message 1 Sent Successfully!"); 
        else 
          Serial.println("Error Sending Message 1..."); 
     #endif 

    if (CAN_MSGAVAIL == CAN0.checkReceive()) { // check if data coming 
      CAN0.readMsgBuf(&rxID0, &len, rxBuf0);/* Show Recieved Buffer BEGIN0 */ 
      if (rxID0 == 0x3B5) { 
        #ifdef DEBUG_RBUF 
        for (j = 0; j <= 7; j++) { 
          Serial.print(rxBuf0[j], HEX); 
          Serial.print(" "); 
        } 
        #endif 
        #ifdef DEBUG_RPMCADENCE 
        Serial.print("   CAN0 ID: "); Serial.print(rxID0, HEX); 
        Serial.print(" "); 
        Serial.print("Cadence: "); 
        Serial.print(hextdec(rxBuf0[3]));Serial.print("(DEC) = "); 
        Serial.print(rxBuf0[3], HEX); Serial.print("(HEX) RPM"); 
        Serial.print("  |  Sent RPM: "); 
 Serial.print(Speed2);Serial.print("  ");Serial.print(Speed1); Serial.print(" = "); Serial.print(Speed2, HEX);Serial.print("  ");Serial.print(Speed1, HEX); Serial.print("\n"); 
        Serial.println(" "); 
        #endif 
      } 
    } 
    /*Show Recieved Buffer END0*/ 

    if (CAN_MSGAVAIL == CAN1.checkReceive()) { // check if data coming 
      CAN1.readMsgBuf(&rxID1, &len, rxBuf1); 
    
    /* Show Recieved Buffer BEGIN1*/ 
    if (rxID1 == 0x3B5) { 
      #ifdef DEBUG_RBUF 
      for (j = 0; j <= 7; j++) { 
        Serial.print(rxBuf1[j], HEX); 
        Serial.print(" "); 
      } 
      #endif 
      #ifdef DEBUG_RPMCADENCE 
      Serial.print("   CAN1 ID: "); Serial.print(rxID1, HEX); 
      Serial.print(" "); 
      Serial.print("Cadence: "); 
     Serial.print(hextdec(rxBuf1[3]));Serial.print("(DEC) = "); 
        Serial.print(rxBuf1[3], HEX); Serial.print("(HEX) RPM"); 
      Serial.print("  |  Sent RPM: "); 
      Serial.print(Speed2);Serial.print("  ");Serial.print(Speed1); Serial.print(" = "); Serial.print(Speed2, HEX);Serial.print("  ");Serial.print(Speed1, HEX); Serial.print("\n"); 
      Serial.println("\n "); 
      #endif 
    } 
    } 
  } 
  /*Show Recieved Buffer END1*/ 
} //END PW Button ON or OFF 
// END Loop 

int hextdec(int hexadecimal_number) { 
  int decimal_number = 0, remainder; 
  int count = 0; 
  while (hexadecimal_number > 0) 
  { 
    remainder = hexadecimal_number % 10; 
    decimal_number = decimal_number + remainder * pow(16, count); 
    hexadecimal_number = hexadecimal_number / 10; 
    count++; 
  } 

  return decimal_number; 
}