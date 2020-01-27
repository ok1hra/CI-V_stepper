/*
CI-V stepper - compile for Arduino Leonardo
-----------------------------------------------------------
https://remoteqth.com/civ-stepper.php

 ___               _        ___ _____ _  _
| _ \___ _ __  ___| |_ ___ / _ \_   _| || |  __ ___ _ __
|   / -_) '  \/ _ \  _/ -_) (_) || | | __ |_/ _/ _ \ '  \
|_|_\___|_|_|_\___/\__\___|\__\_\|_| |_||_(_)__\___/_|_|_|


  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Credit Trinamic_TMC2130.h driver
  @author   Moritz Walter
  https://github.com/makertum/Trinamic_TMC2130

Features:
---------
- control stepper Nema 17 by received frequency from Icom CI-V protocol
- support endstop for adjusting after start up
- without endstop available save lat position to eeprom (not recomended)
- configure via serial USB Command Line Interface (CLI)
- TX inhibit output during stepper operate
- up to 112 frequency memory
- up to 16 stepper counter memory for each frequency, selected by BCD inputs
- linear iterpolation on/off adjustable in CLI
- reverse on/off adjustable in CLI
- running/tuning mode in CLI
- tuning wizard for save to eeprom memory in CLI

Changelog:
----------
2020-01 CLI enter, uSteps settings
2019-05 first initial version

// Stepper //////////////////////////////////////////*/
#define RestoreMemoryFromEeprom     // enable load freq table from EEPROM during reboot
                                    // if disable, store table to eeprom with [s] command for every bank separately, and enable again
const int StepsByTurn = 200;        // dependancy to using stepper motor
int MicroSteps;                     // [1,2,4,8,16,32,64,128,256] number of microstep
int CurrentRun;                     // [0-31] current using during running stepper motor
int CurrentStandby = 0;             // [0-31] current using during stepper motor standby
long CurrentRunTimeout[2] = {0,1000};  // Time, after them current switch to standby value
bool EnableStepperStandby = false;   // disable stepper (off) in standby - ONLY if MicroSteps = 1
bool EnableSlowStartStop = false;
bool Reverse;
bool TuningMode = 0;
byte BankOfMemory;
byte BankOfMemoryPrev;
const int BcdPin[4] = {9, 11, 5, A1};  // BCD encoder PINs
long BcdRefresh[2] = {0,500};
int NumberOfMemory;
int NumberOfBank;
bool EnableEndstop;
bool LastPositionSave = true;
bool LinearInterpolation;

// Misc ///////////////////////////////////////////
#define CLI                     // Command Line Interface
// #define SERIAL_debug
int DebugLevel = 0;
bool SerialNeedStatus = false;

// Icom CI-V ///////////////////////////////////////////
#define ICOM_CIV                // read frequency from CIV
#if defined(ICOM_CIV)
  byte CIV_ADRESS;              // CIV input HEX Icom adress (0x is prefix)
  #define REQUEST  500          // [ms] use TXD output for sending frequency request
  long BAUDRATE1;               // [baud] Serial port in/out baudrate
#endif

// Ethernet ///////////////////////////////////////////
// #define EthModule             // enable Ethernet module if installed
#define __USE_DHCP__          // enable DHCP
#define HW_BCD_SW             // Enable hardware NET-ID bcd switch on bottom side

// Hardware ///////////////////////////////////////////
#define ENCODER     // select
#define BUTTON      // SHORT auto/select-memory/select-step
                    // LONG      /clear-memory /save-step and goto select-memory

// #define LCD                         // Uncoment to Enable I2C LCD
#if defined(LCD)
  const byte LcdI2Caddress = 0x27;    // 0x27 0x3F - may be find with I2C scanner https://playground.arduino.cc/Main/I2cScanner
  #define LCD_PCF8574                 // If LCD uses PCF8574 chip
  //#define LCD_PCF8574T              // If LCD uses PCF8574T chip
  //#define LCD_PCF8574AT             // If LCD uses PCF8574AT chip
#endif

const char* REV = "20200127";

#include "Trinamic_TMC2130.h"
const int StepPin = 8;
const int CS_PIN = 6;
const int DIR_PIN = 4;
const int EN_PIN = 12;
// bool CurrentStatus = false;
Trinamic_TMC2130 myStepper(CS_PIN);
const int Stepper_us = 200; // one step in us (SPEED)
int StepCounter;

const int EndStopPin = 7;
const int TxInhibitPin = 13;

// Serial
const int BAUDRATE0 = 115200; // USB CLI/Debug
bool WizardEnable = false;
int WizardStep = 0;

#if defined(LCD)
  #if defined(LCD_PCF8574T) || defined(LCD_PCF8574)
    #include <LiquidCrystal_PCF8574.h>
    LiquidCrystal_PCF8574 lcd(LcdI2Caddress);
  #endif
  #if defined(LCD_PCF8574AT)
    #include <LiquidCrystal_I2C.h>
    LiquidCrystal_I2C lcd(LcdI2Caddress,16,2);
  #endif

  long LcdRefresh[2]{0,500};
  // byte LockChar[8] = {0b00100, 0b01010, 0b01010, 0b11111, 0b11011, 0b11011, 0b11111, 0b00000};
  uint8_t LockChar[8] = {0x4,0xa,0xa,0x1f,0x1b,0x1b,0x1f,0x0};
  uint8_t EthChar[8] = {0x0,0x0,0x1f,0x11,0x11,0x1b,0x1f,0x0};
  bool LcdNeedRefresh = false;
#endif

// Steppers
unsigned int Multiplier = 1;
int uSeconds;
volatile long SteppersCounter = 0;
volatile long SteppersCounterTarget = 0;
long SteppersCounterLimit;
volatile int InDeCrement;

unsigned long StorageFreqToStep[112][2] = { //0 - 4 294 967 295
// {freq Hz,	uStep}
  // 0
  {7000000, 0},
  {7010000, 500},
  {7020000, 1000},
  {7030000, 1500},
  {7040000, 2000},
  {7050000, 2500},
  {7060000, 3000},
  {7070000, 3500},
  {7080000, 4000},
  {7090000, 4500},
  // 10
  {1830000, 5000},
  {1831000, 5500},
  {1832000, 6000},
  {1833000, 6500},
  {1834000, 7000},
  {1835000, 7500},
  {1836000, 8000},
  {1837000, 8500},
  {1838000, 9000},
  {1839000, 9500},
  // 20
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  // 30
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  // 40
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  // 50
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  // 60
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  // 70
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  // 80
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  // 90
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  // 100
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  // 110
  {0, 0},
  {0, 0},
  // 112
};

#if defined(ICOM_CIV)
  long RequestTimeout[2]={0,
    #if defined(REQUEST)
      REQUEST
    #else
      0
    #endif
  };
  int fromAdress = 0xE0;    // 0E
  byte rdI[10];             // read data icom
  String rdIS;              // read data icom string
  int state = 1;            // state machine
  bool StateMachineEnd = false;
#endif

unsigned long freq = 0;
unsigned long freqPrev;
byte InputByte[21];
byte incomingByte = 0;
byte incomingByte1 = 0;

#if defined(EthModule)
  const byte RemoteDevice = 'g';
  const byte ThisDevice = 'p';
  bool EthLinkStatus = 0;
  long EthLinkStatusTimer[2]={1500,1000};
  byte NET_ID = 0x00;         // NetID [hex] MUST BE UNIQUE IN NETWORK - replace by P6 board encoder
  bool EnableEthernet = 1;
  bool EnableDHCP     = 1;
  #include <Ethernet.h>
  #include <EthernetUdp.h>
  // #include <Ethernet2.h>
  // #include <EthernetUdp2.h>
  //  #include <util.h>
  // #include <Dhcp.h>
  // #include <EthernetServer.h>
  #include <SPI.h>
  byte LastMac = 0x00 + NET_ID;

  byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x22, LastMac};
  // byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, LastMac};
  // IPAddress ip(192, 168, 1, 222);         // IP
  // IPAddress gateway(192, 168, 1, 200);    // GATE
  // IPAddress subnet(255, 255, 255, 0);     // MASK
  // IPAddress myDns(8, 8, 8, 8);            // DNS (google pub)
  // EthernetServer server(80);              // Web server PORT
  // String HTTP_req;

  unsigned int UdpCommandPort = 88;       // local UDP port listen to command
  #define UDP_TX_PACKET_MAX_SIZE 40       // MIN 30
  unsigned char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
  // unsigned char packetBuffer[12]; //buffer to hold incoming packet,
  int UDPpacketSize;
  EthernetUDP UdpCommand; // An EthernetUDP instance to let us send and receive packets over UDP
  IPAddress BroadcastIP(0, 0, 0, 0);        // Broadcast IP address
  int BroadcastPort       = 88;             // destination broadcast packet port
  // IPAddress RemoteSwIP(0, 0, 0, 0);         // remote UDP IP switch - set from UDP DetectRemote array
  // int RemoteSwPort         = 0;             // remote UDP IP switch port
  // int BandDecoderChange    = 0;             // If band change, send query packet to Remote IP switch
  long RemoteSwLatency[2];                  // start, stop mark
  byte RemoteSwLatencyAnsw = 0;             // answer (offline) detect
  byte TxUdpBuffer[8];
  long IpTimeout[1][2] = {0, 60000};          // UDP Broadcast packet [8][0-timer/1-timeout]
#endif

#include <EEPROM.h>

//------------------------------------------------------------------------------------

void setup(){
  pinMode(StepPin, OUTPUT);
    digitalWrite(StepPin, LOW); // no step yet
  pinMode(CS_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, LOW);
  pinMode(EN_PIN, OUTPUT);
  pinMode(EndStopPin, INPUT_PULLUP);
  pinMode(TxInhibitPin, OUTPUT);
    digitalWrite(TxInhibitPin, LOW);
  for (int i = 0; i < 4; i++) {
   pinMode(BcdPin[i], INPUT);
  }

  Serial.begin(BAUDRATE0, SERIAL_8N2);

  // EEPROM 0-99 byte free, 100-1000 freq/ustep array
  #if defined(EthModule)
    NET_ID=EEPROM.read(1); // address, value
    LastMac = 0x00 + NET_ID;
    mac[5] = LastMac;
    EthernetCheck();
  #endif
  Reverse = EEPROM.read(2);
  BankOfMemory = EEPROM.read(3);
  if(BankOfMemory>15){
    BankOfMemory=0;
  }
  BankOfMemoryPrev=BankOfMemory;
  SteppersCounterLimit=EEPROMReadlong(4);

  NumberOfMemory=EEPROM.read(8);
  if(NumberOfMemory>112){
    NumberOfMemory=112;
  }
  NumberOfBank=224/(NumberOfMemory+1);
  if(NumberOfBank>16){
    NumberOfBank=16;
  }

  EnableEndstop=EEPROM.read(9);
  if(EnableEndstop>1){
    EnableEndstop=false;
  }
  if(EnableEndstop==true){
    SteppersCounter = SteppersCounterLimit;
    SteppersCounterTarget = 0;
  }

  if(EnableEndstop==false){
    SteppersCounter=EEPROMReadlong(10);
    SteppersCounterTarget = SteppersCounter;
  }


  #if defined(ICOM_CIV)
    CIV_ADRESS=EEPROM.read(14);
  #endif

  #if defined(ICOM_CIV)
    BAUDRATE1=EEPROMReadlong(15);
    if(BAUDRATE1<80 || BAUDRATE1>115200){
      BAUDRATE1=9600;
    }
    Serial1.begin(BAUDRATE1, SERIAL_8N2);
  #endif

  MicroSteps=EEPROM.read(19);
  if(MicroSteps<0 || MicroSteps>7){
    MicroSteps=16;
  }

  CurrentRun=EEPROM.read(20);
  if(CurrentRun<0 || CurrentRun>31){
    CurrentRun=10;
  }

  // stepper 1
  myStepper.init();
  myStepper.set_mres(MicroSteps); // ({1,2,4,8,16,32,64,128,256}) number of microsteps
  myStepper.set_IHOLD_IRUN(0,0,0); // ([0-31],[0-31],[0-5]) sets all currents to maximum
  myStepper.set_I_scale_analog(1); // ({0,1}) 0: I_REF internal, 1: sets I_REF to AIN
  myStepper.set_tbl(1); // ([0-3]) set comparator blank time to 16, 24, 36 or 54 clocks, 1 or 2 is recommended
  myStepper.set_toff(8); // ([0-15]) 0: driver disable, 1: use only with TBL>2, 2-15: off time setting during slow decay phase
  myStepper.set_en_pwm_mode(1); // StealtChop mode 1-ON 0-OFF
  uStepTouSeconds();
  if(MicroSteps>1){
    EnableStepperStandby = true;
  }
  if(EnableStepperStandby==true){
    digitalWrite(EN_PIN, LOW);    // LOW = enable
  }else{
    digitalWrite(EN_PIN, HIGH);    // HIGH = disable
  }

  LinearInterpolation=EEPROM.read(21);

  // for (int x=0; x<224; x++) {
  //   EEPROMWritelong( x*4+100, x);
  // }
  // set array from eeprom
  #if defined(RestoreMemoryFromEeprom)
    for (int x=0; x<NumberOfMemory; x++) {
      StorageFreqToStep[x][0] = EEPROMReadlong( x*((NumberOfBank+1)*4)+100 );
      StorageFreqToStep[x][1] = EEPROMReadlong( x*((NumberOfBank+1)*4)+100+4*(BankOfMemory+1) );
    }
  #endif

  #if defined(LCD)
    #if defined(LCD_PCF8574T) || defined(LCD_PCF8574)
      lcd.begin(16, 2); // initialize the lcd PFC8574
      lcd.setBacklight(1);
    #else
      //------------------------------------------------------
      // Enable begin or init in dependence on the GUI version
      // lcd.begin();
      lcd.init();
      lcd.backlight();
      //------------------------------------------------------
    #endif

    lcd.createChar(0, LockChar);
    #if defined(EthModule)
      lcd.createChar(1, EthChar);
    #endif
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Version:");
    lcd.setCursor(7,1);
    lcd.print(REV);
    delay(1000);
    lcd.clear();
    lcd.setCursor(0,0);
      #if defined(ICOM_CIV)
        lcd.print("ICOM        ");
        lcd.print(CIV_ADRESS, HEX);
        lcd.print("h");
      #endif
    lcd.clear();
  #endif

  InterruptON(1); // endstop
  delay(100);
  Serial.println(F("STEPPER START - ? for more info"));
  // homing(0);
}
//------------------------------------------------------------------------------------

void loop(){
  EthernetCheck();
  BcdCheck();
  LcdDisplay();
  FrequencyRequest();
  BandDecoderInput();
  SerialCLI();
  WizardCLI();
  OnTheFlyStepperControl();
  StepperWatchdog();
}

//------------------------------------------------------------------------------------
// SUBROUTINES ----------------------------------------------------------------------

void SerialCLI(){
  #if defined(CLI)
    if (Serial.available() > 0) {
            incomingByte = Serial.read();

            // w
          if(incomingByte==119 && TuningMode==1){
            WizardEnable=true;
            Serial.println();
            Serial.print(F("Wizard started..."));
            if(BankOfMemory==0){
              Serial.print(F(" WARNING - Bank-0 overwrite frequency (/ for escape tuning mode)"));
            }
            Serial.println();

          #if defined(SERIAL_debug)
              // *
            }else if(incomingByte==42){
               Serial.println(F("Press Debug level 0-2..."));
               Serial.print(F("> "));
               while (Serial.available() == 0) {
                 // Wait
               }
               Serial.println();
               incomingByte = Serial.read();
               if( incomingByte>=48 && incomingByte<=50 ){
                 DebugLevel=incomingByte-48;
                 Serial.print(F("** Serial DEBUG level "));
                 Serial.print(DebugLevel);
                 Serial.println(F(" **"));
               }else{
                 Serial.print(F(" ["));
                 Serial.write(incomingByte); //, DEC);
                 Serial.println(F("] unknown command - for more info press '?'"));
               }
           #endif

           // r
         }else if(incomingByte==114){
           Reverse= !Reverse;
           EEPROM.write(2, Reverse); // address, value
           Serial.print(F(" ** Reverse "));
           if(Reverse==0){
             Serial.println(F("[OFF] **"));
           }else if(Reverse==1){
             Serial.println(F("[ON]**"));
           }

           // +
         }else if(incomingByte==43){
           EnableEndstop= !EnableEndstop;
           EEPROM.write(9, EnableEndstop); // address, value
           Serial.print(F(" ** Enable "));
           if(EnableEndstop==true){
             Serial.println(F("[Endstop] **"));
           }else if(EnableEndstop==false){
             Serial.println(F("[Last position save to eeprom] **"));
             EEPROMWritelong(10, SteppersCounter);
           }

           // i
         }else if(incomingByte==105){
           LinearInterpolation=!LinearInterpolation;
           Serial.print(F(" ** Linear Interpolation "));
           if(LinearInterpolation==false){
             Serial.println(F("[OFF] **"));
           }else if(LinearInterpolation==true){
             Serial.println(F("[ON]**"));
           }
           EEPROM.write(21, LinearInterpolation);

           // /
         }else if(incomingByte==47){
           TuningMode=!TuningMode;
           Serial.print(F(" ** Tuning mode "));
           if(TuningMode==0){
             Serial.println(F("[OFF] **"));
             WizardEnable=false;
             WizardStep=0;
           }else if(TuningMode==1){
             Serial.println(F("[ON]**"));
           }

         // u
       }else if(incomingByte==117 && TuningMode==1){
         Serial.print(F("Save actual u step counter = "));
         Serial.print(SteppersCounter);
         Serial.print(F(" = "));
         float GRAD = 360/(float(StepsByTurn)*float(MicroSteps))*float(SteppersCounter);
         Serial.print(GRAD);
         Serial.println(F("° as upper limit?"));
         Serial.print(F("> [y/n] or [r] for reset to upper limit "));
         while (Serial.available() == 0) {
           // Wait
         }
         incomingByte = Serial.read();
         if(incomingByte==121){
           SteppersCounterLimit=SteppersCounter;
           EEPROMWritelong(4, SteppersCounterLimit);
           Serial.println(F("Save"));
        }else if(incomingByte==114){
           SteppersCounterLimit=2147483647L;
           EEPROMWritelong(4, SteppersCounterLimit);
           Serial.println(F("Reset"));
         }else{
           Serial.println(F("Discard"));
         }

           // 4 <- CCW
         }else if(incomingByte==52 && TuningMode==1){
            if( (SteppersCounterTarget-(MicroSteps*Multiplier))>=0 ){
              SteppersCounterTarget=SteppersCounterTarget-(MicroSteps*Multiplier);
              Serial.print(F("<"));
              // Serial.println(SteppersCounterTarget);
              #if defined(LCD)
                LcdNeedRefresh = true;
              #endif
              SerialNeedStatus = true;
            }

           // 6 -> CW
           }else if(incomingByte==54 && TuningMode==1){
             SteppersCounterTarget=SteppersCounterTarget+(MicroSteps*Multiplier);
             Serial.print(F(">"));
              // Serial.println(SteppersCounterTarget);
              #if defined(LCD)
                LcdNeedRefresh = true;
              #endif
              SerialNeedStatus = true;

           // 8 up
          }else if(incomingByte==56 && TuningMode==1){
            if(Multiplier<1000){
             Multiplier=Multiplier*10;
             Serial.print(F("Multiplier up to "));
             Serial.print(Multiplier);
             Serial.println(F("x"));
            }
           // 2 dwn
          }else if(incomingByte==50 && TuningMode==1){
            if(Multiplier>=10){
              Multiplier=Multiplier/10;
              Serial.print(F("Multiplier dwn to "));
              Serial.print(Multiplier);
              Serial.println(F("x"));
            }

           // ?
           }else if(incomingByte==63){
             ListCommands();

           // c
         }else if(incomingByte==99){
             Serial.println(F("Press CI-V address in hex (two chars 0-f)"));
             Serial.print(F("> "));
             while (Serial.available() == 0) {
               // Wait
             }
             incomingByte = Serial.read();
             Serial.write(incomingByte);
             if( (incomingByte>=48 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=102) ){
               if(incomingByte>=48 && incomingByte<=57){
                 incomingByte = incomingByte-48;
               }else{
                 incomingByte = incomingByte-87;
               }
               CIV_ADRESS = incomingByte;
               CIV_ADRESS = CIV_ADRESS<<4;
               while (Serial.available() == 0) {
                 // Wait
               }
               incomingByte = Serial.read();
               Serial.write(incomingByte);
               if( (incomingByte>=48 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=102) ){
                 if(incomingByte>=48 && incomingByte<=57){
                   incomingByte = incomingByte-48;
                 }else{
                   incomingByte = incomingByte-87;
                 }
                 CIV_ADRESS = CIV_ADRESS | incomingByte;
                 EEPROM.write(14, CIV_ADRESS);
                 Serial.println();

                 Serial.println(F("Baudrate and [enter]"));
                 // Serial.print(F("> "));
                 // while(!Serial.available()){
                 // }
                 // delay(2000);
                 // long CompareInt = Serial.parseInt();
                 Enter();
                 int intBuf=0;
                 int mult=1;
                 for (int j=InputByte[0]; j>0; j--){
                   intBuf = intBuf + ((InputByte[j]-48)*mult);
                   mult = mult*10;
                 }
                 if(intBuf>=80 && intBuf<=115200){
                   BAUDRATE1 = intBuf;
                   EEPROMWritelong(15, BAUDRATE1);
                   Serial.println(F("  write to EEPROM"));
                   // Serial.println(BAUDRATE1);
                   Serial1.begin(BAUDRATE1, SERIAL_8N2);
                  //  Serial.print("** device will be restarted **");
                   delay(500);
                 }else{
                   Serial.println(F("Out of range."));
                 }
               }else{
                 Serial.println(F(" accepts 0-f, exit"));
               }
             }else{
               Serial.println(F(" accepts 0-f, exit"));
             }

           // @
         }else if(incomingByte==64){
               Serial.println(F("Press micro steps 0-8 (default 4) and [enter], respond 1 2 4 8 16 32 64 128 256 uStep"));
               // Serial.print(F("> "));
               // while (Serial.available() == 0) {
               //   // Wait
               // }
               // incomingByte = Serial.read();
               Enter();
               int intBuf=0;
               int mult=1;
               for (int j=InputByte[0]; j>0; j--){
                 intBuf = intBuf + ((InputByte[j]-48)*mult);
                 mult = mult*10;
               }
               if( (intBuf <= 8) ){
                 MicroSteps=0x00;
                 MicroSteps = MicroSteps | (1<<intBuf);    // Set the n-th bit
                 Serial.print(MicroSteps);
                 Serial.println(F(" microstep"));
                 EEPROM.write(19, MicroSteps);

                 Serial.println(F("Stepper run current 0-31 (default 10) and [enter]"));
                 // Serial.print(F("> "));
                 // while(!Serial.available()){
                 // }
                 // delay(2000);
                 // long CompareInt = Serial.parseInt();
                 Enter();
                 int intBuf=0;
                 int mult=1;
                 for (int j=InputByte[0]; j>0; j--){
                   intBuf = intBuf + ((InputByte[j]-48)*mult);
                   mult = mult*10;
                 }
                 if(intBuf>=0 && intBuf<=31){
                   CurrentRun = intBuf;
                   EEPROMWritelong(20, CurrentRun);
                   Serial.println(CurrentRun);
                   // stepper 1
                   myStepper.init();
                   myStepper.set_mres(MicroSteps); // ({1,2,4,8,16,32,64,128,256}) number of microsteps
                   myStepper.set_IHOLD_IRUN(0,0,0); // ([0-31],[0-31],[0-5]) sets all currents to maximum
                   myStepper.set_I_scale_analog(1); // ({0,1}) 0: I_REF internal, 1: sets I_REF to AIN
                   myStepper.set_tbl(1); // ([0-3]) set comparator blank time to 16, 24, 36 or 54 clocks, 1 or 2 is recommended
                   myStepper.set_toff(8); // ([0-15]) 0: driver disable, 1: use only with TBL>2, 2-15: off time setting during slow decay phase
                   myStepper.set_en_pwm_mode(1); // StealtChop mode 1-ON 0-OFF
                   uStepTouSeconds();
                 }else{
                   Serial.println(F(" accepts 0-31, exit"));
                 }
               }else{
                 Serial.println(F(" accepts 0-8, exit"));
               }

             // #
             #if defined(EthModule)
               }else if(incomingByte==35){
                   Serial.println(F("Press NET-ID X_ prefix 0-f..."));
                   Serial.print(F("> "));
                   while (Serial.available() == 0) {
                     // Wait
                   }
                   incomingByte = Serial.read();
                   if( (incomingByte>=48 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=102) ){
                     bitClear(NET_ID, 4);
                     bitClear(NET_ID, 5);
                     bitClear(NET_ID, 6);
                     bitClear(NET_ID, 7);
                     Serial.write(incomingByte);
                     Serial.println();
                     if(incomingByte>=48 && incomingByte<=57){
                       incomingByte = incomingByte-48;
                       incomingByte = (byte)incomingByte << 4;
                       NET_ID = NET_ID | incomingByte;
                       TxUdpBuffer[0] = NET_ID;
                     }else if(incomingByte>=97 && incomingByte<=102){
                       incomingByte = incomingByte-87;
                       incomingByte = (byte)incomingByte << 4;
                       NET_ID = NET_ID | incomingByte;
                       TxUdpBuffer[0] = NET_ID;
                     }
                     // sufix
                     #if !defined(HW_BCD_SW)
                       Serial.println(F("Press NET-ID _X sufix 0-f..."));
                       Serial.print(F("> "));
                       while (Serial.available() == 0) {
                         // Wait
                       }
                       incomingByte = Serial.read();
                       if( (incomingByte>=48 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=102) ){
                         bitClear(NET_ID, 0);
                         bitClear(NET_ID, 1);
                         bitClear(NET_ID, 2);
                         bitClear(NET_ID, 3);
                         Serial.write(incomingByte);
                         Serial.println();
                         if(incomingByte>=48 && incomingByte<=57){
                           incomingByte = incomingByte-48;
                           NET_ID = NET_ID | incomingByte;
                           TxUdpBuffer[0] = NET_ID;
                         }else if(incomingByte>=97 && incomingByte<=102){
                           incomingByte = incomingByte-87;
                           NET_ID = NET_ID | incomingByte;
                           TxUdpBuffer[0] = NET_ID;
                         }
                     #endif
                         EEPROM.write(1, NET_ID); // address, value
                         // EEPROM.commit();
                         Serial.print(F("** Now NET-ID change to 0x"));
                         Serial.print(NET_ID, HEX);
                         Serial.println(F(" **"));
                         #if defined(SERIAL_debug)
                           if(DebugLevel>1){
                             Serial.print(F("EEPROM read ["));
                             Serial.print(EEPROM.read(1), HEX);
                             Serial.println(F("]"));
                           }
                          #endif
                         TxUDP(ThisDevice, RemoteDevice, 'b', 'r', 'o');
                     #if !defined(HW_BCD_SW)
                       }else{
                         Serial.println(F(" accepts 0-f, exit"));
                       }
                     #endif
                   }else{
                     Serial.println(" accepts 0-f, exit");
                   }
             #endif

              #if !defined(RestoreMemoryFromEeprom)
                 // s
                }else if(incomingByte==115){
                  Serial.print(F("* Copy RAM table to EEPROM bank "));
                  Serial.println(BankOfMemory);
                  for (int x=0; x<NumberOfMemory; x++) {
                    if(BankOfMemory==0){
                      EEPROMWritelong( x*((NumberOfBank+1)*4)+100, StorageFreqToStep[x][0]);
                    }
                    EEPROMWritelong( x*((NumberOfBank+1)*4)+100+(4*(BankOfMemory+1)), StorageFreqToStep[x][1]);
                  }
              #endif

               // l
               }else if(incomingByte==108){
               Serial.println(F("Array[FREQ Hz][uSTEP]"));
               for (int x=0; x<NumberOfMemory; x++) {
                 Serial.print(F("#"));
                 Serial.print(x);
                 Serial.print(F(" {"));
                 Serial.print(StorageFreqToStep[x][0]);
                 Serial.print(F(", "));
                 for (int i=1; i<2; i++) {
                   Serial.print(StorageFreqToStep[x][i]);
                   if(i<1){
                     Serial.print(F(", "));
                   }
                 }
                 Serial.println(F("}"));
               }

            // m
            }else if(incomingByte==109){
              Serial.println(F("Set memory number between 2-112 and [enter]"));
              Serial.print(F("> "));
              // while(!Serial.available()){
              // }
              // delay(2000);
              //     int CompareInt = Serial.parseInt();
              Enter();
              int intBuf=0;
              int mult=1;
              for (int j=InputByte[0]; j>0; j--){
                intBuf = intBuf + ((InputByte[j]-48)*mult);
                mult = mult*10;
              }
                  if(intBuf>=2 && intBuf<=112){
                    NumberOfMemory = intBuf;
                    NumberOfBank=224/(NumberOfMemory+1);
                    if(NumberOfBank>16){
                      NumberOfBank=16;
                    }
                    EEPROM.write(8, NumberOfMemory);
                    Serial.print(F(" Set "));
                    Serial.print(NumberOfMemory);
                    Serial.print(F("memory / "));
                    Serial.print(NumberOfBank);
                    Serial.println(F(" bank"));
                  }else{
                    Serial.println(F(" Out of range."));
                  }

               // e
               }else if(incomingByte==101){
                 Serial.println(F("EEPROM #address kHz | uStep by bank..."));
                 for (int x=0; x<NumberOfMemory; x++) {
                   Serial.print(F("#"));
                   Serial.print(x*((NumberOfBank+1)*4)+100);
                   Serial.print(F(" "));
                   Serial.print(EEPROMReadlong( x*((NumberOfBank+1)*4)+100 ));
                   for (int y=0; y<NumberOfBank; y++) {
                     Serial.print(F(" | "));
                     Serial.print(EEPROMReadlong( x*((NumberOfBank+1)*4)+100+4*y+4 ));
                   }
                   Serial.println();
                 }
             }else{
               if(WizardEnable==false){
                 Serial.print(F(" ["));
                 Serial.write(incomingByte); //, DEC);
                 Serial.print(F("|"));
                 Serial.print(incomingByte, DEC);
                 Serial.print(F("|0x"));
                 Serial.print(incomingByte, HEX);
                 Serial.println(F("] unknown command - for more info press '?'"));
                //  ListCommands();
              }
           }
    }
  #endif
}
//-------------------------------------------------------------------------------------------------------
void Enter(){
  InputByte[0]=0;
  incomingByte = 0;
  bool br=false;
  Serial.print(F("> "));

  while(br==false) {
    if(Serial.available()){
      incomingByte=Serial.read();
      if(incomingByte==13){
        br=true;
        Serial.println("");
      }else{
        Serial.write(incomingByte);
        InputByte[InputByte[0]+1]=incomingByte;
        InputByte[0]++;
      }
      if(InputByte[0]==20){
        br=true;
        Serial.print(F(" too long"));
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------
void ListCommands(){
  #if defined(CLI)
    Serial.println();
    Serial.println(F("  ========================="));
    #if defined(EthModule)
    Serial.print(F("   IP stepper NET ID: "));
     Serial.println(NET_ID, HEX);
    #else
     Serial.println(F("   ICOM CI-V stepper"));
    #endif
    //  Serial.print(" [BCD-");
    // Serial.print(digitalRead(Id1Pin));
    // Serial.print(digitalRead(Id2Pin));
    // Serial.print(digitalRead(Id3Pin));
    //  Serial.println("]");
    Serial.println(F("  ========================="));
    Serial.print(F("  Firmware: "));
    Serial.println(REV);

    #if defined(EthModule)
     Serial.println();
     Serial.println(F("  Ethernet"));
     Serial.print(F("  - IP address:"));
     Serial.println(Ethernet.localIP());
     Serial.print(F("  - IP source ["));
     Serial.write(RemoteDevice);
     Serial.print(F("]: "));
     Serial.print(UdpCommand.remoteIP());
     Serial.print(F(":"));
     Serial.println(UdpCommand.remotePort());
    #endif

    Serial.println();
    Serial.println(F("  Stepper settings"));
    Serial.print(F("  - Actual u step counter = "));
    Serial.print(SteppersCounter);
    Serial.print(F(" = "));
    float GRAD = 360/(float(StepsByTurn)*float(MicroSteps))*float(SteppersCounter);
    Serial.print(GRAD);
    Serial.println("°");
    Serial.print(F("  - Steps by turn = "));
    Serial.println(StepsByTurn);
    Serial.print(F("  - Microsteps = "));
    Serial.println(MicroSteps);
    Serial.print(F("  - Current Run = "));
    Serial.println(CurrentRun);
    Serial.print(F("  - Current Standby = "));
    Serial.println(CurrentStandby);
    Serial.print(F("  - Current Run timeout [ms] = "));
    Serial.println(CurrentRunTimeout[1]);
    Serial.print(F("  - Enable stepper if standby = "));
    Serial.println(EnableStepperStandby);
    //  Serial.print(F("  - Enable slow start/stop = "));
    //  Serial.println(EnableSlowStartStop);
    Serial.print(F("  - One step uSeconds = "));
    Serial.println(uSeconds);

    #if defined(ICOM_CIV)
     Serial.println();
     Serial.println(F("  ICOM CI-V"));
     Serial.print(F("  - Actual frequency Hz = "));
     Serial.println(freq);
     Serial.print(F("  - Address = "));
     Serial.print(CIV_ADRESS, HEX);
     Serial.println("h");
     if(EnableEndstop==false){
       Serial.print(F("  - Request time [ms] = "));
       Serial.println(REQUEST);
     }
     Serial.print(F("  - Baudrate = "));
     Serial.println(BAUDRATE1);
    #endif
    Serial.println();
    Serial.println(F("  BCD input"));
    Serial.print(F("  - Actual Bank-"));
    Serial.println(BankOfMemory);
    Serial.println();
    Serial.println(F("  -----------------------------"));
    Serial.println(F("  Character commands:"));
    Serial.println(F("      ? for info"));
    Serial.println(F("      c CI-V preset (baud, address)"));
    Serial.println(F("      @ stepper settings"));
    Serial.print(F("      i Linear Interpolation ["));
    if(LinearInterpolation==true){
     Serial.println(F("ON]"));
    }else if(LinearInterpolation==false){
     Serial.println(F("OFF], hysteresis +-500Hz"));
    }
    Serial.print(F("      r reverse "));
    if(Reverse==0){
     Serial.println(F("[OFF]"));
    }else if(Reverse==1){
     Serial.println(F("[ON]"));
    }
    Serial.print(F("      + enable "));
    if(EnableEndstop==true){
     Serial.println(F("[Endstop]/Last position save to eeprom"));
    }else if(EnableEndstop==false){
     Serial.print(F("Endstop/[Last position save to eeprom] saved "));
     Serial.print(EEPROMReadlong(10));
     Serial.print(F(" = "));
     GRAD = 360/(float(StepsByTurn)*float(MicroSteps))*float(EEPROMReadlong(10));
     Serial.print(GRAD);
     Serial.println("°");
    }
    #if defined(SERIAL_debug)
      Serial.print(F("      * serial debug level ["));
      Serial.print(DebugLevel);
      Serial.println(F("] set"));
    #endif
    Serial.print(F("      m number of frequencies in memory ["));
    Serial.print(NumberOfMemory);
    Serial.print(F("]/"));
    Serial.print(NumberOfBank);
    Serial.println(F(" bank"));
    Serial.print(F("      l Freqency/uSteps listing for actual bank "));
    Serial.println(BankOfMemory);
    Serial.println(F("      e Freqency/uSteps EEPROM listing "));
    #if defined(EthModule)
     Serial.println(F("   #0-f network ID prefix [hex]"));
     #if !defined(HW_BCD_SW)
       Serial.println(F("        +network ID sufix [hex]"));
     #endif
    #endif
    #if !defined(RestoreMemoryFromEeprom)
    Serial.println(F("      s Save RAM freq table to EEPROM by selected bank"));
    #endif
    Serial.println();
    Serial.print(F("      / "));
    if(TuningMode==0){
     Serial.println(F("[running]/tuning mode"));
    }else if(TuningMode==1){
     Serial.println(F("running/[tuning] mode"));
    }
    if(TuningMode==1){
     Serial.print(F("      u upper ustepper counter limit ["));
     Serial.print(SteppersCounterLimit);
     Serial.print(F(" uStep] = "));
     GRAD = 360/(float(StepsByTurn)*float(MicroSteps))*float(SteppersCounterLimit);
     Serial.print(GRAD);
     Serial.println("°");
     Serial.println();
     Serial.println(F("      6 rotate CW"));
     Serial.println(F("      4 rotate CCW"));
     Serial.println(F("      8 step multiplier up"));
     Serial.println(F("      2 step multiplier down"));
       Serial.print(F("        actual step multiplier ["));
     Serial.print(Multiplier);
     Serial.println(F("x]"));
     Serial.println();
     Serial.print(F("      w tuning Wizard [bank "));
     Serial.print(BankOfMemory);
     if(WizardEnable==true){
       Serial.print(F("] actual wizard step ["));
       Serial.print(WizardStep);
     }
     Serial.println(F("]"));
    }
    Serial.println(F("  -----------------------------"));
  #endif
}

//---------------------------------------------------------------------------------------------------------
void WizardCLI(){
  #if defined(CLI)
    if(WizardEnable==true && TuningMode==1){
      int longer=String(freq/1000).length();
      switch (WizardStep%2) {
          case 0: // even
            if(BankOfMemory==0){
              Serial.print(F(" tune FREQUENCY and stepper, then press [5] for storage to mem "));
            }else{
              Serial.print(F(" Set frequency to "));
              Serial.print(StorageFreqToStep[(WizardStep)/2][0]);
              Serial.print(F(" kHz and move stepper, then press [5] for storage to mem "));
              txCIV(0, freq, CIV_ADRESS);         // 0 - set freq
            }
            Serial.print(WizardStep/2);
            Serial.print(F("/"));
            Serial.print(NumberOfMemory);
            Serial.print(F(" bank-"));
            Serial.println(BankOfMemory);
            WizardStep++;
            break;
          case 1: // odd
            // 5
            if(incomingByte==53){
              if(
                   (BankOfMemory==0 && (WizardStep==1 || ( freq>StorageFreqToStep[(WizardStep-1)/2-1][0] && SteppersCounter>StorageFreqToStep[(WizardStep-1)/2-1][1] )))
                || (BankOfMemory!=0 && (freq==StorageFreqToStep[(WizardStep-1)/2][0] && ( WizardStep==1 || SteppersCounter>StorageFreqToStep[(WizardStep-1)/2-1][1] )))
              ){
                Serial.println();
                Serial.print(F("* Set "));
                // if(BankOfMemory==0){
                  StorageFreqToStep[(WizardStep-1)/2][0]=freq;
                  Serial.print(String(freq/1000).substring(0, longer-3));
                  Serial.print(F(" "));
                  Serial.print(String(freq/1000).substring(longer-3, longer));
                  Serial.print(F("."));
                  Serial.print(String(freq).substring(longer-6, longer));
                  Serial.print(F("kHz "));
                // }
                StorageFreqToStep[(WizardStep-1)/2][1]=SteppersCounter;
                Serial.print(SteppersCounter);
                Serial.print(F("us to memory "));
                Serial.print((WizardStep-1)/2);
                Serial.print(F("/"));
                Serial.print(NumberOfMemory-1);
                WizardStep++;
                incomingByte=0x00;
                if(WizardStep==NumberOfMemory*2){
                  Serial.println(F(" - Save to EEPROM, wizard END and switch to RUN mode"));
                  Serial.println();
                  WizardStep=0;
                  WizardEnable=false;
                  TuningMode=0;
                  for (int x=0; x<NumberOfMemory; x++) {
                    if(BankOfMemory==0){
                      EEPROMWritelong( x*((NumberOfBank+1)*4)+100, StorageFreqToStep[x][0]);
                    }
                    EEPROMWritelong( x*((NumberOfBank+1)*4)+100+(4*(BankOfMemory+1)), StorageFreqToStep[x][1]);
                  }
                }
              }else{
                Serial.println();
                if(BankOfMemory==0){
                  Serial.println(F("* NOT set - freq and steps must be bigger than previous memory"));
                  Serial.print(freq);
                  Serial.print(F(">"));
                  Serial.print(StorageFreqToStep[(WizardStep-1)/2-1][0]);
                }else{
                  Serial.print(F("* NOT set - freq must be set to "));
                  Serial.print(StorageFreqToStep[(WizardStep-1)/2][0]);
                  Serial.println(F("kHz and steps must be bigger than previous memory"));
                  Serial.print(freq);
                  Serial.print(F("="));
                  Serial.print(StorageFreqToStep[(WizardStep-1)/2][0]);
                }
                Serial.print(F(" "));
                Serial.print(SteppersCounter);
                Serial.print(F(">"));
                Serial.println(StorageFreqToStep[(WizardStep-1)/2-1][1]);
                Serial.println();
                WizardStep--;
                incomingByte=0x00;
              }
            }
            break;
      }
    }
  #endif
}

//---------------------------------------------------------------------------------------------------------
void ShortStatus(){
  #if defined(SERIAL_debug) && defined(CLI)
    if(DebugLevel>0){
      int longer=String(freq/1000).length();
      if(longer<4){
        Serial.print(freq);
      }else{
        Serial.print(String(freq/1000).substring(0, longer-3));
        Serial.print(F(" "));
        Serial.print(String(freq/1000).substring(longer-3, longer));
        Serial.print(F("."));
        Serial.print(String(freq).substring(longer-6, longer));
      }
      Serial.print(F("kHz "));
      Serial.print(SteppersCounter);
      Serial.print(F("us "));
      float GRAD = 360/(float(StepsByTurn)*float(MicroSteps))*float(SteppersCounter);
      Serial.print(GRAD);
      Serial.println(F("°"));
    }
  #endif
}

//---------------------------------------------------------------------------------------------------------

void  LcdDisplay(){
  #if defined(LCD)
    if(millis()-LcdRefresh[0]>LcdRefresh[1] || LcdNeedRefresh == true){

      #if defined(EthModule)
        if(EthLinkStatus==0){
          lcd.setCursor(0, 0);
          lcd.print((char)1);   // EthChar
          lcd.print(F(" Please connect"));
          lcd.setCursor(0, 1);
          lcd.print(F("  ethernet      "));
        }else{
      #endif

        // STEP
        lcd.setCursor(1,0);
        Space(6, String(SteppersCounter).length(), ' ');
        lcd.print(SteppersCounter);
        lcd.print("|");
        float GRAD = 360/(float(StepsByTurn)*float(MicroSteps))*float(SteppersCounter);
        lcd.print(GRAD);
        lcd.print(char(223));
        lcd.print("   ");

        lcd.setCursor(0,1);
          lcd.print(BankOfMemory, HEX);
          Space(9, String(freq).length(), ' ');
          PrintFreq();
          lcd.setCursor(12,1);
          lcd.print(" kHz");

      LcdRefresh[0]=millis();
      LcdNeedRefresh = false;

      #if defined(EthModule)
      }
      #endif
    }
  #endif
}

//------------------------------------------------------------------------------------

#if defined(LCD)
  void PrintFreq(){
    int longer=String(freq/1000).length();
    if(longer<4){
      lcd.print(" ");
      lcd.print(freq);
    }else{
      lcd.print(String(freq/1000).substring(0, longer-3));
      lcd.print(" ");
      lcd.print(String(freq/1000).substring(longer-3, longer));
      lcd.print(".");
      lcd.print(String(freq).substring(longer-6, longer));
    }
  }
#endif

//---------------------------------------------------------------------------------------------------------

#if defined(LCD)
  void Space(int MAX, int LENGHT, char CHARACTER){
    int NumberOfSpace = MAX-LENGHT;
    if(NumberOfSpace>0){
      for (int i=0; i<NumberOfSpace; i++){
        lcd.print(CHARACTER);
      }
    }
  }
#endif

//---------------------------------------------------------------------------------------------------------

void BandDecoderInput(){
  #if defined(ICOM_CIV)
    if (Serial1.available() > 0) {
        incomingByte1 = Serial1.read();
        #if defined(SERIAL_debug)
          if(DebugLevel>1){
            Serial.print(F("CIV "));
            Serial.print(incomingByte1, HEX);
            Serial.println();
          }
        #endif
        icomSM(incomingByte1);
        rdIS="";
        // if(rdI[10]==0xFD){    // state machine end
        if(StateMachineEnd == true){    // state machine end
          StateMachineEnd = false;
          for (int i=9; i>=5; i-- ){
              if (rdI[i] < 10) {            // leading zero
                  rdIS = rdIS + 0;
              }
              rdIS = rdIS + String(rdI[i], HEX);  // append BCD digit from HEX variable to string
          }
          freq = rdIS.toInt();
          // Serial.println(freq);
          // Serial.println("-------");
          // FreqToBandRules();
          // bandSET();
          Tune();
          #if defined(LCD)
            LcdNeedRefresh=true;
          #endif
          RequestTimeout[0]=millis();
        }
    }
  #endif
}
//---------------------------------------------------------------------------------------------------------

int icomSM(byte b){      // state machine
  #if defined(ICOM_CIV)
    // This filter solves read from 0x00 0x05 0x03 commands and 00 E0 F1 address used by software
    // Serial.print(b, HEX);
    // Serial.print(" | ");
    // Serial.println(state);
    switch (state) {
        case 1: if( b == 0xFE ){ state = 2; rdI[0]=b; rdI[10]=0x00; }; break;
        case 2: if( b == 0xFE ){ state = 3; rdI[1]=b; }else{ state = 1;}; break;
        // addresses that use different software 00-trx, e0-pc-ale, winlinkRMS, f1-winlink trimode
        case 3: if( b == 0x00 || b == 0xE0 || b == 0x0E || b == 0xF1 ){ state = 4; rdI[2]=b;                       // choose command $03
        }else if( b == CIV_ADRESS ){ state = 6; rdI[2]=b;
                }else if( b == 0xFE ){ state = 3; rdI[1]=b;      // FE (3x reduce to 2x)
                }else{ state = 1;}; break;                       // or $05

        case 4: if( b == CIV_ADRESS ){ state = 5; rdI[3]=b; }else{ state = 1;}; break;                      // select command $03
        case 5: if( b == 0x00 || b == 0x03 ){state = 8; rdI[4]=b;  // freq
                }else if( b == 0x04 ){state = 14; rdI[4]=b;        // mode
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;        // FE
                }else{ state = 1;}; break;

        case 6: if( b == 0x00 || b == 0xE0 || b == 0xF1 ){ state = 7; rdI[3]=b; }else{ state = 1;}; break;  // select command $05
        case 7: if( b == 0x00 || b == 0x05 ){ state = 8; rdI[4]=b; }else{ state = 1;}; break;

        case 8: if( b <= 0x99 ){state = 9; rdI[5]=b;             // 10Hz 1Hz
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1;}; break;
        case 9: if( b <= 0x99 ){state = 10; rdI[6]=b;            // 1kHz 100Hz
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1;}; break;
       case 10: if( b <= 0x99 ){state = 11; rdI[7]=b;            // 100kHz 10kHz
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1;}; break;
       case 11: if( b <= 0x52 ){state = 12; rdI[8]=b;            // 10MHz 1Mhz
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1;}; break;
       case 12: if( b <= 0x01 || b == 0x04){state = 13; rdI[9]=b; // 1GHz 100MHz  <-- 1xx/4xx MHz limit
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1;}; break;
       case 13: if( b == 0xFD ){state = 1; rdI[10]=b; StateMachineEnd = true;
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1; rdI[10] = 0x00;}; break;

       case 14: if( b <= 0x12 ){state = 15; rdI[5]=b;
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1;}; break;   // Mode
       case 15: if( b <= 0x03 ){state = 16; rdI[6]=b;
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1;}; break;   // Filter
       case 16: if( b == 0xFD ){state = 1; rdI[7]=b;
                }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                }else{state = 1; rdI[7] = 0;}; break;
    }
    #endif
}

//---------------------------------------------------------------------------------------------------------

int txCIV(int commandCIV, long dataCIVtx, int toAddress) {
  #if defined(ICOM_CIV)
    Serial1.write(254);                                    // FE
    Serial1.write(254);                                    // FE
    Serial1.write(toAddress);                              // to adress
    Serial1.write(fromAdress);                             // from OE
    Serial1.write(commandCIV);                             // data
    if (dataCIVtx != 0){
        String freqCIVtx = String(dataCIVtx);             // to string
        String freqCIVtxPart;
        while (freqCIVtx.length() < 10) {                 // leding zeros
            freqCIVtx = 0 + freqCIVtx;
        }
        for (int x=8; x>=0; x=x-2){                       // loop for 5x2 char [xx xx xx xx xx]
            freqCIVtxPart = freqCIVtx.substring(x,x+2);   // cut freq to five part
                Serial1.write(hexToDec(freqCIVtxPart));    // HEX to DEC, because write as DEC format from HEX variable
        }
    }
    Serial1.write(253);                                    // FD
    #if defined(SERIAL_debug)
      if(DebugLevel>1){
        Serial.println(F("txCIV"));
      }
    #endif
    // Serial1.flush();
    while(Serial1.available()){        // clear buffer
      Serial1.read();
    }
  #endif
}
//---------------------------------------------------------------------------------------------------------

unsigned int hexToDec(String hexString) {
  #if defined(ICOM_CIV)
    unsigned int decValue = 0;
    int nextInt;
    for (int i = 0; i < hexString.length(); i++) {
        nextInt = int(hexString.charAt(i));
        if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
        if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
        if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
        nextInt = constrain(nextInt, 0, 15);
        decValue = (decValue * 16) + nextInt;
    }
    return decValue;
  #endif
}
//---------------------------------------------------------------------------------------------------------

void FrequencyRequest(){
  #if defined(REQUEST) && defined(ICOM_CIV)

  if((millis() - RequestTimeout[0] > RequestTimeout[1] && EnableEndstop==false) ){ // Request enable only if disable endstop - undefined bug in txCIV
    txCIV(3, 0, CIV_ADRESS);  // ([command], [freq]) 3=read
    RequestTimeout[0]=millis();
  }
  #endif
}
//---------------------------------------------------------------------------------------------------------

void Tune(){
  // LinearInterpolation
  if(freqPrev!=freq && TuningMode==0){
    if(LinearInterpolation==true){
      for (int i = 0; i < NumberOfMemory; i++) {
          if(StorageFreqToStep[i][0] <= freq && freq <= StorageFreqToStep[i+1][0]){   // find range
              SteppersCounterTarget = map(freq, StorageFreqToStep[i][0], StorageFreqToStep[i+1][0], StorageFreqToStep[i][1], StorageFreqToStep[i+1][1]);  // map(value, fromLow, fromHigh, toLow, toHigh) //*10 one decimal
              SerialNeedStatus=true;
              break;
          }
      }
      freqPrev=freq;
    // LinearInterpolation OFF
    }else{
      const int Hysteresis = 500;
      for (int i = 0; i < NumberOfMemory; i++) {
        if( (i==0
          && (StorageFreqToStep[i+1][0]-StorageFreqToStep[i][0])/2+StorageFreqToStep[i][0] >= freq
          && abs(freq-freqPrev)>Hysteresis )
          || (i!=0 && i!=NumberOfMemory-1
          && (StorageFreqToStep[i][0]-StorageFreqToStep[i-1][0])/2+StorageFreqToStep[i-1][0] <= freq
          && (StorageFreqToStep[i+1][0]-StorageFreqToStep[i][0])/2+StorageFreqToStep[i][0] >= freq
          && abs(freq-freqPrev)>Hysteresis )
          || ( i==NumberOfMemory-1
          && (StorageFreqToStep[i][0]-StorageFreqToStep[i-1][0])/2+StorageFreqToStep[i-1][0] <= freq
          && abs(freq-freqPrev)>Hysteresis )
          ){
            SteppersCounterTarget = StorageFreqToStep[i][1];
            freqPrev=freq;
            SerialNeedStatus=true;
            break;
        }
      } // end for
    } // end LinearInterpolation OFF
  } // end TuningMode
}
//------------------------------------------------------------------------------------

void OnTheFlyStepperControl(){
  if(SteppersCounterTarget<=SteppersCounterLimit){
    if(SteppersCounterTarget!=SteppersCounter){
      myStepper.set_IHOLD_IRUN(CurrentRun,CurrentRun,5); // ([0-31],[0-31],[0-5]) sets all currents to maximum
      if(EnableStepperStandby==false){
        digitalWrite(EN_PIN, LOW);    // LOW = enable
      }
      digitalWrite(TxInhibitPin, HIGH);
      #if defined(LCD)
        lcd.setCursor(0, 0);
        lcd.print((char)0);   // LockChar
      #endif

      if(SteppersCounterTarget<SteppersCounter){
        digitalWrite(DIR_PIN, !Reverse);
        InDeCrement = -1;
      }else if(SteppersCounterTarget>SteppersCounter){
        digitalWrite(DIR_PIN, Reverse);
        InDeCrement = 1;
      }
      while(SteppersCounterTarget!=SteppersCounter){
        // InterruptON(0);
        digitalWrite(StepPin, HIGH);
        delayMicroseconds(uSeconds);
        digitalWrite(StepPin, LOW);
        delayMicroseconds(uSeconds);
        SteppersCounter=SteppersCounter+InDeCrement;
      }
      CurrentRunTimeout[0] = millis();
      LastPositionSave=false;
    }
    if(SerialNeedStatus==true){
      ShortStatus();
      SerialNeedStatus=false;
    }
  }else{
    #if defined(SERIAL_debug)
      if(DebugLevel>0){
        Serial.print(F("Target "));
        Serial.print(SteppersCounterTarget);
        Serial.print(F(" over counter limit "));
        Serial.println(SteppersCounterLimit);
        SteppersCounterTarget=SteppersCounterLimit;
      }
    #endif
  }
}

//------------------------------------------------------------------------------------
void uStepTouSeconds(){
  // Microstep long in uSeconds
  switch (MicroSteps) {
      case 1: uSeconds = 2000 ; break;
      case 2: uSeconds = 700 ; break;
      case 4: uSeconds = 600 ; break;
      case 8: uSeconds = 300 ; break;
      case 16: uSeconds = 200 ; break;
      case 32: uSeconds = 60 ; break;
      case 64: uSeconds = 16 ; break;
      case 128: uSeconds = 11 ; break;
      case 256: uSeconds = 11 ; break;
  }
}
//------------------------------------------------------------------------------------

void InterruptON(int endstop){
  if(endstop==0 && EnableEndstop==true){
    detachInterrupt(digitalPinToInterrupt(EndStopPin));
    #if defined(SERIAL_debug)
    if(DebugLevel>0){
      Serial.print(F("* Interrupt OFF *"));
    }
    #endif
  }else if(endstop==1 && EnableEndstop==true){
    attachInterrupt(digitalPinToInterrupt(EndStopPin), EndstopNow, FALLING);  // need detachInterrupt in RX_UDP() subroutine
    #if defined(SERIAL_debug)
    if(DebugLevel>0){
      Serial.print(F("* Interrupt ON *"));
    }
    #endif
  }
}

//---------------------------------------------------------------------------------------------------------
void EndstopNow(){
  SteppersCounter = 0-InDeCrement;
  #if defined(SERIAL_debug)
  if(DebugLevel>0){
    Serial.println(F("* INTERRUPT NOW *"));
  }
  #endif
}

//------------------------------------------------------------------------------------
//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to address + 3.
void EEPROMWritelong(int address, long value){
     //Decomposition from a long to 4 bytes by using bitshift.
     //One = Most significant -> Four = Least significant byte
     byte four = (value & 0xFF);
     byte three = ((value >> 8) & 0xFF);
     byte two = ((value >> 16) & 0xFF);
     byte one = ((value >> 24) & 0xFF);

     //Write the 4 bytes into the eeprom memory.
     EEPROM.write(address, four);
     EEPROM.write(address + 1, three);
     EEPROM.write(address + 2, two);
     EEPROM.write(address + 3, one);
 }

//------------------------------------------------------------------------------------

long EEPROMReadlong(long address){
     //Read the 4 bytes from the eeprom memory.
     long four = EEPROM.read(address);
     long three = EEPROM.read(address + 1);
     long two = EEPROM.read(address + 2);
     long one = EEPROM.read(address + 3);

     //Return the recomposed long by using bitshift.
     return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

//---------------------------------------------------------------------------------------------------------
void BcdCheck(){
  if(millis()-BcdRefresh[0]>BcdRefresh[1]){
    BankOfMemory=GetBcd();
    if(BankOfMemory!=BankOfMemoryPrev){
      BankOfMemoryPrev=BankOfMemory;
      EEPROM.write(3, BankOfMemory);
      Serial.println();
      Serial.print(F("* Bank change to "));
      Serial.print(BankOfMemory);
      Serial.println(F(", load memory from EEPROM"));
      // set array from eeprom
      for (int x=0; x<NumberOfMemory; x++) {
        // StorageFreqToStep[x][0] = EEPROMReadlong( x*((NumberOfBank+1)*4)+100 );
        StorageFreqToStep[x][1] = EEPROMReadlong( x*((NumberOfBank+1)*4)+100+4*(BankOfMemory+1) );
      }
    }
    BcdRefresh[0]=millis();
  }
}

//------------------------------------------------------------------------------------
void StepperWatchdog(){
    if(millis()-CurrentRunTimeout[0]>CurrentRunTimeout[1]){
      myStepper.set_IHOLD_IRUN(CurrentStandby,CurrentStandby,0); // ([0-31],[0-31],[0-5]) sets all currents to maximum
      if(EnableStepperStandby==false){
        digitalWrite(EN_PIN, HIGH);    // LOW = enable
      }
      digitalWrite(TxInhibitPin, LOW);
      if(EnableEndstop==false && LastPositionSave==false){
        EEPROMWritelong(10, SteppersCounter);
        LastPositionSave=true;
        #if defined(SERIAL_debug)
          if(DebugLevel>0){
            Serial.println(F("* Last position save to eeprom *"));
          }
        #endif
      }
      #if defined(LCD)
        lcd.setCursor(0, 0);
        lcd.print(" ");
      #endif
    }
}

//---------------------------------------------------------------------------------------------------------
byte GetBcd(){
byte BCD = 0;
if(digitalRead(BcdPin[0])==0){
BCD = BCD | (1<<0);    // Set the n-th bit
}
if(digitalRead(BcdPin[1])==0){
BCD = BCD | (1<<1);    // Set the n-th bit
}
if(digitalRead(BcdPin[2])==0){
BCD = BCD | (1<<2);    // Set the n-th bit
}
if(digitalRead(BcdPin[3])==0){
BCD = BCD | (1<<3);    // Set the n-th bit
}
if(BCD>NumberOfBank-1){
  BCD=NumberOfBank-1;
}
return BCD;
}

//---------------------------------------------------------------------------------------------------------

void TxUDP(byte FROM, byte TO, byte A, byte B, byte C){
  #if defined(EthModule)
    InterruptON(0); // ptt, enc

    // TxUdpBuffer[0] = NET_ID;
    TxUdpBuffer[1] = FROM;
    TxUdpBuffer[2] = TO;
    TxUdpBuffer[3] = B00111010;           // :
    TxUdpBuffer[4] = A;
    TxUdpBuffer[5] = B;
    TxUdpBuffer[6] = C;
    TxUdpBuffer[7] = B00111011;           // ;

    // BROADCAST
    if(A=='b' && B=='r' && C=='o'){  // b r o
    // if(A==B01100010 && B==B01110010 && C==B01101111){  // b r o
      // direct
      // for (int i=0; i<15; i++){
        if(UdpCommand.remotePort()!=0){
        // if(IdSufix(NET_ID)==i ){
          UdpCommand.beginPacket(UdpCommand.remoteIP(), UdpCommand.remotePort());
            UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
          UdpCommand.endPacket();
          RemoteSwLatency[0] = millis(); // set START time mark UDP command latency
          RemoteSwLatencyAnsw = 0;   // send command, wait to answer

          #if defined(SERIAL_debug)
            if(DebugLevel>0){
              Serial.print(F("TX direct "));
              // Serial.print(i);
              // Serial.print(F(" "));
              Serial.print(UdpCommand.remoteIP());
              Serial.print(F(":"));
              Serial.print(UdpCommand.remotePort());
              Serial.print(F(" ["));
              Serial.print(TxUdpBuffer[0], HEX);
              for (int i=1; i<8; i++){
                Serial.print(char(TxUdpBuffer[i]));
                // Serial.print(F(" "));
              }
              Serial.println(F("]"));
            }
          #endif
        }
      // }

      // broadcast
      BroadcastIP = ~Ethernet.subnetMask() | Ethernet.gatewayIP();
      UdpCommand.beginPacket(BroadcastIP, BroadcastPort);   // Send to IP and port from recived UDP command
        UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
      UdpCommand.endPacket();
      IpTimeout[0][0] = millis();                      // set time mark
      RemoteSwLatency[0] = millis(); // set START time mark UDP command latency
      RemoteSwLatencyAnsw = 0;   // send command, wait to answer

        #if defined(SERIAL_debug)
          if(DebugLevel>0){
            Serial.print(F("TX broadcast "));
            Serial.print(BroadcastIP);
            Serial.print(F(":"));
            Serial.print(BroadcastPort);
            Serial.print(F(" ["));
            Serial.print(TxUdpBuffer[0], HEX);
            for (int i=1; i<8; i++){
              Serial.print(char(TxUdpBuffer[i]));
              // Serial.print(F(" "));
            }
            Serial.println(F("]"));
          }
        #endif

    // DATA
    }else{
      // if(DetectedRemoteSw[IdSufix(NET_ID)][4]!=0){
        UdpCommand.beginPacket(UdpCommand.remoteIP(), UdpCommand.remotePort());
          UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
        UdpCommand.endPacket();
        RemoteSwLatency[0] = millis(); // set START time mark UDP command latency
        RemoteSwLatencyAnsw = 0;   // send command, wait to answer

        #if defined(SERIAL_debug)
          if(DebugLevel>0){
            Serial.println();
            Serial.print(F("TX ["));
            Serial.print(TxUdpBuffer[0], HEX);
            for (int i=1; i<4; i++){
              Serial.print(char(TxUdpBuffer[i]));
            }
            Serial.print(TxUdpBuffer[4], BIN);
            Serial.print(F("|"));
            Serial.print(TxUdpBuffer[5], BIN);
            Serial.print(F("|"));
            Serial.print(TxUdpBuffer[6], BIN);
            Serial.print(char(TxUdpBuffer[7]));
            Serial.print(F("] "));
            // Serial.print(RemoteSwIP);
            Serial.print(F(":"));
            // Serial.println(RemoteSwPort);
          }
        #endif
      // }
    }
  InterruptON(1); // ptt, enc
  #endif
}
//-------------------------------------------------------------------------------------------------------

void EthernetCheck(){
  #if defined(EthModule)
    if(millis()-EthLinkStatusTimer[0]>EthLinkStatusTimer[1] && EnableEthernet==1){
      if ((Ethernet.linkStatus() == Unknown || Ethernet.linkStatus() == LinkOFF) && EthLinkStatus==1) {
        EthLinkStatus=0;
        #if defined(SERIAL_debug)
        // if(DEBUG==1){
          Serial.println(F("Ethernet DISCONNECTED"));
        // }
        #endif
      }else if (Ethernet.linkStatus() == LinkON && EthLinkStatus==0) {
        EthLinkStatus=1;
        #if defined(SERIAL_debug)
        // if(DEBUG==1){
          Serial.println(F("Ethernet CONNECTED"));
        // }
        #endif

        #if defined(LCD)
          lcd.clear();
          lcd.setCursor(1, 0);
          lcd.print(F("Net-ID: "));
          lcd.print(NET_ID, HEX);
          lcd.setCursor(1, 1);
          lcd.print(F("[DHCP-"));
        #endif
        if(EnableDHCP==1){
            #if defined(LCD)
              lcd.print(F("ON]..."));
            #endif
            Ethernet.begin(mac);
            IPAddress CheckIP = Ethernet.localIP();
            if( CheckIP[0]==0 && CheckIP[1]==0 && CheckIP[2]==0 && CheckIP[3]==0 ){
              #if defined(LCD)
                lcd.clear();
                lcd.setCursor(1, 0);
                lcd.print(F("DHCP FAIL"));
                lcd.setCursor(1, 1);
                lcd.print(F("please restart"));
              #endif
                Serial.println(F("DHCP FAIL - please restart"));
              while(1) {
                // infinite loop
              }
            }
        }else{
          #if defined(LCD)
            lcd.print(F("OFF]"));
          #endif
          // Ethernet.begin(mac, ip, myDns, gateway, subnet);     // Fixed IP
        }
        #if defined(LCD)
            delay(2000);
            lcd.clear();
            lcd.setCursor(1, 0);
            lcd.print(F("IP address:"));
            lcd.setCursor(1, 1);
            lcd.print(Ethernet.localIP());
            delay(2500);
            lcd.clear();
        #endif
        Serial.print(F("IP address: "));
        Serial.println(Ethernet.localIP());
        Serial.print(F("NET-ID: "));
        Serial.println(NET_ID, HEX);

        // server.begin();                     // Web
        UdpCommand.begin(UdpCommandPort);   // UDP
        TxUDP(ThisDevice, RemoteDevice, 'b', 'r', 'o');
        // NeedRxSettings=1;
      }
      EthLinkStatusTimer[0]=millis();
    }
  #endif
}
//---------------------------------------------------------------------------------------------------------
byte IdSufix(byte ID){
  bitClear(ID, 4);
  bitClear(ID, 5);
  bitClear(ID, 6);
  bitClear(ID, 7);
  return(ID);
}
//---------------------------------------------------------------------------------------------------------
