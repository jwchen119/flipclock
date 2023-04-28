/*
Connection of Arduino AVR328P and TPIC6B595 and the next TIPC

 Nano || TPIC6B595
   5V <> 2 (VCC)
  GND <> GND
   D8 <> 8 (SRCLR)
  D10 <> 12 (RCK)
  D11 <> 3 (SER IN) of the first chip
  D13 <> 13 (SRCK)
      || 9 to GND
      || 10, 11,19 to GND
      || 18 (SER OUT) to 3 of the next chip

  Put 0.1 uF cap between VCC and GND of TIPC6B595

       ESP32 || TPIC6B595
         5V  <> 2 (VCC)
         GND <> GND
         D7  <> 8 (SRCLR)
 HSPI or VSPI
  D15 or D5  <> 12 (RCK)
  D13 or D23 <> 3 (SER IN) of the first chip
  D14 or D18 <> 13 (SRCK)
             || 9 to GND
             || 10, 11,19 to GND
             || 18 (SER OUT) to 3 of the next chip
*/


#include "Arduino.h"
#if ESP8266
#include "ESPAsyncTCP.h"
#include "ESP8266WiFi.h"
#elif ESP32
#include "WiFi.h"
// #include "AsyncTCP.h"
#endif
#include "SPIFFS.h"
// #include <ESPAsyncWebServer.h>
// #include <AsyncElegantOTA.h>
// #include "RTClib.h"
#include "time.h"
// #include <DNSServer.h>
#include <SPI.h>
#include <RTClib.h>
#include <Arduino_JSON.h>

// #define CONFIG_ASYNC_TCP_RUNNING_CORE 0
// #define CONFIG_ASYNC_TCP_USE_WDT 0

SPIClass hSPI(HSPI);
const long  gmtOffset_sec = 28800; //gmt+8(utc-28800)
const int   daylightOffset_sec = 0;
const char* ntpServer = "pool.ntp.org";
RTC_DS3231 myRTC;
#define DEBUG
#define motors 32      // num of motors
const byte clrPin = 7; // TPIC6B595 SRCLR pin
const byte oePin = 9;
const byte latchPin = 15; // TPIC6B595 RCK pin
int nowPos[motors], newPos[motors];
unsigned long prevMicros; // timing
byte val[4];              // TPIC write bytes
unsigned long prevMillis, ntpUpdatemillis, shutAPmillis, temp_prevMillis, wifiSettingTimeout;
unsigned long checkMillis, pMillis;
int hours, minutes, seconds, hourOnes, hourTens, minuteOnes, minuteTens, secondTens, secondOnes;
int speedControl = 2440;
// int speedControl = 4880;
int timeSeq[4], nTime[4][9], newTime[4], oldTime[4];
int quarterTurnSteps = 512;
int checkTimeInterval = 1000; // millisecond
bool firstRun = true;
bool tempTrigger = false;
bool effector = false;
void effectorSub(int[], byte);
void goHomePos();
void tickTak();
void checkTime();
struct tm timeinfo;
char timeYear[5];
char timeMonth[3];
char timeDay[3];
char timeHour[3];
char timeMinute[3];
char timeSecond[3];
bool ESPAPDisconnect = false;
bool showTemp = false;
int errorCode = 0;
int timeoutCount = 10;
bool wifi_changing = false;
bool checksteps = 0;

int digits[15][8] = {
    {0, 0, 1, 1, 1, 1, 1, 1}, // 0
    {0, 0, 1, 1, 0, 0, 0, 0}, // 1
    {0, 1, 1, 0, 1, 1, 0, 1}, // 2
    {0, 1, 1, 1, 1, 0, 0, 1}, // 3
    {0, 1, 1, 1, 0, 0, 1, 0}, // 4
    {0, 1, 0, 1, 1, 0, 1, 1}, // 5
    {0, 1, 0, 1, 1, 1, 1, 1}, // 6
    {0, 0, 1, 1, 0, 0, 1, 1}, // 7
    {0, 1, 1, 1, 1, 1, 1, 1}, // 8
    {0, 1, 1, 1, 1, 0, 1, 1}, // 9
    {0, 0, 0, 0, 1, 1, 1, 1}, // C
    {0, 1, 1, 0, 0, 0, 1, 1}, // degree
    {0, 0, 0, 0, 0, 0, 0, 1}, // -
    {0, 0, 0, 0, 0, 0, 0, 0}, // full open
    {0, 1, 1, 1, 1, 0, 0, 1}  // Error
};

// int digits[12][7] = {
//     {0, 1, 1, 1, 1, 1, 1}, // 0
//     {0, 1, 1, 0, 0, 0, 0}, // 1
//     {1, 1, 0, 1, 1, 0, 1}, // 2
//     {1, 1, 1, 1, 0, 0, 1}, // 3
//     {1, 1, 1, 0, 0, 1, 0}, // 4
//     {1, 0, 1, 1, 0, 1, 1}, // 5
//     {1, 0, 1, 1, 1, 1, 1}, // 6
//     {0, 1, 1, 0, 0, 1, 1}, // 7
//     {1, 1, 1, 1, 1, 1, 1}, // 8
//     {1, 1, 1, 1, 0, 1, 1}, // 9
//     {0, 0, 0, 0, 0, 0, 0},  // full open
//     {1, 1, 1, 1, 0, 0, 1}  // Error
// };

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif
  if (!myRTC.begin()) {
    errorCode = 1;
    Serial.println("no RTC");
  }
  hSPI.begin();
  hSPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
  pinMode(latchPin, OUTPUT);

  if (myRTC.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    myRTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  myRTC.disable32K();
  myRTC.disableAlarm(1);
  myRTC.disableAlarm(2);
  myRTC.writeSqwPinMode(DS3231_OFF);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  if(!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  Serial.println("go home");
  goHomePos();
  Serial.println("home done");
  delay(100);
}


void loop() {
 
  if (firstRun) {
    Serial.println("firstRun");
    for (byte i = 0; i < motors; i++) {
      newPos[i] = quarterTurnSteps;
    }
    for (int j = 0; j < quarterTurnSteps + 1; j++) {
      tickTak();
    }
    // checkTime();
    newTime[0] = 9;
    newTime[1] = 9;
    newTime[2] = 9;
    newTime[3] = 9;

    for (byte j = 0; j < 4; j++) {
      for (byte i = 0; i < 8; i++) {
        if (digits[newTime[j]][i] - digits[timeSeq[j]][i] == -1) {
          nTime[j][i] = 0;
        } else if (digits[newTime[j]][i] - digits[timeSeq[j]][i] == 1) {
          nTime[j][i] = 1;
        } else {
          nTime[j][i] = digits[timeSeq[j]][i];
        }
        // newPos[0 + i] = nTime[j][i] * quarterTurnSteps;
        // newPos[7 + i] = nTime[j][i] * quarterTurnSteps;
        // newPos[14 + i] = nTime[j][i] * quarterTurnSteps;
        // newPos[21 + i] = nTime[j][i] * quarterTurnSteps;
      }
    }
    for (byte j = 0; j < 8; j++) {
      newPos[0 + j] = nTime[0][j] * quarterTurnSteps;
      newPos[8 + j] = nTime[1][j] * quarterTurnSteps;
      newPos[16 + j] = nTime[2][j] * quarterTurnSteps;
      newPos[24 + j] = nTime[3][j] * quarterTurnSteps;
    }

    for (int j = 0; j < quarterTurnSteps + 1; j++) {
      tickTak();
    }

    firstRun = false;
    prevMillis = millis();
    ntpUpdatemillis = millis();
    shutAPmillis = millis();
  }

  if (millis() - prevMillis > checkTimeInterval) {
     Serial.println("loop");
    //  checksteps = 1;
    prevMillis = millis();
    // temp_prevMillis = millis();
    // DateTime now = myRTC.now();
    
    // Serial.println(ESP.getFreeHeap());
    
    oldTime[0] = hourTens;
    oldTime[1] = hourOnes;
    oldTime[2] = minuteTens;
    oldTime[3] = minuteOnes;
    // checkTime();

    // hours = now.hour();
    // minutes = now.minute();
    // seconds = now.second();
    // hourTens = hours / 10;
    // hourOnes = hours % 10;
    // minuteTens = minutes / 10;
    // minuteOnes = minutes % 10;
    // secondTens = seconds / 10;
    // secondOnes = seconds % 10;

    newTime[0] = 4;
    newTime[1] = 3;
    newTime[2] = 2;
    newTime[3] = 1;

    for (byte j = 0; j < 4; j++) {
      for (byte i = 0; i < 8; i++) {
        if (digits[newTime[j]][i] - digits[timeSeq[j]][i] == -1) {
          nTime[j][i] = 0;
        } else if (digits[newTime[j]][i] - digits[timeSeq[j]][i] == 1) {
          nTime[j][i] = 1;
        } else {
          nTime[j][i] = digits[timeSeq[j]][i];
        }
        newPos[0 + i] = nTime[j][i] * quarterTurnSteps;
        newPos[8 + i] = nTime[j][i] * quarterTurnSteps;
        newPos[16 + i] = nTime[j][i] * quarterTurnSteps;
        newPos[24 + i] = nTime[j][i] * quarterTurnSteps;
      }
    }
    // for (byte j = 0; j < 8; j++) {
    //   newPos[0 + j] = nTime[0][j] * quarterTurnSteps;
    //   newPos[8 + j] = nTime[1][j] * quarterTurnSteps;
    //   newPos[16 + j] = nTime[2][j] * quarterTurnSteps;
    //   newPos[24 + j] = nTime[3][j] * quarterTurnSteps;
    // }
  }

  // if (millis() - testmillis > 120000 && wifiDisconnect == true) {
  //   Serial.println("reconnect");
  //   // ws.textAll(getStatus(0));
  //   WiFi.disconnect();
  //   WiFi.reconnect();
  //   testmillis = millis();
  //   wifiDisconnect = false;
  // }
  tickTak(); // let's step
}

void tickTak() {
  while (micros() - prevMicros < speedControl); // minimum step interval, 12RPM motor limit
  prevMicros = micros(); // update, for next interval
  for (int i = 0; i < motors; i++) {
    if (newPos[i] > nowPos[i]) {              // forwards
      nowPos[i]++; // one step forward
    }
    if (newPos[i] < nowPos[i]) {              // reverse
      nowPos[i]--; // one step reverse
    }
    if (newPos[i] == nowPos[i]) {
      val[i & 3] = B00000000;
    }

    else {
      switch (nowPos[i] & 3) {
      // this block for torque
      // case 0: val[i & 3] = B00000011; break;
      // case 1: val[i & 3] = B00001001; break;
      // case 2: val[i & 3] = B00001100; break;
      // case 3: val[i & 3] = B00000110; break;
      // or this block for low power
      case 0:
        val[i & 3] = B00000001;
        break;
      case 1:
        val[i & 3] = B00001000;
        break;
      case 2:
        val[i & 3] = B00000100;
        break;
      case 3:
        val[i & 3] = B00000010;
        break;
      }
    }
    if ((i & 3) == 3) // process set of four motors (faster)
    {
      unsigned int chain = val[0] << 12 | val[1] << 8 | val[2] << 4 | val[3]; // concat
#ifdef DEBUG
  if (checksteps == 1) {
    Serial.println(chain);
  } else {}
      
#endif
      hSPI.transfer16(chain); // transfer 4-motor int
    }
  }
  digitalWrite(latchPin, LOW); // transfer to TPIC driver outputs
  digitalWrite(latchPin, HIGH);  // latch time ~8us
}

void goHomePos() {
  for (byte i = 0; i < motors; i++) {
    newPos[i] = -550; // make sure all motor back to home position
  }
  // for (int j = 0; j < quarterTurnSteps + 1; j++) { // tick all motors to home position
  for (int j = 0; j < 550; j++) { // tick all motors to home position
    tickTak();
  }
  for (byte i = 0; i < motors; i++) {
    newPos[i] = 0;
    nowPos[i] = 0; // set nowpos as homepos
  }
}

void effectorSub(int effectorBool[], byte n) {
  byte posMultiply = 0;
  for (byte i = 0; i < n; i++) {
    for (byte j = 0; j < 8; j++) {
      if (effectorBool[i] == 1) {
        newPos[posMultiply + j] = 0;
      }
    }
    posMultiply += 8;
  }
  for (int j = 0; j < quarterTurnSteps + 1; j++) {
    tickTak();
  }
  for (byte i = 0; i < n; i++) {
    for (byte j = 0; j < 8; j++) {
      if (effectorBool[i] == 1) {
        newPos[posMultiply + j] = 0;
        nowPos[posMultiply + j] = 0;
      }
    }
    posMultiply += 8;
  }
}
