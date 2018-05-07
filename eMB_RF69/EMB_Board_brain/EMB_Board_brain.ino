// rf69 demo tx rx oled.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <SPI.h>
#include <Wire.h>
#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <USBSabertooth.h>
#include "RTClib.h"
#include <SD.h>

/************ OLED Setup ***************/
Adafruit_SSD1306 oled = Adafruit_SSD1306();

#define BUTTON_A 9
#define LED      13


/************ Radio Setup ***************/
// Where to send packets to!
#define DEST_ADDRESS   2
// change addresses for each client board, any number :)
#define MY_ADDRESS     1
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#define RFM69_CS      10   // "B"
#define RFM69_RST     11   // "A"
#define RFM69_IRQ     6    // "D"
#define RFM69_INT    digitalPinToInterrupt(RFM69_IRQ)

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "";
char radiopacket[RH_RF69_MAX_MESSAGE_LEN] = "time";

/************ RTC Setup ***************/
#define DS3231_I2C_ADDR 0x68
#define DS3231_TEMP_MSB 0x11
RTC_DS3231 rtc;

/************ SDCard Setup ***************/
#define SD_CHIP_SELECT 4
File logFile;

// Global Use data
String logName = "";
String dateNow = "";
String timeNow = "";
String theTemp = "";
boolean joystiq_connected = false;

int joystiq_info[5] = {
  0, // vertical
  0, // horizontal
  0, // j-button state
  0, // z-button state
  0  // c-button state
};

String logTitles[18] = {
  "Time",
  "Temperature",
  "Brightness",
  "Lights",
  "Throttle Raw",
  "Throttle %",
  "Z-Button",
  "C-Button",
  "Attopilot Voltage",
  "Attopilot Amperage",
  "ESC Battery Voltage",
  "ESC Motor Amperage",
  "ESC Motor Temperature",
  "ESC Motor Torque",
  "ESC Motor Current RPM",
  "ESC Motor Maximum RPM",
  "Wheel RPM",
  "Current Mph"
};

void setup() {
  
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  // Initialize OLED display
  showSplash(3000);

  // Preflight Checks
  preflightChecks();
  
  pinMode(LED, OUTPUT);
  
}

/*
   oledPrint Function
   updates OLED with info.
*/
void oledPrint(String err, int wait) {
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.print(err);
  oled.display();

  if (wait) {
    delay(wait);
  }
}

/*
   showSplash Function
   inits OLED and adds delay
   for hardware connections
*/
void showSplash(int holdup) {
  pinMode(BUTTON_A, INPUT_PULLUP);
  // Initialize OLED with I2C addr 0x32 (60)
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // OLED Init done.

  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
  oled.print(" {eMB v4} ");
  oled.println("-HighJinx-");
  oled.display();

  if (holdup) {
    delay(holdup);
  }

}

/*
   preflightChecks Function
   runs functions for missing
   hardware or not.
*/
void preflightChecks() {
  // check if RTC is present;
  checkRTC();
  
  // check if SD Card is connected;
  checkSDCard();

  // check if Nunchuk is connected;
  checkRadio();

}

void checkRadio() {
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    oledPrint("RFM69 radio init failed",500);
    while (1);
  }
  oledPrint("RFM69 radio init OK!",500);
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    oledPrint("setFrequency failed",500);
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x06, 0x00, 0x05, 0x01, 0x06, 0x00, 0x05,
                    0x01, 0x06, 0x00, 0x05, 0x01, 0x06, 0x00, 0x05};
  rf69.setEncryptionKey(key);
}

/*
   checkRTC Function
   checks RTC presence and
   intitalizes it.
*/
void checkRTC() {
  // Serial.println("Init the RTC");
  if (!rtc.begin()) {
    // Serial.println("Couldn't find RTC");
    oledPrint("RTC Module not found", 500);
    while (1);
  } else {
    // Serial.println("RTC Initialized");
    oledPrint("RTC Initialized.", 500);
  }

  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

}

/*
   checkSDCard Function
   checks for SD Card presence.
*/
void checkSDCard() {
  // Serial.println("Init the SD Card");
  if (!SD.begin(SD_CHIP_SELECT)) {
    // Serial.println("No SD Card Found");
    oledPrint("SD Card  is Missing", 500);
    // die
    while (1);
  } else {
    // Serial.println("Card Initalized");
    oledPrint("SD Card Connected. ", 500);
    setupLogFile();
  }
}

/*
   setupLogFile Function
   creates logfile for the
   current date.
*/
void setupLogFile() {
  String logDate = theDate();
  logName = logDate + ".csv";

  if (!SD.exists("/logs/")) {
    oledPrint("logs folder missing!", 500);
    oledPrint("creating logs folder", 500);
    if (SD.mkdir("/logs/")) {
      oledPrint("created logs folder", 500);
    } else {
      oledPrint("error making folder", 0);
      while (1);
    }
  }

  logFile = SD.open("/logs/" + logName, FILE_WRITE);
  if (logFile.size() == 0) {
    // newly created, add titles
    String title_str = "";
    int titles_len = 18;
    for(int i = 0; i < titles_len;i++) {
      title_str += logTitles[i];
      if (i != (titles_len - 1)) {
        title_str += ",";
      }
    }
    logFile.println(title_str);
    logFile.close();
  } else {
    // just close for now.
    logFile.close();
  }

  if (SD.exists("/logs/" + logName)) {
    oledPrint(logName + " present!", 500);
  } else {
    oledPrint(logName + " missing!", 0);
    while (1);
  }
}

void loop(){  
  
  dateNow = theDate();
  timeNow = theTime();
  theTemp = getTemp();
  
  createRadioPacket();

  if(!joystiq_connected) {
    oledPrint("Remote Not Found!", 0);
  }
    
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS)) {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;   
    if (rf69_manager.recvfromAckTimeout(buf, &len, 500, &from)) {
      killswitch(false);
      buf[len] = 0;
      Serial.print("Remote #");
      Serial.print(from); Serial.print(": ");
      Serial.println((char*)buf);

      decodeRemotePacket((char*)buf);

      oled.clearDisplay();
      oled.setCursor(0,0);
      oled.println(joystiq_info[0]);
      oled.print("RSSI: "); oled.print(rf69.lastRssi());
      oled.display(); 
      
    } else {
      killswitch(true);
    }
    
  } else {
    killswitch(true);
  }

}

void createRadioPacket() {
  String data_to_send = "";

  data_to_send =  timeNow;
  data_to_send += ",";
  data_to_send += dateNow;

  data_to_send.toCharArray(radiopacket, data_to_send.length() + 1);
}

void decodeRemotePacket(char* data) {
    String the_data(data);
    int ind1   = the_data.indexOf(",");
    int ind2   = the_data.indexOf(",", ind1+1);
    int ind3   = the_data.indexOf(",", ind2+1);
    int ind4   = the_data.indexOf(",", ind3+1);
    int ind5   = the_data.indexOf(",", ind4+1);

    String vert = the_data.substring(0, ind1);
    String horz = the_data.substring(ind1+1,ind2);
    String jbut = the_data.substring(ind2+1,ind3);
    String zbut = the_data.substring(ind3+1,ind4);
    String cbut = the_data.substring(ind4+1,ind5);

    joystiq_info[0] = vert.toInt();
    joystiq_info[1] = horz.toInt();
    joystiq_info[2] = jbut.toInt();
    joystiq_info[3] = zbut.toInt();
    joystiq_info[4] = cbut.toInt();
    
}

void killswitch(boolean engage) {
  if(engage) {
    Serial.println("Killswitch Engaged!");
    joystiq_connected = false;
    joystiq_info[0] = 0;
    joystiq_info[1] = 0;
    joystiq_info[2] = 0;
    joystiq_info[3] = 0;
    joystiq_info[4] = 0;
  } else {
    joystiq_connected = true;
  }
}

/*
   theTime Function
   checks RTC for time and returns
   time in HH:MM:SS am/pm format
*/
String theTime() {
  DateTime now = rtc.now();

  String nT = "";
  unsigned int hr = now.hour();
  unsigned int mn = now.minute();
  unsigned int sc = now.second();
  String tMn = "";
  String tSc = "";
  String ampm = "";

  if (hr >= 13) {
    hr = hr - 12;
    ampm = " pm";
  } else {
    ampm = " am";
  }

  if (hr == 0) {
    hr = 12;
  }

  if (mn < 10) {
    tMn = "0" + String(mn);
  } else {
    tMn = String(mn);
  }

  if (sc < 10) {
    tSc = "0" + String(sc);
  } else {
    tSc = String(sc);
  }

  nT += hr;
  nT += ':';
  nT += tMn;
  nT += ':';
  nT += tSc;
  nT += ampm;

  return nT;
}

/*
   theDate Function
   checks RTC for date and returns
   date in MM:DD:YY format
*/
String theDate() {
  DateTime now = rtc.now();
  String lD = "";
  unsigned int mn = now.month();
  unsigned int dy = now.day();
  unsigned int yr = now.year();

  yr = yr - 2000;

  lD += mn;
  lD += "-";
  lD += dy;
  lD += "-";
  lD += yr;

  return lD;
}

/*
    Function getTemp
    returns approximate temperature
    from DS3231 RTC Module in F.
*/
String getTemp() {
  short temp_msb;
  String temp;

  Wire.beginTransmission(DS3231_I2C_ADDR);
  Wire.write(DS3231_TEMP_MSB);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDR, 1);
  temp_msb = Wire.read();

  // for readings in C just use temp_msb
  double tmp = (temp_msb * 1.8) + 32;
  temp = String(tmp);
  return temp;
}

