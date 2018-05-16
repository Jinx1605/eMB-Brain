// Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
//
// This code shows how to listen to the GPS module via polling. Best used with
// Feathers or Flora where you have hardware Serial and no interrupt
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
// 
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

     
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <USBSabertooth.h>
#include <SD.h>
#include "SparkFun_MLX90632_Arduino_Library.h"

/********* Motor Temp Setup ***********/
const byte R_Motor_addr = 0x3B;
const byte L_Motor_addr = 0x3A;

MLX90632 R_Motor_temp;
MLX90632 L_Motor_temp;

/***** Add Serial2 @  Rx/A2 Tx/A1 *****/
// Uart Serial2 (&sercom4, A2, A1, SERCOM_RX_PAD_1, UART_TX_PAD_0);

/************ GPS Setup ***************/
// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
     
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

const int timezone_hr_offset = -4; //Timezone hour offset
const int timezone_mn_offset = 0; //Timezone minute offset

String gps_data[14] = {
  "0", // time
  "0",  // date
  "0", // Fix
  "0", // Quality
  "0", // lattitude
  "0", // longitude
  "0", // latitudeDegrees
  "0", // longitudeDegrees
  "0", // Speed (knots)
  "0", // Speed (Mph)
  "0", // Speed (Kph)
  "0", // Angle
  "0", // Altitude
  "0", // Sattelites
};

boolean preflightChecks[8] = {
  false, // OLED 
  false, // GPS 
  false, // TIME
  false, // SD
  false, // LOGFILE
  false, // RADIO
  false, // R MOTOR TEMP
  false // L MOTOR TEMP
};

/************ OLED Setup ***************/
Adafruit_SSD1306 oled = Adafruit_SSD1306();

#define BUTTON_A 9
#define LED      13

/************ SDCard Setup ***************/
#define SD_CHIP_SELECT 4
File logFile;

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

/************ ESC Setup ***************/
#define ST_ESC_BAUDRATE 19200
//USBSabertoothSerial C;
//USBSabertooth ST(C, 128);


// Global Use data
String logName = "";
String dateNow = "";
String timeNow = "";
boolean joystiq_connected = false;

int joystiq_info[5] = {
  0, // vertical
  0, // horizontal
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

uint32_t timer = millis();

//void SERCOM4_Handler() {
//  Serial2.IrqHandler();
//}

void setup(){
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Wire.begin();

//  // Assign pins 10 & 11 SERCOM functionality
//  pinPeripheral(A1, PIO_SERCOM_ALT);
//  pinPeripheral(A2, PIO_SERCOM_ALT);

  // Onboard LED
  pinMode(LED, OUTPUT);

  // Initialize OLED display
  showSplash(3000);
  
  setupGPS();
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
  preflightChecks[0] = true;
  
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

void setupGPS() {
  oledPrint("Setup GPS Module", 0);
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  oledPrint("Getting GPS Data...", 1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
  
}

void parseGPS() {
  int hr = GPS.hour;
  int mn = GPS.minute;
  unsigned int sc = GPS.seconds;
  unsigned int nDay = GPS.day;
  String tMn = "";
  String tSc = "";
  String ampm = "";

  mn = mn + timezone_mn_offset; //Add The Minute Time Zone Offset
  if (mn > 59) { //If The Minute Is Over 59 (From The Time Zone Conversion)
    mn = mn - 60; //Subtract 60 From It
    hr = hr + 1; //Then Add 1 To The Hour
  } else {
    if (mn < 0) { //If Minute Is Less Than 0, Do The Inverse Of Above
      mn = mn + 60;
      hr = hr - 1;
    }
  }

  hr = hr + timezone_hr_offset; //Add The Hour Time Zone Offset
  if (hr > 23) {
    hr = hr - 24;
  } else {
    if (hr < 0) {
      hr = hr + 24;
      nDay = nDay - 1;
    }
  }

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

  gps_data[0] = "";
  if(hr <= 9) {
    gps_data[0] += "0";
    gps_data[0] += hr;
  } else {
    gps_data[0] += hr;
  }
  gps_data[0] += ':';
  gps_data[0] += tMn;
  gps_data[0] += ':';
  gps_data[0] += tSc;
  gps_data[0] += ampm;

  gps_data[1] = GPS.month;
  gps_data[1] += "-";
  gps_data[1] += nDay;
  gps_data[1] += "-";
  gps_data[1] += GPS.year;

  if(!GPS.month && !GPS.day && !GPS.year){
    preflightChecks[2] = false; 
  } else {
    preflightChecks[2] = true;
  }

  gps_data[2] = String(GPS.fix);
  gps_data[3] = String(GPS.fixquality);

  if (GPS.fix) {
    gps_data[4] = String(GPS.latitude, 4);
    gps_data[4] += GPS.lat;

    gps_data[5] = String(GPS.longitude, 4);
    gps_data[5] += GPS.lon;

    gps_data[6] = String(GPS.latitudeDegrees, 4);
    gps_data[7] = String(GPS.longitudeDegrees, 4);

    gps_data[8] = String(GPS.speed, 4); // knots
    gps_data[9] = String(GPS.speed * 1.15078, 4); // Mph
    gps_data[10] = String(GPS.speed * 1.852, 4); // Kph

    gps_data[11] = GPS.angle;
    gps_data[12] = GPS.altitude;
    gps_data[13] = GPS.satellites;
  }
}

/*
   checkSDCard Function
   checks for SD Card presence.
*/
void setupSD() {
  // Serial.println("Init the SD Card");
  if (!SD.begin(SD_CHIP_SELECT)) {
    Serial.println("No SD Card Found");
    oledPrint("SD Card  is Missing", 500);
    // die
    while(1);
  } else {
    Serial.println("Card Initalized");
    oledPrint("SD Card Connected. ", 500);
    preflightChecks[3] = true;
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
    preflightChecks[4] = true;
  } else {
    oledPrint(logName + " missing!", 0);
    while (1);
  }
}

void setupRadio() {
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
  
  preflightChecks[5] = true;
}

void loop() {
  
  readGPS();
  
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
     
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
    sprintGPS();
    pfChecks();
  }
  
}

void pfChecks() {

//boolean preflightChecks[8] = {
//  false, // OLED 
//  false, // GPS 
//  false, // TIME
//  false, // SD
//  false, // LOGFILE
//  false, // RADIO
//  false, // R MOTOR TEMP
//  false // L MOTOR TEMP
//};
  
  // Setup SD
  if (preflightChecks[2] && !preflightChecks[3]) {
    setupSD();
  }

  // Setup Datalogger
  if (preflightChecks[3] && !preflightChecks[4]) {
    setupLogFile();
  }

  // Setup Radio
  if (preflightChecks[2] && preflightChecks[3] && preflightChecks[4]) {
    if (!preflightChecks[5]) {
      setupRadio();
    }
    
  }

  // Setup motor temps.
  if (preflightChecks[2] && preflightChecks[3] && preflightChecks[4] && preflightChecks[5]) { 
    oledPrint(gps_data[0], 0);
  }
  
}

void readGPS(){
    // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO) {
    if (c) Serial.print(c);
  } 
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) { // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
    } else {
      parseGPS();
      preflightChecks[1] = true;
    }
  }
}

void sprintGPS() {
  Serial.print("\nTime: ");
  Serial.print(GPS.hour, DEC); Serial.print(" : ");
  Serial.println(gps_data[0]);
  Serial.print("Date: ");
  Serial.println(gps_data[1]);
  Serial.print("Fix: "); Serial.print(gps_data[2]);
  Serial.print(" quality: "); Serial.println(gps_data[3]);
  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(gps_data[4]);
    Serial.print(", ");
    Serial.println(gps_data[5]);
    Serial.print(gps_data[6]);
    Serial.print(", ");
    Serial.println(gps_data[7]);
    Serial.print("Speed (knots): "); Serial.println(gps_data[8]);
    Serial.print("Speed (mph): "); Serial.println(gps_data[9]);
    Serial.print("Speed (kmh): "); Serial.println(gps_data[10]);
    Serial.print("Angle: "); Serial.println(gps_data[11]);
    Serial.print("Altitude: "); Serial.println(gps_data[12]);
    Serial.print("Satellites: "); Serial.println(gps_data[13]);
  }
}

/*
   theTime Function
   checks RTC for time and returns
   time in HH:MM:SS am/pm format
*/
String theTime() {
  return gps_data[0];
}

/*
   theDate Function
   checks RTC for date and returns
   date in MM:DD:YY format
*/
String theDate() {
  return gps_data[1];
}

