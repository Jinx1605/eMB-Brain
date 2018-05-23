/*
   eMB GPS Board Brain v2



   Timer intterupt code from
   https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f

*/

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

//sample rate of the sine wave in Hertz, how many times per second the TC5_Handler() function gets called per second basically
uint32_t sampleRate = 10;

/************ GPS Setup ***************/
// what's the name of the hardware serial port?
#define GPSSerial Serial1
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

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

/********* Motor Temp Setup ***********/
const byte R_Motor_addr = 0x3B;
const byte L_Motor_addr = 0x3A;

MLX90632 R_Motor_temp;
MLX90632 L_Motor_temp;

boolean which_motor = false;

/************ OLED Setup ***************/
Adafruit_SSD1306 oled = Adafruit_SSD1306();

#define BUTTON_A 9
#define LED      13

/************ SDCard Setup ***************/
#define SD_CHIP_SELECT 4
File logFile;
boolean logFileCreated = false;

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

// Global Use data
String logName = "";
String dateNow = "";
String timeNow = "";
boolean joystiq_connected = false;

int joystiq_info[6] = {
  0, // vertical
  0, // horizontal
  0, // j-button state
  0, // z-button state
  0, // c-button state
  0  // battery voltage
};

float right_motor_info[6] = {
  0.0, // motor temp
  0.0, // voltage
  0.0, // amperage
  0.0, // max rpm
  0.0, // curr rpm
  0.0  // torque
};

float left_motor_info[6] = {
  0.0, // motor temp
  0.0, // voltage
  0.0, // amperage
  0.0, // max rpm
  0.0, // curr rpm
  0.0  // torque
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
short loopCount = 0;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(3400000);

  // Initialize OLED display
  showSplash(3000);
  // Initialize SD Card and Logfile
  setupSD();
  // Iniitalize RFM69 Radio Module
  setupRadio();
  // Initialize MLX90632 Temp Sensor
  setupMotorTempSensor();
  // Initialize GPS Module
  setupGPS();
  // Timer for GPS Inturrupt. DO NOT CHANGE!
  tcConfigure(sampleRate); //configure the timer to run at <sampleRate>Hertz
  tcStartCounter(); //starts the timer
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
   checkSDCard Function
   checks for SD Card presence.
*/
void setupSD() {
  // Serial.println("Init the SD Card");
  if (!SD.begin(SD_CHIP_SELECT)) {
    Serial.println("No SD Card Found");
    oledPrint("SD Card  is Missing", 500);
    // die
    while (1);
  } else {
    Serial.println("SD Card Initalized");
    oledPrint("SD Card Connected. ", 500);
  }
}

/*
   setupLogFile Function
   creates logfile for the
   current date.
*/
void setupLogFile() {
  String logDate = theDate();
  // String logDate = "EMB_DATA";
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
    for (int i = 0; i < titles_len; i++) {
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
    Serial.println(logName + " present!");
    oledPrint(logName + " present!", 0);
    logFileCreated = true;
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
    oledPrint("RFM69 radio init failed", 500);
    while (1);
  }
  oledPrint("RFM69 radio init OK!", 500);

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    oledPrint("setFrequency failed", 500);
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x06, 0x00, 0x05, 0x01, 0x06, 0x00, 0x05,
                    0x01, 0x06, 0x00, 0x05, 0x01, 0x06, 0x00, 0x05
                  };
  rf69.setEncryptionKey(key);

}

void setupMotorTempSensor() {
  MLX90632::status errorFlag; //Declare a variable called errorFlag that is of type 'status'

  //Now begin communication with all these settings
  R_Motor_temp.begin(R_Motor_addr, Wire, errorFlag);
  L_Motor_temp.begin(L_Motor_addr, Wire, errorFlag);

  //The errorFlag is set to one of a handful of different errors
  if (errorFlag == MLX90632::SENSOR_SUCCESS) {
    Serial.println("MLX90632 online!");
  } else {
    //Something went wrong
    if (errorFlag == MLX90632::SENSOR_ID_ERROR) {
      Serial.println("Sensor ID did not match the sensor address. Probably a wiring error.");
    } else if (errorFlag == MLX90632::SENSOR_I2C_ERROR) {
      Serial.println("Sensor did not respond to I2C properly. Check wiring.");
    } else if (errorFlag == MLX90632::SENSOR_TIMEOUT_ERROR) {
      Serial.println("Sensor failed to respond.");
    } else {
      Serial.println("Other Error");
    }
  }
}

void setupGPS() {
  // oledPrint("Setup GPS Module", 0);
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

  // oledPrint("Getting GPS Data...", 1000);
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

void loop() {
  //tcDisable(); //This function can be used anywhere if you need to stop/pause the timer
  //tcReset(); //This function should be called everytime you stop the timer

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA())) {   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    } else {
      parseGPS();
      timeNow = theTime();
      dateNow = theDate();
      if (!logFileCreated) {
        if (dateNow != "0" || dateNow != "00-00-0") {
          setupLogFile();
        }
      }
    }
  }

  loopCount++;
  
  if (loopCount % 2) {
    readRadio();
    if (!joystiq_connected) {
      oledPrint("Joystiiq Not Found!!", 0);
    } else {
      oledPrint(gps_data[0], 0);
    }
  } else if (loopCount % 5){
    sprintGPS();
    sprintJoystiiq();
  } else if (loopCount % 9) {
    readMotorTempSensor();
    sprintMotorTemps();
  }
  
  if (loopCount == 10 || loopCount == 20) {
    loopCount = 0;
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  if (millis() - timer > 100) {
    timer = millis(); // reset the timer
  }

}

void parseGPS() {
  int hr = GPS.hour;
  int mn = GPS.minute;
  unsigned int sc = GPS.seconds;
  unsigned int nDay = GPS.day;
  String tMn = "";
  String tSc = "";
  String ampm = "";
  String nMnt = "";
  String snDay = "";

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
  if (hr <= 9) {
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

  if (GPS.month < 10) {
    nMnt = "0" + String(GPS.month);
  } else {
    nMnt = String(GPS.month);
  }

  if (nDay < 10) {
    snDay = "0" + String(nDay);
  } else {
    snDay = String(nDay);
  }

  gps_data[1] = nMnt;
  gps_data[1] += "-";
  gps_data[1] += snDay;
  gps_data[1] += "-";
  gps_data[1] += GPS.year;

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

void sprintGPS() {
  Serial.print("\nTime: ");
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
    Serial.print("LocationDegrees: ");
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
  int ind2   = the_data.indexOf(",", ind1 + 1);
  int ind3   = the_data.indexOf(",", ind2 + 1);
  int ind4   = the_data.indexOf(",", ind3 + 1);
  int ind5   = the_data.indexOf(",", ind4 + 1);
  int ind6   = the_data.indexOf(",", ind5 + 1);

  String vert = the_data.substring(0, ind1);
  String horz = the_data.substring(ind1 + 1, ind2);
  String jbut = the_data.substring(ind2 + 1, ind3);
  String zbut = the_data.substring(ind3 + 1, ind4);
  String cbut = the_data.substring(ind4 + 1, ind5);
  String bvol = the_data.substring(ind5 + 1, ind6);

  joystiq_info[0] = vert.toInt();
  joystiq_info[1] = horz.toInt();
  joystiq_info[2] = jbut.toInt();
  joystiq_info[3] = zbut.toInt();
  joystiq_info[4] = cbut.toInt();
  joystiq_info[5] = bvol.toInt();

  if (!joystiq_info[2]) {
    joystiq_info[2] = 1;
  } else {
    joystiq_info[2] = 0;
  }

  if (!joystiq_info[3]) {
    joystiq_info[3] = 1;
  } else {
    joystiq_info[3] = 0;
  }

  if (!joystiq_info[4]) {
    joystiq_info[4] = 1;
  } else {
    joystiq_info[4] = 0;
  }

}

void killswitch(boolean engage) {
  if (engage) {
    Serial.println("Killswitch Engaged!");
    joystiq_connected = false;
    joystiq_info[0] = 0;
    joystiq_info[1] = 0;
    joystiq_info[2] = 0;
    joystiq_info[3] = 0;
    joystiq_info[4] = 0;
    joystiq_info[5] = 0;
  } else {
    joystiq_connected = true;
  }
}

void readRadio() {
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS)) {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rf69_manager.recvfromAckTimeout(buf, &len, 500, &from)) {
      killswitch(false);
      buf[len] = 0;
      //      Serial.print("Remote #");
      //      Serial.print(from); Serial.print(": ");
      //      Serial.println((char*)buf);

      decodeRemotePacket((char*)buf);
    }
  } else {
    if (joystiq_connected) {
      killswitch(true);
    }
  }
}

void readMotorTempSensor() {
  if (!which_motor) {
    float rOTemp = R_Motor_temp.getObjectTempF();
    right_motor_info[0] = rOTemp;
    which_motor = true;
  } else {
    float lOTemp = L_Motor_temp.getObjectTempF();
    left_motor_info[0] = lOTemp;
    which_motor = false;
  }  
}

void sprintMotorTemps() {
  Serial.print("\nRight Motor Temp: ");
  Serial.print(right_motor_info[0]);
  Serial.print("\nLight Motor Temp: ");
  Serial.print(left_motor_info[0]);
}

void sprintJoystiiq() {
  Serial.print("\nVert: ");
  Serial.println(joystiq_info[0]);
  Serial.print("Horz: ");
  Serial.println(joystiq_info[1]);
  Serial.print("j-button: "); Serial.println(joystiq_info[2]);
  Serial.print("z-button: "); Serial.println(joystiq_info[3]);
  Serial.print("c-button: "); Serial.println(joystiq_info[4]);
  Serial.print("battery : "); Serial.println(joystiq_info[5]);
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

//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler (void) {
  //YOUR CODE HERE

  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif

  // END OF YOUR CODE
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
}

/*
    TIMER SPECIFIC FUNCTIONS FOLLOW
    you shouldn't change these unless you know what you're doing
*/

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
void tcConfigure(int sampleRate) {
  // Enable GCLK for TCC2 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY);

  tcReset(); //reset TC5

  // Set Timer counter Mode to 16 bits
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  // Set TC5 mode as match frequency
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  //set prescaler and enable TC5
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
  //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
  TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
  while (tcIsSyncing());

  // Configure interrupt request
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable the TC5 interrupt request
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  while (tcIsSyncing()); //wait until TC5 is done syncing
}

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing() {
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter() {
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5
void tcReset() {
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable() {
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}
