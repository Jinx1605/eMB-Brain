/*
   Electric Mountianboard Brain v4
   Parts Used:
   - Adafruit M0 Adalogger Feather / Adafruit WICED Feather
   - Adafruit DS3231 RTC Featherwing / Adalogger Featherwing
   - Adafruit OLED Featherwing
   - Solarbotics Nunchucky Adaptor
   - Sparkfun Bi-directional LLC
   - Dimensionengineering Sabertooth 2x32 ESC
   - Adafruit NeoPixels
   - AttoPilot Voltage Breakout
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoNunchuk.h>
#include <Adafruit_NeoPixel.h>
#include <USBSabertooth.h>
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>

#ifdef __AVR__
#include <avr/power.h>
#endif

/*
   NeoPixel pin settings
*/
#ifdef _VARIANT_ARDUINO_STM32_
#define NEOPIXEL_FRNT PB5
#define NEOPIXEL_BACK PB4
#else
#define NEOPIXEL_FRNT 13
#define NEOPIXEL_BACK 10
#endif

/*
   Light Detector Settings 
   LDR w/ 10k Ohm Resistor
*/
#ifdef _VARIANT_ARDUINO_STM32_
#define LDR_PIN PA2
#else
#define LDR_PIN A4
#endif
#define LDR_HIGH 700 // upper limit/on
#define LDR_LOW 600  // lower limit/off

/*
   Attopilot pin settings
*/
#ifdef _VARIANT_ARDUINO_STM32_
#define ATTOPILOT_V PA0
#define ATTOPILOT_I PC3
#else
#define ATTOPILOT_V A0
#define ATTOPILOT_I A1
#endif

/*
   Horn pin settings
*/
#ifdef _VARIANT_ARDUINO_STM32_
#define HORN_PIN PA1
#else
#define HORN_PIN A5
#endif

/*
   RTC Info.
*/
#define DS3231_I2C_ADDR 0x68
#define DS3231_TEMP_MSB 0x11

#define OLED_RESET 3

#define ST_ESC_BAUDRATE 19200

/*
   SD Card pin settings
*/
#ifdef _VARIANT_ARDUINO_STM32_
#define SD_CHIP_SELECT PB4
#else
#define SD_CHIP_SELECT 4
#endif

/*
   OLED Buttons
*/
#ifdef _VARIANT_ARDUINO_STM32_
#define OLED_BTN_A PA15
#define OLED_BTN_B PC7
#define OLED_BTN_C PC5
#else
#define OLED_BTN_A 9
#define OLED_BTN_B 6
#define OLED_BTN_C 5
#endif

#define NUNCHUK_I2C_ADDRESS 82
#define NUNCHUK_TYPE "wireless" // "wireless" or "wired"

#define FPM_2_MPH 0.0114
#define WHEEL_CIRCUMFERENCE 2.0125

/*
   MOTOR_RPM_2_VOLT
   ampflow A23-150 = 269
   ampflow A28-150 = 257
   ampflow F30-150 = 287
   ampflow E30-150 = 237
*/
#define MOTOR_RPM_2_VOLT 269

/*
   MOTOR_OZIN_2_AMP (torque)
   ampflow A23-150 = 5.03
   ampflow A28-150 = 5.26
   ampflow F30-150 = 4.66
   ampflow E30-150 = 5.70
*/
#define MOTOR_OZIN_2_AMP 5.03

#define MOTOR_TOOTH_COUNT 12
#define WHEEL_TOOTH_COUNT 64

Adafruit_SSD1306 display(OLED_RESET);

Adafruit_NeoPixel frnt_lights = Adafruit_NeoPixel(7, NEOPIXEL_FRNT, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel rear_lights = Adafruit_NeoPixel(7, NEOPIXEL_BACK, NEO_GRBW + NEO_KHZ800);

USBSabertoothSerial C;
USBSabertooth ST(C, 128);

#ifdef _VARIANT_ARDUINO_STM32_
RTC_PCF8523 rtc;
#else
RTC_DS3231 rtc;
#endif

// Global Use data
String logName = "";
String dateNow = "";
String timeNow = "";
String theTemp = "";
String batteryVoltage = "";

// Attopilot Readings
int apVRaw; // raw voltage
int apARaw; // raw amperage
float apVFinal; // converted voltage
float apAFinal; // converted amperage

// More Global Use data
float throttle = 0;
float throttlePercentage = 0;
boolean killSwitch = true;

// LDR Detection states
boolean isDayTime = false;
boolean isLightsOn = false;
int LDRReading = 0;

// OLED Button States
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button


float nunchukInfo[4] = {
  0.0, // throttle
  0.0, // throttle %
  0.0, // z-button state
  0.0  // c-button state
};

float motor1[6] = {
  0.0, // voltage
  0.0, // amperage
  0.0, // temp
  0.0, // max rpm
  0.0, // curr rpm
  0.0  // torque
};

float motor2[6] = {
  0.0, // voltage
  0.0, // amperage
  0.0, // temp
  0.0, // max rpm
  0.0, // curr rpm
  0.0  // torque
};

float reductionRatio;
float wheelRPM;
float wheelFPM;
float currentMPH;

short loopCount = 0;

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

ArduinoNunchuk nunchuk = ArduinoNunchuk();

File logFile;

void setup() {

  // Calculate now, only need it once.
  reductionRatio = WHEEL_TOOTH_COUNT / MOTOR_TOOTH_COUNT;

  // for the horn :P
  // pinMode(HORN_PIN, OUTPUT);

  showSplash(3000);

  // Serial.begin(19200);
  // while (!Serial) {}

  // Preflight Checks
  preflightChecks();

  // setup lights
  setupLights();

}

/*
   oledPrint Function
   updates OLED with info.
*/
void oledPrint(String err, int wait) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(err);
  display.display();

  if (wait) {
    delay(wait);
  }
}
/*
   oledUpdate Function
   shows data after all
   checks are good and
   everything is peachy
*/
void oledUpdate() {
  String dateTime = "";
  String disp = "";
  dateTime = dateNow + "  " + timeNow;
  if (dateTime.length() != 21) {
    int strDiff = 21 - dateTime.length();
    for (int i=0; i < strDiff;i++) {
      dateTime += " ";
    }
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(dateTime);
  display.setTextSize(2);
  if (buttonPushCounter % 4 == 0) {
    disp += String(currentMPH);
    disp += " Mph";
  }

  if (buttonPushCounter % 4 == 1) {
    disp += String(motor1[0]);
    disp += "v";
  }

  if (buttonPushCounter % 4 == 2) {
    disp += String(apAFinal);
    disp += "A";
  }

  if (buttonPushCounter % 4 == 3) {
    disp += String(motor1[4]);
    disp += " RPM";
  }
  
  display.print(disp);
  display.display();
}

/*
   showSplash Function
   inits OLED and adds delay
   for hardware connections
*/
void showSplash(int holdup) {
  pinMode(OLED_BTN_A, INPUT_PULLUP);
  // Initialize OLED with I2C addr 0x32 (60)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // OLED Init done.

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(" {eMB v4} ");
  display.println("-HighJinx-");
  display.display();

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
  // check if SD Card is connected;
  checkSDCard();

  // check if RTC is present;
  checkRTC();

  // check if Nunchuk is connected;
  checkNunchuk(true);

  // setupESC
  if (setupESC(0)) {
    oledPrint("EverythingGood To Go", 500);
  } else {
    oledPrint("Sabertooth ESC Missing", 500);
  }
}

/*
   setupLights Function
   sets up front and rear
   NeoPixel lights.
*/
void setupLights() {
  frnt_lights.begin();
  rear_lights.begin();
  frnt_lights.show();
  rear_lights.show();
}

void lightsOn() {
  frontLights(frnt_lights.Color(0,0,0,255), 255);
  rearLights(rear_lights.Color(255,0,0,0), 127);
}

void lightsOff() {
  frontLights(frnt_lights.Color(0,0,0,0), 0);
  rearLights(rear_lights.Color(0,0,0,0), 0);
}

// Fill the dots one after the other with a color
void frontLights(uint32_t color, uint16_t brightness) {
  frnt_lights.setBrightness(brightness);
  for (uint16_t i=0; i < frnt_lights.numPixels(); i++) {
    frnt_lights.setPixelColor(i, color);
    frnt_lights.show();
  }
}

void rearLights(uint32_t color, uint16_t brightness) {
  rear_lights.setBrightness(brightness);
  for (uint16_t i=0; i < rear_lights.numPixels(); i++) {
    rear_lights.setPixelColor(i, color);
    rear_lights.show();
  }
}

/*
   checkNunchuk Function
   checks for Wii Nunchuk presence.
*/
boolean checkNunchuk(boolean showConn) {
  // check for nunchuk
  Wire.begin();
  Wire.beginTransmission(NUNCHUK_I2C_ADDRESS);
  if (Wire.endTransmission() == 0) {
    if (showConn) {
    // Serial.println("Wiichuk connected");
      oledPrint("WiiChuk Connected.", 500);
    }
    // Init the Nunchuk
    nunchuk.init();
  } else {
    // Serial.println("Wiichuk not found");
    oledPrint(" WiiChuk  not found.", 0);
    delay(5000);
    checkNunchuk(true);
    return false;
  }
  return true;
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

#ifdef _VARIANT_ARDUINO_STM32_
  if (!rtc.initialized()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
#else
   if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
   }
#endif

}

/*
   setupESC Function
   inits Sabertooth ESC and
   can set freewheeling mode
   (disable/enable regenerative braking)
   
   NOTE: Still need to figure out
         how to check if ESC is
         actually present or not.
*/
boolean setupESC(int FreeWheel) {
  oledPrint("Checking Sabertooth ESC", 500);
  // Setup Sabertooth ESC
  SabertoothTXPinSerial.begin(ST_ESC_BAUDRATE);
  // ST.autobaud();
  if (!FreeWheel) {
    ST.freewheel(1, false);
    ST.freewheel(2, false);
  } else {
    ST.freewheel(1, true);
    ST.freewheel(2, true);
  }
  return true;
}

void loop() {

  // read the pushbutton input pin:
  buttonState = digitalRead(OLED_BTN_A);
 
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == LOW) {
      // if the current state is LOW then the button was pressed
      buttonPushCounter++;
    }
  }
  
  // save the current state as the last state,
  //for next time through the loop
  lastButtonState = buttonState;

  throttle = readChuk(NUNCHUK_TYPE);
  throttlePercentage = map(throttle, 0, 2047, 0, 100);
  nunchukInfo[0] = throttle;
  nunchukInfo[1] = throttlePercentage;

  dateNow = theDate();
  timeNow = theTime();

#ifdef _VARIANT_ARDUINO_STM32_
  theTemp = "28.00";
#else
  theTemp = getTemp();
#endif

  readLDR();
  readAttopilot();
  motorInfo(motor1, 1);
  batteryVoltage = String(motor1[0]);


  wheelRPM = (float)motor1[4] / reductionRatio;
  wheelFPM = WHEEL_CIRCUMFERENCE * wheelRPM;

  currentMPH = wheelFPM * FPM_2_MPH;

  // Send throttle to Sabertooth ESC
  ST.motor(1, throttle);
  ST.motor(2, throttle);

  // delay(50);

  loopCount++;

  if (loopCount == 5 || loopCount == 15) {
    //update oled every half second
    // Serial.println("update oled");
    oledUpdate();
  } else if (loopCount == 10 || loopCount == 20) {
    // Serial.println("oled update and log time!");
    // log every secondish :P
    logData(false);
    oledUpdate();
    loopCount = 0;
  }
}

/*
   readLDR Function
   reads light detector and
   turns NeoPixels on or off
   Set High and Low Settings
   in the #defines LDR_HIGH
   and LDR_LOW
*/
void readLDR() {
  LDRReading = analogRead(LDR_PIN);
  // Serial.println(LDRReading);
  if (LDRReading >= LDR_HIGH) {
    isDayTime = false;
  } else if (LDRReading <= LDR_LOW) {
    isDayTime = true;
  }
  if (!isDayTime && !isLightsOn) {
    lightsOn();
    isLightsOn = true;
  } else if (isDayTime && isLightsOn) {
    lightsOff();
    isLightsOn = false;
  }
}

/*
   readAttopilot Function
   reads Attopilot and saves
   the data.
*/
void readAttopilot() {
  apVRaw = analogRead(ATTOPILOT_V);
  apARaw = analogRead(ATTOPILOT_I);
  // apVFinal = apVRaw / 49.44; // 45A board
  // apVFinal = apVRaw / 12.99; // 90A board
  // apVFinal = apVRaw / 12.99; // 180A board
  apVFinal = apVRaw / 20.77; // 180A board (adjusted)
  // apIFinal = apIRaw / 14.9; // 45A Board
  // apIFinal = apIRaw / 7.4; // 90A Board
  // apAFinal = apARaw / 3.7; // 180A Board
  apAFinal = apARaw / 14.9; // 180A Board (adjusted)
}

/*
   readChuk Function
   checks nunchuk for throttle,
   C & Z Button for values.
*/
int readChuk(String Type) {
  if (!checkNunchuk(false)) {
    killSwitch = true;
    throttle = 0;
    nunchukInfo[0] = 0;
    nunchukInfo[2] = 0;
    checkNunchuk(true);
  }

  nunchuk.update();

  int tVal;
  boolean c_state;
  boolean z_state;
  
  if (Type == "wired") {
    tVal = nunchuk.analogX;
    tVal = map(tVal, 40, 216, -2047, 2047);
    tVal = constrain(tVal, -2047, 2047);
    int c_z_buttonState = nunchuk.accelZ;

    if (c_z_buttonState == 8) {
      z_state = true;
      c_state = false;
    } else if (c_z_buttonState == 4) {
      z_state = false;
      c_state = true;
    } else if (c_z_buttonState == 0) {
      z_state = true;
      c_state = true;
    } else {
      z_state = false;
      c_state = false;
    }
    
  } else if (Type == "wireless") {
    // map values to ESC acceptable values
    tVal = nunchuk.analogY;
    tVal = map(tVal, -2, 255, -2047, 2047);
    tVal = constrain(tVal, -2047, 2047);
    
    if (!nunchuk.zButton){
      z_state = false;
    } else {
      z_state = true;
    }

    if (!nunchuk.cButton) {
      c_state = false;
    } else {
      c_state = true;
    }
  }

  // if Z button isnt pressed, 0 throttle.
  if (!z_state) {
    killSwitch = true;
    tVal = 0;
    nunchukInfo[0] = tVal;
    nunchukInfo[2] = 0;
  } else {
    killSwitch = false;
    nunchukInfo[0] = tVal;
    nunchukInfo[2] = 1;
  }

  // if C button is pressed engage horn
  if (!c_state) {
    nunchukInfo[3] = 0;
    // noTone(HORN_PIN);
  } else {
    nunchukInfo[3] = 1;
    // tone(HORN_PIN, 1000);
  }

  return tVal;
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
    Function motorInfo(array, int);
    updates info array passed to it with
    data gotten from sabertooth using the
    int given as motor #. either 1 or 2
*/
void motorInfo(float(&motorArr)[6], int whichMotor) {
  // voltage
  double battVoltage = ST.getBattery(1);
  battVoltage = battVoltage / 10;
  motorArr[0] = battVoltage;

  // amperage
  double motorAmps = ST.getCurrent(whichMotor);
  motorAmps = motorAmps / 10;
  motorArr[1] = motorAmps;

  // temperature
  double motorTemp = ST.getTemperature(whichMotor);
  motorTemp = (motorTemp * 1.8) / 10;
  motorArr[2] = motorTemp;

  // max rpm
  double maxRPM = 0.0;
  maxRPM = battVoltage * MOTOR_RPM_2_VOLT;
  motorArr[3] = maxRPM;

  // current rpm
  double currRPM = 0.0;
  currRPM = mapf(throttlePercentage, 0, 100, 0, maxRPM);
  motorArr[4] = currRPM;

  // current torque
  double currTorque = 0.0;
  currTorque = apAFinal * MOTOR_OZIN_2_AMP;
  motorArr[5] = currTorque; 

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

  double tmp = (temp_msb * 1.8) + 28;
  temp = String(tmp);
  // temp = (temp_msb * 1.8) + 28;
  return temp;
}


void logData(boolean toSerial) {
  String dataString = "";

  dataString += String(timeNow);        // the time now
  dataString += String(",");
  dataString += String(theTemp);        // the temp outside
  dataString += String(",");
  dataString += String(LDRReading);     // brightness outside
  dataString += String(",");
  dataString += String(isLightsOn);     // lights on/off state
  dataString += String(",");
  dataString += String(nunchukInfo[0]); // throttle
  dataString += String(",");
  dataString += String(nunchukInfo[1]); // throttle %
  dataString += String(",");
  dataString += String(nunchukInfo[2]); // z-button state
  dataString += String(",");
  dataString += String(nunchukInfo[3]); // c-button state
  dataString += String(",");
  dataString += String(apVFinal);       // attopilot voltage
  dataString += String(",");
  dataString += String(apAFinal);       // attopilot amperage
  dataString += String(",");
  dataString += String(motor1[0]);      // sabertooth battery voltage
  dataString += String(",");
  dataString += String(motor1[1]);      // sabertooth motor amperage
  dataString += String(",");
  dataString += String(motor1[2]);      // sabertooth motor temprature
  dataString += String(",");
  dataString += String(motor1[5]);      // sabertooth motor torque
  dataString += String(",");
  dataString += String(motor1[4]);      // sabertooth motor current rpm (theorhetical)
  dataString += String(",");
  dataString += String(motor1[3]);      // sabertooth motor maximum rpm (theorhetical)
  dataString += String(",");
  dataString += String(wheelRPM);       // wheel rpm (theorhetical)
  dataString += String(",");
  dataString += String(currentMPH);     // current mph (theorhetical)
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  // "/logs/" + <= don't forget to add to logName!!! 
  logFile = SD.open("/logs/" + logName, FILE_WRITE);
  
  // if the file is available, write to it:
  if (SD.exists("/logs/" + logName)) {
    logFile.println(dataString);
    logFile.flush();
    // logFile.close();
  } else {
    // Serial.println("log file not found to datalog to....");
  }
  if(toSerial) {
    Serial.println(dataString);
  }
}

/*
    Function mapf
    returns a map with floats.
*/
double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
