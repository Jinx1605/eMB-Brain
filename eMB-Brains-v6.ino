/*************************************************** 
  eMB Brains v5 - I2C Gone Nanners!!

  - Dual Multiplexed Temp Monitor -
    1x TCA9548A I2C Multiplexer
    2x MLX90614 I2C Temp sensors
    2x 128x32 I2C OLED Displays

  
 ****************************************************/
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

// OakOLED Library.
// modified to use zio 128x32 Qwiic oleds..
#include "OakOLED.h"
OakOLED display;

#include <SparkFun_I2C_GPS_Arduino_Library.h>
I2CGPS myI2CGPS;

#include <TinyGPS++.h> //From: https://github.com/mikalhart/TinyGPSPlus
TinyGPSPlus gps; //Declare gps object
#define TZDIFF -5

#include <Sparkfun_APDS9301_Library.h>
APDS9301 apds;
uint8_t lux_val = 0;

#include "Qwiic_LED_Stick.h"
LED BackLED; //Create an object of the LED class

byte redArray[10]   = {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}; //r
byte greenArray[10] = {  0,   0,   0,   0,   0,   0,   0,   0,   0,   0}; //g
byte blueArray[10]  = {  0,   0,   0,   0,   0,   0,   0,   0,   0,   0}; //b

#if defined(__SAMD51__)

#else
// samd21 or other arduino
#include <RHReliableDatagram.h>
#include <RH_RF69.h>

#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define RF69_FREQ     915.0

// Where to send packets to!
#define DEST_ADDRESS  2
// change addresses for each client board, any number :)
#define MY_ADDRESS    1

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

// The encryption key has to be the same as the one in the server
uint8_t crypt_key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};


uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "";
char radiopacket[RH_RF69_MAX_MESSAGE_LEN] = "";
#endif

boolean joystick_connected = false;
int joystick_info[6] = {
  0, // vertical
  0, // horizontal
  0, // j-button state
  0, // z-button state
  0, // c-button state
  0  // battery voltage
};

/************ DS321 RTC Module Setup ***********/
#include "RTClib.h"
RTC_DS3231 rtc;

#define DS3231_I2C_ADDR 0x68
#define DS3231_TEMP_MSB 0x11

char daysOfTheWeek[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

struct RTC_Time {
  int month;
  int day;
  int year;
  int hour;
  int minute;
  int second;
  String temp;
};

RTC_Time rtc_data;


/************ SDCard Setup ***************/
#include <SD.h>
#define SD_CHIP_SELECT 9
File logFile;

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


#define TCAADDR 0x70

struct MotorInfo {
  String name;
  int    tcaport;
  double ambient;
  double object;
};

MotorInfo left_motor  = {"Left Motor" , 7, 00.00, 00.00};
MotorInfo right_motor = {"Right Motor", 1, 00.00, 00.00};


struct GPSData {
  String  time;
  String  date;
  uint8_t lux;
  uint8_t sats;
  double  lat;
  double  lng;
  uint8_t deg;
  String  card;
  double  mph;
  double  kmh;
};

GPSData gData = {
  "",   // time
  "",   // date
  0,    // lux
  0,    // sats
  0.00, // lat
  0.00, // long
  0,    // deg
  "",   // card
  0.00, // mph
  0.00 // kmph
  
};

boolean enableDebug = true;

// Global Use data
String logName = "";
String dateNow = "";
String timeNow = "";

String format_time(int hr, int mn, int sx, int tzd){
  String theTime = "";
  int hour = hr + tzd;
  String ampm = (hour >= 12 ) ? "pm" : "am";
  hour = (hour > 12) ? (hour - 12) : hour;
  hour = (hour == 0) ? 12 : hour;
  if (hour < 0) { ampm = "pm"; };
  hour  = (hour <= 0) ? hour + 12 : hour;
  String hur  = String(hour);
  String min  = (mn < 10) ? "0" + String(mn) : String(mn);
  String sec  = (sx < 10) ? "0" + String(sx) : String(sx);
  theTime += hur + ":";
  theTime += min + ":";
  theTime += sec;
  theTime += " " + ampm;
  return theTime;
}

String format_date(int mn, int dy, int yr) {
  String theDate = "";
  String month = (mn < 10) ? "0" + String(mn) : String(mn);
  String day   = (dy < 10) ? "0" + String(dy) : String(dy);
  String year  = String((yr - 2000));
  theDate += month + "-" + day + "-" + year;
  return theDate;
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);

  RTC_Init();
  
  /*
  rtc_data = RTC_Now();
  Serial.print(rtc_data.month);
  Serial.print("-");
  Serial.print(rtc_data.day);
  Serial.print("-");
  Serial.print(rtc_data.year);
  Serial.print(" | ");
  Serial.print(rtc_data.hour);
  Serial.print(":");
  Serial.print(rtc_data.minute);
  Serial.print(":");
  Serial.print(rtc_data.second);
  Serial.print(" | ");
  Serial.print(rtc_data.temp);
  Serial.println();
  */
  
  delay(50);
  init_motor(left_motor,  50);
  init_motor(right_motor, 50);
  
  if (!myI2CGPS.begin()) {
    oled_print(left_motor, "GPS Module");
    oled_print(right_motor, "Failed Init");
    Serial.println("Module failed to respond. Please check wiring.");
    while (1); //Freeze!
  } else {
    oled_print(left_motor, "GPS Module");
    oled_print(right_motor, "Working!");
    Serial.println("GPS module found!");
  }

  while (myI2CGPS.available()) {
    gps.encode(myI2CGPS.read()); //Feed the GPS parser
  }
  if (gps.time.isValid()) {
    rtc.adjust(DateTime(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second()));
    timeNow = format_time(gps.time.hour(),gps.time.minute(),gps.time.second(), TZDIFF);
    dateNow = format_date(gps.date.month(), gps.date.day(), gps.date.year());
    Serial.println(timeNow + " - " + dateNow);
    oled_print(left_motor, dateNow);
    oled_print(right_motor, timeNow);
  }

  delay(500);

  
  // Init SD
  //SDCard_init();

  init_light_sensor();
  
  BackLED.begin();
  BackLED.setLEDBrightness(2);
  BackLED.setLEDColor(255, 0, 0, 10);
  
  #if defined(__SAMD51__)
  
  #else
  // samd21 or arduino
  radio_init();
  #endif
}

void loop() {

  while (myI2CGPS.available()) {
    gps.encode(myI2CGPS.read()); //Feed the GPS parser
  }

  collect_motor_temps(left_motor);
  collect_motor_temps(right_motor);
  lux_val = read_light_sensor();
  
  //Check to see if new GPS info is available
  if (gps.time.isUpdated()) {
    // displayInfo();
    process_gps(gData, enableDebug);
  }
  #if defined(__SAMD51__)
  #else
  radio_create_packet();
  radio_read();
  #endif
}

#if defined(__SAMD51__)

#else
// samd21 or arduino
void radio_init() {
  Serial.println("RFM69 Radio Init Start.");
  // RFM69 Setup  
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init()) {
    Serial.println("RFM69 Radio Init Failed");
    while (1);
  }
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(14, true);
  
  // set encryption
  rf69.setEncryptionKey(crypt_key);

  Serial.println("RFM69 Radio Init Success!");
}

void radio_decode(char* data) {
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

  joystick_info[0] = vert.toInt();
  joystick_info[1] = horz.toInt();
  joystick_info[2] = jbut.toInt();
  joystick_info[3] = zbut.toInt();
  joystick_info[4] = cbut.toInt();
  joystick_info[5] = bvol.toInt();

  if (!joystick_info[2]) {
    joystick_info[2] = 1;
  } else {
    joystick_info[2] = 0;
  }

  if (!joystick_info[3]) {
    joystick_info[3] = 1;
  } else {
    joystick_info[3] = 0;
  }

  if (!joystick_info[4]) {
    joystick_info[4] = 1;
  } else {
    joystick_info[4] = 0;
  }

}

void radio_read() {
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS)) {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rf69_manager.recvfromAckTimeout(buf, &len, 500, &from)) {
      //killswitch(false);
      buf[len] = 0;
      //      Serial.print("Remote #");
      //      Serial.print(from); Serial.print(": ");
      //      Serial.println((char*)buf);

      radio_decode((char*)buf);
    }
  } else {
    if (joystick_connected) {
      //killswitch(true);
    }
  }
}

void radio_create_packet() {
  String data_to_send = "";

  data_to_send = timeNow;
  data_to_send += ",";
  data_to_send += dateNow;

  data_to_send.toCharArray(radiopacket, data_to_send.length() + 1);
}

#endif

/*
   RTC_Init Function
   checks for RTC Module presence.
*/
void RTC_Init() {
  
  if (!RTC_Check()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
}

boolean RTC_Check() {
  Wire.begin();
  Wire.beginTransmission(DS3231_I2C_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("RTC present");
  } else {
    Serial.println("RTC not found");
    return false;
  }
  return true;
}

/*
   RTC_Now Function
   Returns RTC Data in RTC_Time Structure.
*/
RTC_Time RTC_Now() {
  DateTime now = rtc.now();
  RTC_Time data;
  data.month  = now.month();
  data.day    = now.day();
  data.year   = now.year();
  data.hour   = now.hour();
  data.minute = now.minute();
  data.second = now.second();
  data.temp   = RTC_Temp() + "F";
  return data;
}

/*
    Function getTemp
    returns approximate temperature
    from DS3231 RTC Module in F.
*/
String RTC_Temp() {
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

/*
   SDCard_init Function
   checks for SD Card presence.
*/
void SDCard_init() {
  Serial.println("Init the SD Card");
  if (!SD.begin(SD_CHIP_SELECT)) {
    Serial.println("No SD Card Found");
    //oledPrint("SD Card  is Missing", 500);
    // die
    while (1);
  } else {
    Serial.println("Card Initalized");
    //oledPrint("SD Card Connected. ", 500);
    LogFile_setup();
  }
}

/*
   setupLogFile Function
   creates logfile for the
   current date.
*/
void LogFile_setup() {
  String logDate = dateNow;
  logName = logDate + ".csv";

  if (!SD.exists("/logs/")) {
    Serial.println("Logs Folder Missing, Creating now...");
    //oledPrint("logs folder missing!", 500);
    //oledPrint("creating logs folder", 500);
    if (SD.mkdir("/logs/")) {
      Serial.println("Logs Folder Created.");
      //oledPrint("created logs folder", 500);
    } else {
      Serial.println("Error Creating Logs Folder...");
      //oledPrint("error making folder", 0);
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
    //oledPrint(logName + " present!", 500);
    Serial.print(logName + " present!");
  } else {
    //oledPrint(logName + " missing!", 0);
    Serial.print(logName + " missing!");
    while (1);
  }
}

void process_gps(GPSData data, boolean debug) {
  data.time = format_time(gps.time.hour(),gps.time.minute(),gps.time.second(), TZDIFF);
  data.date = format_date(gps.date.month(), gps.date.day(), gps.date.year());
  data.lux  = lux_val;
  data.sats = gps.satellites.value();
  data.lat  = gps.location.lat();
  data.lng  = gps.location.lng();
  data.deg  = gps.course.deg();
  data.card = TinyGPSPlus::cardinal(gps.course.deg());
  data.mph  = gps.speed.mph();
  data.kmh  = gps.speed.kmph();

  timeNow = data.time;
  dateNow = data.date;
  
  if (debug) {
    sprint_gps_data(data);
  }
}

void sprint_gps_data(GPSData data) {
  Serial.print(data.time + ",");
  Serial.print(data.date + ",");
  Serial.print(data.lux);
  Serial.print(",");
  Serial.print(data.sats);
  Serial.print(",");
  Serial.print(data.lat, 6);
  Serial.print(",");
  Serial.print(data.lng, 6);
  Serial.print(",");
  Serial.print(data.deg);
  Serial.print(",");
  Serial.print(data.card + ",");
  Serial.print(data.mph);
  //Serial.print(",");
  Serial.println();
}

void init_light_sensor(){
  delay(5);
  // APDS9301 sensor setup.
  apds.begin(0x39);
  apds.setGain(APDS9301::LOW_GAIN);

  apds.setIntegrationTime(APDS9301::INT_TIME_13_7_MS);
  apds.setLowThreshold(0);
  apds.setHighThreshold(500);
  apds.setCyclesForInterrupt(1);
  apds.enableInterrupt(APDS9301::INT_ON);
  apds.clearIntFlag();

  //Serial.println(apds.getLowThreshold());
  //Serial.println(apds.getHighThreshold());
}

uint8_t read_light_sensor() {
  return apds.readCH0Level();
}

void init_motor (MotorInfo side, uint8_t dely) {
  init_display(side.tcaport, side.name + " MLX90614 Init.");
  delay(dely);
  init_mlx(side);
  delay(dely);
}

void init_mlx (MotorInfo side) {
    if (!MLX90614_begin(side.tcaport)) {
    Serial.print(side.name + " MLX90614 not found!");
    display.clearDisplay();
    display.setCursor(0,0);
    display.print(side.name + " MLX90614 Init Failed");
    display.display();
    while(1);
  } else {
    Serial.println(side.name + " MLX90614 init passed!");
  }
}

void init_display(uint8_t i, String txt) {
  tcaselect(i);
  delay(50);
  //display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.begin();
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print(txt);
  display.display();
  // init done
}

void collect_motor_temps (MotorInfo side) {
  tcaselect(side.tcaport);
  side.ambient = readAmbientTempF();
  side.object = readObjectTempF();
  display_motor_temps(side);
}

void display_motor_temps(MotorInfo side) {
  //Serial.print(side.name + " Ambient = "); Serial.print(side.ambient); 
  //Serial.print("*F\t" + side.name +" = "); Serial.print(side.object); Serial.println("*F");
  //Serial.println();
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("A:" + String(side.ambient) + " F");
  display.setCursor(0,16);
  display.println("M:" + String(side.object) + " F");
  display.display();
}

void oled_print(MotorInfo side, String txt) {
  tcaselect(side.tcaport);
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(txt);
  display.display();
}

boolean MLX90614_begin(uint8_t tca_port) {
  tcaselect(tca_port);
  Wire.beginTransmission(0x5A);
  if (Wire.endTransmission() == 0) {
    return true;
  } else {
    return false;
  }
}

double readObjectTempF(void) {
  return (readTemp(0x07) * 9 / 5) + 32;
}

double readAmbientTempF(void) {
  return (readTemp(0x06) * 9 / 5) + 32;
}

float readTemp (uint8_t reg) {
  float temp;
  temp = read16(reg);
  temp *= .02;
  temp -= 273.15;
  return temp;
}

uint16_t read16(uint8_t a) {
  uint16_t ret;
  Wire.beginTransmission(0x5A); //MLX90614
  Wire.write(a);
  Wire.endTransmission(false);

  Wire.requestFrom(0x5A, (uint8_t)3);
  ret = Wire.read();
  ret |= Wire.read() << 8;

  //uint8_t pec = Wire.read();

  return ret;
}
