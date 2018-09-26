/*************************************************** 
  eMB Brains v5 - I2C Gone Nanners!!

  - Dual Multiplexed Temp Monitor -
    1x TCA9548A I2C Multiplexer
    2x MLX90614 I2C Temp sensors
    2x 128x32 I2C OLED Displays

  
 ****************************************************/

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

#include <Sparkfun_APDS9301_Library.h>
APDS9301 apds;
uint8_t lux_val = 0;

#include "Qwiic_LED_Stick.h"
LED BackLED; //Create an object of the LED class

byte redArray[10]   = {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}; //r
byte greenArray[10] = {  0,   0,   0,   0,   0,   0,   0,   0,   0,   0}; //g
byte blueArray[10]  = {  0,   0,   0,   0,   0,   0,   0,   0,   0,   0}; //b

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
char radiopacket[RH_RF69_MAX_MESSAGE_LEN] = "time";

boolean joystick_connected = false;
int joystick_info[6] = {
  0, // vertical
  0, // horizontal
  0, // j-button state
  0, // z-button state
  0, // c-button state
  0  // battery voltage
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
  Serial.begin(9600);
  //while (!Serial);
  
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

  init_light_sensor();
  
  BackLED.begin();
  BackLED.setLEDBrightness(2);
  BackLED.setLEDColor(redArray, greenArray, blueArray, 10);

  radio_init();

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
    process_gps(gData);
  }

  radio_read();
  
}

void radio_init() {
  // RFM69 Setup  
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
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


void process_gps(GPSData data){
  data.time = format_time(gps.time.hour(),gps.time.minute(),gps.time.second(),-4);
  data.date = format_date(gps.date.month(), gps.date.day(), gps.date.year());
  data.lux  = lux_val;
  data.sats = gps.satellites.value();
  data.lat  = gps.location.lat();
  data.lng  = gps.location.lng();
  data.deg  = gps.course.deg();
  data.card = TinyGPSPlus::cardinal(gps.course.deg());
  data.mph  = gps.speed.mph();
  data.kmh  = gps.speed.kmph();

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
