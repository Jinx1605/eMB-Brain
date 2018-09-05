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

#define TCAADDR 0x70

struct MotorInfo {
  String name;
  int    tcaport;
  double ambient;
  double object;
};

MotorInfo left_motor  = {"Left Motor" , 7, 00.00, 00.00};
MotorInfo right_motor = {"Right Motor", 1, 00.00, 00.00};

String format_time(int hr, int mn, int sx, int tzd){
  String theTime = "";
  String hur  = ((hr + tzd) > 12 ) ? String((hr + tzd) - 12 ) : String(hr + tzd);
  String ampm = ((hr + tzd) > 12 ) ? "pm" : "am";
  String min  = (mn < 10) ? "0" + String(mn) : String(mn);
  String sec  = (sx < 10) ? "0" + String(sx) : String(sx);
  theTime += hur;
  theTime += min;
  theTime += sec;
  theTime += " " + ampm;
  return theTime;
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

  init_motor(left_motor);
  init_motor(right_motor);
  
}

void loop() {

  collect_motor_temps(left_motor);
  collect_motor_temps(right_motor);
  delay(250);
  
}

void init_motor (MotorInfo side) {
  init_display(side.tcaport, "Left Sensor Init.");
  init_mlx(side);
}

void init_mlx (MotorInfo side) {
    if (!MLX90614_begin(side.tcaport)) {
    Serial.print(side.name + " Temp Sensor not found!");
    display.clearDisplay();
    display.setCursor(0,0);
    display.print(side.name + " Sensor Init Failed");
    display.display();
    while(1);
  } else {
    Serial.println(side.name + " Left Sensor init passed!");
  }
}

void init_display(uint8_t i, String txt) {
  tcaselect(i);
  delay(100);
  //display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.begin();
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print(txt);
  display.display();
  delay(50);
  // init done
}

void collect_motor_temps (MotorInfo side) {
  tcaselect(side.tcaport);
  side.ambient = readAmbientTempF();
  side.object = readObjectTempF();
  display_motor_temps(side);
}

void display_motor_temps(MotorInfo side) {
  Serial.print(side.name + " Ambient = "); Serial.print(side.ambient); 
  Serial.print("*F\t" + side.name +" = "); Serial.print(side.object); Serial.println("*F");
  Serial.println();
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("A:" + String(side.ambient) + " F");
  display.setCursor(0,16);
  display.println("M:" + String(side.object) + " F");
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
