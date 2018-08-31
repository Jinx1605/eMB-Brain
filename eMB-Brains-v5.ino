/*************************************************** 
  eMB Brains v4 - Dual Multiplexed Temp Monitor
 ****************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

#include "OakOLED.h"
OakOLED display;

// #include <Adafruit_SSD1306.h>
// #define OLED_RESET 4
// Adafruit_SSD1306 display(OLED_RESET);

#define TCAADDR 0x70

double r_a_temp;
double r_o_temp;
double l_a_temp;
double l_o_temp;

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

  init_display(7, "Left Sensor Init.");
  init_display(1, "Right Sensor Init.");
  
  /* Initialise the left sensor */
  if (!MLX90614_begin(7)) {
    Serial.println("Left Temp Sensor not found!");
    display.clearDisplay();
    display.setTextSize(2);
    //display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.print("Left Sensor Init Failed");
    display.display();
    while(1);
  } else {
    Serial.println("Left Temp Sensor init passed!");
  }
  
  /* Initialise the right sensor */
  if (!MLX90614_begin(1)) {
    Serial.println("Right Temp Sensor not found!");
    display.clearDisplay();
    display.setTextSize(2);
    //display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.print("Right Sensor Init Failed");
    display.display();
    while(1);
  } else {
    Serial.println("Right Temp Sensor init passed!");
  }
  
}

void loop() {
  
  tcaselect(7);
  l_a_temp = readAmbientTempF();
  l_o_temp = readObjectTempF();
  Serial.print("Left Ambient = "); Serial.print(l_a_temp); 
  Serial.print("*F\tLeft Object = "); Serial.print(l_o_temp); Serial.println("*F");
  Serial.println();
  display_temp(String(l_a_temp), String(l_o_temp));
  
  tcaselect(1);
  r_a_temp = readAmbientTempF();
  r_o_temp = readObjectTempF();
  Serial.print("Right Ambient = "); Serial.print(r_a_temp); 
  Serial.print("*F\tRight Object = "); Serial.print(r_o_temp); Serial.println("*F");
  Serial.println();
  display_temp( String(r_a_temp), String(r_o_temp));
  
  delay(250);
  //display_temps();
}

void init_display(uint8_t i, String txt) {
  tcaselect(i);
  delay(100);
  //display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.begin();
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(2);
  //display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print(txt);
  display.display();
  delay(50);
  // init done
}

void display_temp(String ambient_temp, String object_temp) {
  display.clearDisplay();
  display.setTextSize(2);
  //display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("A:" + ambient_temp + " F");
  display.setCursor(0,16);
  display.println("M:" + object_temp + " F");
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
