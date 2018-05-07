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

// Where to send packets to!
#define DEST_ADDRESS   1

// change addresses for each client board, any number :)
#define MY_ADDRESS     2
#define LED 13

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
#endif

byte DEV_ADDR = 0x20;
int reading = 0;

int joystiq_info[5] = {
  0, // vertical
  0, // horizontal
  0, // j-button state
  0, // z-button state
  0  // c-button state
};

int c_button_pin = 5;
int z_button_pin = 6;

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

// Global Use data
String dateNow = "";
String timeNow = "";


void setup() {
  
  Wire.begin();// join i2c bus

  pinMode(c_button_pin, INPUT_PULLUP);
  pinMode(z_button_pin, INPUT_PULLUP);

  Serial.begin(115200);
  // while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  checkRadio();
  
  pinMode(LED, OUTPUT);
  
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
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x06, 0x00, 0x05, 0x01, 0x06, 0x00, 0x05,
                    0x01, 0x06, 0x00, 0x05, 0x01, 0x06, 0x00, 0x05};
  rf69.setEncryptionKey(key);
}

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
char data[] = "";
char radiopacket[40] = "";

void loop() {  

  read_joystiq();
  format_joystiq_data();

  if (rf69_manager.available()) {
  // Wait for board brain Ack
  uint8_t len = sizeof(buf);
  uint8_t from;
    if (rf69_manager.recvfromAck(buf, &len, &from)) {
     buf[len] = 0;
     Serial.print("got request from : 0x");
     Serial.print(from, HEX);
     Serial.print(": ");
     
     decodeRemotePacket((char*)buf);

     Serial.print(timeNow);
     
     Serial.print(" - RSSI: "); Serial.println(rf69.lastRssi(), DEC);
    
     // echo last button       
     // Send data back to the board brain.
     Serial.print("Sending: ");
     Serial.println(radiopacket);
     if (!rf69_manager.sendtoWait((uint8_t*)radiopacket, sizeof(radiopacket), from)){
      Serial.println("reply to brain from remote failed");
     }
     digitalWrite(LED, HIGH);
     digitalWrite(LED, LOW);
    }
  
  }
  
}

void read_joystiq() {
  
  joystiq_info[0] = getVertical();
  joystiq_info[1] = getHorizontal();
  joystiq_info[2] = getButton();
  joystiq_info[3] = digitalRead(z_button_pin);
  joystiq_info[4] = digitalRead(c_button_pin);

}

void format_joystiq_data() {
  String joystiq_string = "";
  
  joystiq_string = joystiq_info[0];
  joystiq_string += ",";
  joystiq_string += joystiq_info[1];
  joystiq_string += ",";
  joystiq_string += joystiq_info[2];
  joystiq_string += ",";
  joystiq_string += joystiq_info[3];
  joystiq_string += ",";
  joystiq_string += joystiq_info[4];
  
  joystiq_string.toCharArray(radiopacket, joystiq_string.length() + 1);
}

void decodeRemotePacket(char* data) {
    String the_data(data);
    int ind1   = the_data.indexOf(",");
    int ind2   = the_data.indexOf(",", ind1+1);

    timeNow = the_data.substring(0, ind1);
    dateNow = the_data.substring(ind1+1,ind2);
    
}

// getHorizontal() returns an int between 0 and 1023 
// representing the Joystiic's horizontal position
// (axis indicated by silkscreen on the board)
// centered roughly on 512
int getHorizontal() {

  Wire.beginTransmission(DEV_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(DEV_ADDR, 2);

  while (Wire.available()) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    reading = (uint16_t)msb << 8 | lsb;
  }

  Wire.endTransmission();
  return reading;

}

// getVertical() returns an int between 0 and 1023 
// representing the Joystiic's vertical position
// (axis indicated by silkscreen on the board)
// centered roughly on 512
int getVertical() {

  Wire.beginTransmission(DEV_ADDR);
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(DEV_ADDR, 2);

  while (Wire.available()) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    reading = (uint16_t)msb << 8 | lsb;
  }

  Wire.endTransmission();
  return reading;

}

// getButton() returns a byte indicating the 
// position of the button where a 0b1 is not
// pressed and 0b0 is pressed
byte getButton() {

  byte btnbyte = 0b00000000;

  Wire.beginTransmission(DEV_ADDR);
  Wire.write(0x04);
  Wire.endTransmission(false);
  Wire.requestFrom(DEV_ADDR, 1);    // request 1 byte

  while (Wire.available()) {
    btnbyte = Wire.read();
  }

  Wire.endTransmission();
  return btnbyte;

}

