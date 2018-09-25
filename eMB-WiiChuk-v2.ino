
#include <SPI.h>
#include <Wire.h>
#include <RHReliableDatagram.h>
#include <RH_RF69.h>


#define VERT          1
#define HORZ          0
#define JBTN          11
#define ZBTN          12
#define CBTN          13

#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define RF69_FREQ     915.0

// Where to send packets to!
#define DEST_ADDRESS  1
// change addresses for each client board, any number :)
#define MY_ADDRESS    2

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

// The encryption key has to be the same as the one in the server
uint8_t crypt_key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

uint16_t vert_val, horz_val;
boolean jbtn_val, zbtn_val, cbtn_val;


void setup() {
  Wire.begin(); // setup i2c
  Serial.begin(9600); // setup serial

  // Joystick Setup
  pinMode(JBTN, INPUT_PULLUP);
  pinMode(ZBTN, INPUT_PULLUP);
  pinMode(CBTN, INPUT_PULLUP);

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

void loop() {
  // put your main code here, to run repeatedly:
  
  // read all values from the joystick
  vert_val = analogRead(VERT); // will be 0-1023
  horz_val = analogRead(HORZ); // will be 0-1023
  jbtn_val = digitalRead(JBTN); // will be HIGH (1) if not pressed, and LOW (0) if pressed
  
  // print out the values
  
  Serial.print("vertical: ");
  Serial.print(vert_val,DEC);
  Serial.print(" horizontal: ");
  Serial.print(horz_val,DEC);
  Serial.print(" select: ");
  if(jbtn_val){
    Serial.println("not pressed");
  } else {
    Serial.println("PRESSED!");
  }
}
