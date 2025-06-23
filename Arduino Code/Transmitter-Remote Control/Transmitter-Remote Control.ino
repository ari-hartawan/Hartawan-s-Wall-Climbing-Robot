#include <PS2X_lib.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

PS2X ps2x;
RF24 radio(7, 8);

const byte address[6] = "00001";

struct Data_Package {
  char direction;
  float motorRPML;
  float motorRPMR;
};
Data_Package data;

void setup() {
  Serial.begin(9600);
  int error = ps2x.config_gamepad(5, 3, 2, 4, true, true);
  if (error == 0) {
    Serial.println("PS2 connected.");
  } else {
    Serial.println("PS2 connection failed.");
  }

  radio.begin();
  radio.openWritingPipe(address);
  radio.openReadingPipe(0, address);

  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
}

void loop() {
  ps2x.read_gamepad(false, 0);

  if (ps2x.Button(PSB_PAD_UP)) {
    data.direction = 'F';
  } else if (ps2x.Button(PSB_PAD_DOWN)) {
    data.direction = 'B';
  } else if (ps2x.Button(PSB_PAD_LEFT)) {
    data.direction = 'L';
  } else if (ps2x.Button(PSB_PAD_RIGHT)) {
    data.direction = 'R';
  } else if (ps2x.Button(PSB_GREEN)) {
    data.direction = 'T';
  } else if (ps2x.Button(PSB_BLUE)) {
    data.direction = 'C';
  } else if (ps2x.Button(PSB_PINK)) {
    data.direction = 'Q';
  } else if (ps2x.Button(PSB_RED)) {
    data.direction = 'O';
  } else {
    data.direction = 'S';
  }

  radio.stopListening();
  if (radio.write(&data, sizeof(Data_Package))) {
    //Serial.println("Control data sent successfully.");
  } 
  radio.startListening();

  // Add debug to check if data is received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));
    Serial.print("Received Data | Direction: ");
    Serial.print(data.direction);
    Serial.print(" | MotorL RPM: ");
    Serial.print(data.motorRPML, 2);
    Serial.print(" | MotorR RPM: ");
    Serial.println(data.motorRPMR, 2);
  } else {
    //Serial.println("No data received.");
  }

  delay(20);
}
