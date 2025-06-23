#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// NRF24L01 setup
RF24 radio(9, 10);  // CE, CSN
const byte address[6] = "00001";

// Motor control pins
const int enA = 5;
const int in1 = 7;
const int in2 = 8;
const int enB = 6;
const int in3 = A1;
const int in4 = A0;

// Encoder pins
const int encoderA1Pin = 2;
const int encoderA2Pin = 3;
const int pulsesPerRevolution = 11 * 103;

volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

unsigned long prevTime = 0;
long prevCountA = 0;
long prevCountB = 0;

// PID parameters
float setpointRPM = 38.0;

float Kp = 3;
float Ki = 0.05;
float Kd = 0.15;

float errorA = 0, lastErrorA = 0, integralA = 0;
float errorB = 0, lastErrorB = 0, integralB = 0;

int pwmA = 0;
int pwmB = 0;

const int filterSize = 2;
float rpmA_history[filterSize] = {0};
float rpmB_history[filterSize] = {0};
float rpmA_sum = 0;
float rpmB_sum = 0;
int rpmIndex = 0;

bool running = false;
char lastDirection = 'S'; // Default direction

// Data package for transmission
struct Data_Package {
  char direction;
  float motorRPML;
  float motorRPMR;
};
Data_Package data;

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Encoder pins
  pinMode(encoderA1Pin, INPUT);
  pinMode(encoderA2Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA1Pin), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA2Pin), encoderB_ISR, RISING);

  // NRF24L01
  radio.begin();
  radio.openWritingPipe(address);   // <== ADD THIS LINE
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();


  prevTime = millis();
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));
    running = (data.direction != 'S');
    if (running) lastDirection = data.direction;
  }

  if (millis() - prevTime >= 200) {
    long countA = encoderCountA;
    long countB = encoderCountB;

    float deltaTime = (millis() - prevTime) / 1000.0;

    long deltaA = countA - prevCountA;
    long deltaB = countB - prevCountB;

    float rpmA = (deltaA / (float)pulsesPerRevolution) / deltaTime * 60.0;
    float rpmB = (deltaB / (float)pulsesPerRevolution) / deltaTime * 60.0;

    rpmA_sum = rpmA_sum - rpmA_history[rpmIndex] + rpmA;
    rpmB_sum = rpmB_sum - rpmB_history[rpmIndex] + rpmB;

    rpmA_history[rpmIndex] = rpmA;
    rpmB_history[rpmIndex] = rpmB;

    rpmIndex = (rpmIndex + 1) % filterSize;

    float avg_rpmA = rpmA_sum / filterSize;
    float avg_rpmB = rpmB_sum / filterSize;

    Serial.print("RPM A: ");
    Serial.print(avg_rpmA, 2);
    Serial.print(" | PWM A: ");
    Serial.print(pwmA);
    Serial.print(" | RPM B: ");
    Serial.print(avg_rpmB, 2);
    Serial.print(" | PWM B: ");
    Serial.print(pwmB);
    Serial.print(" | Dir: ");
    Serial.println(lastDirection);

    // Update data package
    data.motorRPML = avg_rpmA;
    data.motorRPMR = avg_rpmB;
    data.direction = lastDirection;

    // Send RPM and direction data to remote control via NRF24L01
    radio.stopListening(); // Stop listening so we can transmit
    bool success = radio.write(&data, sizeof(Data_Package));
    radio.startListening(); // Resume listening for commands
    
    if (success) {
      Serial.println("Data sent successfully.");
      } else {
        Serial.println("Failed to send data.");
      }


    if (running) {
      errorA = setpointRPM - avg_rpmA;
      integralA += errorA * deltaTime;
      float derivativeA = (errorA - lastErrorA) / deltaTime;
      pwmA += Kp * errorA + Ki * integralA + Kd * derivativeA;
      pwmA = constrain(pwmA, 0, 255);
      lastErrorA = errorA;

      errorB = setpointRPM - avg_rpmB;
      integralB += errorB * deltaTime;
      float derivativeB = (errorB - lastErrorB) / deltaTime;
      pwmB += Kp * errorB + Ki * integralB + Kd * derivativeB;
      pwmB = constrain(pwmB, 0, 255);
      lastErrorB = errorB;

      setMotorDirection(lastDirection);
      analogWrite(enA, pwmA);
      analogWrite(enB, pwmB);
    } else {
      analogWrite(enA, 0);
      analogWrite(enB, 0);
    }

    prevCountA = countA;
    prevCountB = countB;
    prevTime = millis();
  }
}

void setMotorDirection(char dir) {
  switch (dir) {
    case 'F': // Forward
      digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
      break;
    case 'B': // Backward
      digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
      break;
    case 'L': // Turn left
      digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
      break;
    case 'R': // Turn right
      digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
      digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
      break;
    case 'S': // Stop
    default:
      digitalWrite(in1, LOW); digitalWrite(in2, LOW);
      digitalWrite(in3, LOW); digitalWrite(in4, LOW);
      break;
  }
}

void encoderA_ISR() {
  encoderCountA++;
}

void encoderB_ISR() {
  encoderCountB++;
}
