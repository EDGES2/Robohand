/*
  Robo Hand — Arduino Nano (SG90 180°)
  Protocol: <IDX,ANGLE>\n  |  IDX: 0–4,  ANGLE: 0–180

  Servo pins:
    D2  – thumb (0)       D4  – index (1)
    D6  – middle (2)      D7  – ring (3)
    D11 – pinky (4)

  Calibration: swap ANGLE_OPEN / ANGLE_CLOSE for a finger if it moves in reverse.
*/

#include <Servo.h>

const int BAUD_RATE    = 9600;
const int NUM_SERVOS   = 5;
const int SERVO_PINS[] = {2, 4, 6, 7, 11};

//                         [thumb, index, middle, ring, pinky]
const int ANGLE_CLOSE[] = {   70,    70,     70,   70,    70 };
const int ANGLE_OPEN[]  = {  180,   180,    180,  180,   180 };

// Timeout: no packets received → return all fingers to open position
const unsigned long TIMEOUT_MS = 3000;

Servo servos[NUM_SERVOS];
int   currentAngle[NUM_SERVOS];

const int BUFFER_SIZE = 32;
char      rxBuffer[BUFFER_SIZE];
int       rxIndex     = 0;
bool      packetReady = false;

unsigned long lastPacketTime = 0;
bool          timedOut       = false;

void setup() {
  Serial.begin(BAUD_RATE);

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    currentAngle[i] = ANGLE_OPEN[i];
    servos[i].write(ANGLE_OPEN[i]);
  }

  lastPacketTime = millis();
  delay(500);
  Serial.println("READY");
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '<') {
      rxIndex = 0;
    } else if (c == '>') {
      rxBuffer[rxIndex] = '\0';
      packetReady = true;
    } else if (rxIndex < BUFFER_SIZE - 1) {
      rxBuffer[rxIndex++] = c;
    }
  }
}

void parsePacket() {
  int idx   = -1;
  int angle = -1;
  char* p   = rxBuffer;

  idx = atoi(p);
  p   = strchr(p, ','); if (!p) return; p++;
  angle = atoi(p);

  if (idx < 0 || idx >= NUM_SERVOS) return;
  if (angle < 0 || angle > 180)     return;

  lastPacketTime    = millis();
  currentAngle[idx] = angle;
  servos[idx].write(angle);

  Serial.print("MOV "); Serial.print(idx);
  Serial.print(" -> ");  Serial.println(angle);
}

void checkTimeout() {
  bool expired = (millis() - lastPacketTime > TIMEOUT_MS);

  if (expired && !timedOut) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      servos[i].write(ANGLE_OPEN[i]);
      currentAngle[i] = ANGLE_OPEN[i];
    }
    timedOut = true;
    Serial.println("TIMEOUT: opening fingers");
  }
  if (!expired && timedOut) {
    timedOut = false;
    Serial.println("RECONNECTED");
  }
}

void loop() {
  readSerial();

  if (packetReady) {
    packetReady = false;
    parsePacket();
  }

  checkTimeout();
}
