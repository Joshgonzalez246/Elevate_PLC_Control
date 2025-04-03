//***************************************************************************************
// Simple Elevator Physics Model with PID Control Loop
// Version 4
//***************************************************************************************

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define GRAPH_WIDTH SCREEN_WIDTH
float posBuffer[GRAPH_WIDTH];

// Simulation parameters
const float mass = 10.0;
const float g = 9.81;
const float dt = 0.1;

// Motor
const float motorForceConst = 20.0;
const float maxVoltage = 12.0;
float voltageCommand = 0;

// Friction
const float baselineFriction = 5.0;
const float brakingFriction = 30.0;
const float descentBrakingFriction = 80.0;
const float brakingZone = 1.5;

// PID
float Kp = 2;
float Ki = 0.8;
float Kd = 4;

const float integralMax = 50.0;
const float integralMin = -50.0;

float error = 0.0, lastError = 0.0, integral = 0.0, derivative = 0.0;
bool pidEnabled = true;

// Elevator state
float position = 0.0;
float velocity = 0.0;
float acceleration = 0.0;
float targetPosition = 0.0;

unsigned long lastTime = 0;
unsigned long simulationStartTime = 0;

// FSM
enum ElevatorState {
  TO_6M, DOOR_OPENING_6M, DOOR_WAIT_6M,
  TO_9M, DOOR_OPENING_9M, DOOR_WAIT_9M,
  TO_0M, DOOR_OPENING_0M, DOOR_WAIT_0M
};

ElevatorState state = TO_6M;
unsigned long doorOpenStartTime = 0;
const unsigned long doorDelay = 2000;

const float positionTolerance = 0.05;
const float velocityTolerance = 0.05;
bool brakeEngaged = false;

enum LastFloor { FLOOR_9, FLOOR_0 };
LastFloor lastFloor = FLOOR_0;

void setup() {
  Serial.begin(9600);
  lastTime = millis();
  simulationStartTime = millis();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    Serial.println("SSD1306 allocation failed");
    while (true);
  }
  display.clearDisplay();
  display.display();

  for (int i = 0; i < GRAPH_WIDTH; i++) {
    posBuffer[i] = position;
  }
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= dt * 1000) {
    lastTime = currentTime;

    // --- FSM ---
    switch (state) {
      case TO_6M:
        pidEnabled = true;
        targetPosition = 6.0;
        if ((abs(position - targetPosition) < positionTolerance && abs(velocity) < velocityTolerance) ||
            (velocity < 0 && position < targetPosition)) {
          brakeEngaged = true;
          velocity = 0;
          acceleration = 0;
          pidEnabled = false;
          error = 0; integral = 0; derivative = 0; lastError = 0;
          doorOpenStartTime = currentTime;
          state = DOOR_OPENING_6M;
        }
        break;

      case DOOR_OPENING_6M:
        brakeEngaged = true;
        velocity = 0;
        acceleration = 0;
        error = 0; integral = 0; derivative = 0; lastError = 0;
        if (currentTime - doorOpenStartTime >= doorDelay) {
          state = DOOR_WAIT_6M;
        }
        break;

      case DOOR_WAIT_6M:
        if (lastFloor == FLOOR_0) {
          state = TO_9M;
          lastFloor = FLOOR_9;
        } else {
          state = TO_0M;
          lastFloor = FLOOR_0;
        }
        break;

      case TO_9M:
        pidEnabled = true;
        targetPosition = 9.0;
        if ((abs(position - targetPosition) < positionTolerance && abs(velocity) < velocityTolerance) ||
            (velocity < 0 && position < targetPosition)) {
          brakeEngaged = true;
          velocity = 0;
          acceleration = 0;
          pidEnabled = false;
          error = 0; integral = 0; derivative = 0; lastError = 0;
          doorOpenStartTime = currentTime;
          state = DOOR_OPENING_9M;
        }
        break;

      case DOOR_OPENING_9M:
        brakeEngaged = true;
        velocity = 0;
        acceleration = 0;
        error = 0; integral = 0; derivative = 0; lastError = 0;
        if (currentTime - doorOpenStartTime >= doorDelay) {
          state = DOOR_WAIT_9M;
        }
        break;

      case DOOR_WAIT_9M:
        state = TO_6M;
        break;

      case TO_0M:
        pidEnabled = true;
        targetPosition = 0.0;
        if ((abs(position - targetPosition) < positionTolerance && abs(velocity) < velocityTolerance) ||
            (velocity < 0 && position < targetPosition) ||
            (position <= 0.0)) {
          brakeEngaged = true;
          position = 0.0;
          velocity = 0;
          acceleration = 0;
          pidEnabled = false;
          error = 0; integral = 0; derivative = 0; lastError = 0;
          doorOpenStartTime = currentTime;
          state = DOOR_OPENING_0M;
        }
        break;

      case DOOR_OPENING_0M:
        brakeEngaged = true;
        velocity = 0;
        acceleration = 0;
        error = 0; integral = 0; derivative = 0; lastError = 0;
        if (currentTime - doorOpenStartTime >= doorDelay) {
          state = DOOR_WAIT_0M;
        }
        break;

      case DOOR_WAIT_0M:
        state = TO_6M;
        break;
    }

    if (abs(targetPosition - position) > positionTolerance) {
      brakeEngaged = false;
    }

    float motorForce = 0;
    voltageCommand = 0;

    if (!brakeEngaged && pidEnabled) {
      error = targetPosition - position;
      integral += error * dt;
      integral = constrain(integral, integralMin, integralMax);
      derivative = (error - lastError) / dt;
      float pidOutput = Kp * error + Ki * integral + Kd * derivative;

      if (abs(error) < positionTolerance && abs(velocity) < 0.1) {
        pidOutput = 0;
      }

      if (targetPosition < position && velocity < -0.5) {
        pidOutput -= 2.0 * velocity;
      }

      voltageCommand = constrain(pidOutput, -maxVoltage, maxVoltage);
      lastError = error;

      motorForce = motorForceConst * voltageCommand;

      if (abs(velocity) < 0.2 && abs(error) < 1.0) {
        motorForce += mass * g * 0.7;
      }

    } else {
      voltageCommand = 0;
      motorForce = 0;
    }

    float gravityForce = mass * g;

    float currentFriction = baselineFriction;
    if (velocity < -0.1 && position > targetPosition) {
      currentFriction = descentBrakingFriction;
    } else if (abs(targetPosition - position) < brakingZone) {
      currentFriction = brakingFriction;
    }

    float frictionForce = -currentFriction * velocity;
    float brakeAssist = 0.0;
    if (velocity < -0.1 && position > targetPosition) {
      brakeAssist = 70.0;
    }

    float netForce = motorForce + frictionForce + brakeAssist - gravityForce;

    if (!brakeEngaged) {
      acceleration = netForce / mass;
      velocity += acceleration * dt;
      position += velocity * dt;
    } else {
      acceleration = 0;
      velocity = 0;
    }

    if (position < 0.0) {
      position = 0.0;
      velocity = 0.0;
      acceleration = 0.0;
      brakeEngaged = true;
    }

    for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
      posBuffer[i] = posBuffer[i + 1];
    }
    posBuffer[GRAPH_WIDTH - 1] = position;

    display.clearDisplay();
    drawGraph();

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 50);
    display.print("STATE: ");
    if (state == TO_6M || state == DOOR_OPENING_6M || state == DOOR_WAIT_6M) display.print("6M");
    else if (state == TO_9M || state == DOOR_OPENING_9M || state == DOOR_WAIT_9M) display.print("9M");
    else if (state == TO_0M || state == DOOR_OPENING_0M || state == DOOR_WAIT_0M) display.print("0M");

    display.setCursor(0, 58);
    display.print("P:");
    display.print(position, 1);
    display.print(" V:");
    display.print(velocity, 1);
    display.print(" U:");
    display.print(voltageCommand, 1);
    display.print(" ");
    display.print(brakeEngaged ? "B:ON" : "B:OFF");

    display.display();
  }
}

void drawGraph() {
  for (int x = 0; x < GRAPH_WIDTH - 1; x++) {
    float pos1 = posBuffer[x];
    float pos2 = posBuffer[x + 1];

    int y1 = SCREEN_HEIGHT - 17 - (int)((pos1 / 10.0) * 48);
    int y2 = SCREEN_HEIGHT - 17 - (int)((pos2 / 10.0) * 48);

    display.drawLine(x, y1, x + 1, y2, SSD1306_WHITE);
  }

  int targetY = SCREEN_HEIGHT - 17 - (int)((targetPosition / 10.0) * 48);
  display.drawLine(0, targetY, SCREEN_WIDTH, targetY, SSD1306_WHITE);
}