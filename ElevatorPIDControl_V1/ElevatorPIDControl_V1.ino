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

// Elevator state
float position = 0.0;
float velocity = 0.0;
float acceleration = 0.0;
float targetPosition = 0.0;

unsigned long lastTime = 0;
unsigned long simulationStartTime = 0;

// FSM
enum ElevatorState { TO_6M, TO_9M, TO_0M, IDLE };
ElevatorState state = TO_6M;

const float positionTolerance = 0.05;
const float velocityTolerance = 1.0;
bool brakeEngaged = false;

void setup() {
  Serial.begin(9600);
  lastTime = millis();
  simulationStartTime = millis();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
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
        targetPosition = 6.0;
        if (abs(position - targetPosition) < positionTolerance && abs(velocity) < velocityTolerance) {
          brakeEngaged = true;
          velocity = 0;
          acceleration = 0;
          state = TO_9M;
        }
        break;
      case TO_9M:
        targetPosition = 9.0;
        if (abs(position - targetPosition) < positionTolerance && abs(velocity) < velocityTolerance) {
          brakeEngaged = true;
          velocity = 0;
          acceleration = 0;
          state = TO_0M;
        }
        break;
      case TO_0M:
        targetPosition = 0.0;
        if (abs(position - targetPosition) < positionTolerance && abs(velocity) < velocityTolerance) {
          brakeEngaged = true;
          velocity = 0;
          acceleration = 0;
          state = IDLE;
        }
        break;
      case IDLE:
        targetPosition = position;
        brakeEngaged = true;
        velocity = 0;
        acceleration = 0;
        break;
    }

    if (abs(targetPosition - position) > positionTolerance) {
      brakeEngaged = false;
    }

    float motorForce = 0;
    float voltageCommand = 0;

    if (!brakeEngaged) {
      // PID Control
      error = targetPosition - position;
      integral += error * dt;
      integral = constrain(integral, integralMin, integralMax);
      derivative = (error - lastError) / dt;
      float pidOutput = Kp * error + Ki * integral + Kd * derivative;

      if (abs(error) < positionTolerance && abs(velocity) < 0.1) {
        pidOutput = 0;
      }

      // Apply additional damping if descending
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
      error = 0;
      integral = 0;
      derivative = 0;
    }

    float gravityForce = mass * g;

    // --- Friction ---
    float currentFriction = baselineFriction;
    if (velocity < -0.1 && position > targetPosition) {
      currentFriction = descentBrakingFriction;
    } else if (abs(targetPosition - position) < brakingZone) {
      currentFriction = brakingFriction;
    }

    float frictionForce = -currentFriction * velocity;

    // --- Active Brake Assist ---
    float brakeAssist = 0.0;
    if (velocity < -0.1 && position > targetPosition) {
      brakeAssist = 70.0;  // Try increasing if needed
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

    // --- Serial Debug ---
    Serial.print("Time: ");
    Serial.print((currentTime - simulationStartTime) / 1000.0);
    Serial.print(" s, State: ");
    switch (state) {
      case TO_6M: Serial.print("TO_6M"); break;
      case TO_9M: Serial.print("TO_9M"); break;
      case TO_0M: Serial.print("TO_0M"); break;
      case IDLE: Serial.print("IDLE"); break;
    }
    Serial.print(", Brake: ");
    Serial.print(brakeEngaged ? "ON" : "OFF");
    Serial.print(", Target: ");
    Serial.print(targetPosition);
    Serial.print(" m, Pos: ");
    Serial.print(position);
    Serial.print(" m, Vel: ");
    Serial.print(velocity);
    Serial.print(" m/s, Volt: ");
    Serial.print(voltageCommand);
    Serial.print(" V | MotorF: ");
    Serial.print(motorForce);
    Serial.print(" N | FricF: ");
    Serial.print(frictionForce);
    Serial.print(" N | BrakeA: ");
    Serial.print(brakeAssist);
    Serial.print(" N | Gravity: ");
    Serial.println(gravityForce);

    // --- Graph Update ---
    for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
      posBuffer[i] = posBuffer[i + 1];
    }
    posBuffer[GRAPH_WIDTH - 1] = position;

    // --- OLED Display ---
    display.clearDisplay();
    drawGraph();

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 50);
    display.print("STATE: ");
    switch (state) {
      case TO_6M: display.print("TO_6M"); break;
      case TO_9M: display.print("TO_9M"); break;
      case TO_0M: display.print("TO_0M"); break;
      case IDLE:  display.print("IDLE "); break;
    }

    display.setCursor(0, 58);
    display.print("P:");
    display.print(position, 1);
    display.print(" V:");
    display.print(velocity, 1);
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
