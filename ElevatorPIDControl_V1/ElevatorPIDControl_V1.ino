// Rough Elevator Car Physics model with simple PID control loop

// Libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED display configuration
#define SCREEN_WIDTH 128  // Width in pixels
#define SCREEN_HEIGHT 64  // Height in pixels
#define OLED_RESET    -1  // Reset pin
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define GRAPH_WIDTH SCREEN_WIDTH  // One data point per horizontal pixel

// Buffer to hold past position values for graphing
float posBuffer[GRAPH_WIDTH];

// ----- Simulation Parameters ----- //
const float mass = 10.0;       // Elevator mass in kg
const float g = 9.81;          // Gravitational acceleration (m/s^2)
const float dt = 0.1;          // Time step for simulation (s)

// Motor and force constants
const float motorForceConst = 20.0;  // Motor force per volt (N/V)
const float maxVoltage = 12.0;       // Maximum motor voltage

// Friction and braking parameters
const float baselineFriction = 5.0;  // Baseline friction force (N)
const float brakingFriction = 40;    // Friction force when braking (N)
const float brakingZone = 1.5;       // Distance (m) within which braking is applied

// PID controller parameters (Needs tuning)
float Kp = 3;
float Ki = 0.5;
float Kd = 8;

// Integral anti-windup limits
const float integralMax = 50.0;
const float integralMin = -50.0;

// PID state variables
float error = 0.0;
float lastError = 0.0;
float integral = 0.0;
float derivative = 0.0;

// Elevator state variables (Start at ground floor 0m)
float position = 0.0;
float velocity = 0.0;
float acceleration = 0.0;

float targetPosition = 0.0;

// Time tracking variables
unsigned long lastTime = 0;
unsigned long simulationStartTime = 0;

void setup() {
  Serial.begin(9600);
  lastTime = millis();
  simulationStartTime = millis();
  
  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // I2C address 0x3C for 128x64
    Serial.println("SSD1306 allocation failed");
    while(true);
  }
  display.clearDisplay();
  display.display();
  
  // Initialize the graph buffer with the starting position
  for (int i = 0; i < GRAPH_WIDTH; i++) {
    posBuffer[i] = position;
  }
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= dt * 1000) {
    lastTime = currentTime;
    
    // Cycle through 3 positions, from 0 m to 6 m
    if (currentTime - simulationStartTime >= 2000) {
      targetPosition = 6.0;
    }

    // Change the target position from 6 m to 9m
    if (currentTime - simulationStartTime >= 10000) {
      targetPosition = 9;
    }

    // Change the target position from 9 m to 0m
    if (currentTime - simulationStartTime >= 20000) {
      targetPosition = 0;
    }
    
    // ---- PID Control Calculation ---- //
    error = targetPosition - position;
    
    // Update integral term and apply anti-windup limits
    integral += error * dt;
    if (integral > integralMax) integral = integralMax;
    if (integral < integralMin) integral = integralMin;
    
    derivative = (error - lastError) / dt;
    float pidOutput = Kp * error + Ki * integral + Kd * derivative;
    float voltageCommand = constrain(pidOutput, -maxVoltage, maxVoltage);
    lastError = error;
    
    // ---- Force Calculations ---- //
    float motorForce = motorForceConst * voltageCommand;
    float gravityForce = mass * g;
    
    // Use increased friction if within braking zone
    float currentFriction = (abs(targetPosition - position) < brakingZone) ? brakingFriction : baselineFriction;
    float frictionForce = 0;
    if (velocity > 0.01) {
      frictionForce = -currentFriction;
    } else if (velocity < -0.01) {
      frictionForce = currentFriction;
    }
    
    // Compute net force (upward positive)
    float netForce = motorForce + frictionForce - gravityForce;
    acceleration = netForce / mass;
    
    // Euler integration to update state
    velocity += acceleration * dt;
    position += velocity * dt;
    
    // ---- Debug Output ---- //
    Serial.print("Time: ");
    Serial.print((currentTime - simulationStartTime) / 1000.0);
    Serial.print(" s, Target: ");
    Serial.print(targetPosition);
    Serial.print(" m, Pos: ");
    Serial.print(position);
    Serial.print(" m, Vel: ");
    Serial.print(velocity);
    Serial.print(" m/s, Volt: ");
    Serial.print(voltageCommand);
    Serial.print(" V, Error: ");
    Serial.println(error);
    
    // ---- Update Graph Buffer ---- //
    // Shift the buffer left one pixel
    for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
      posBuffer[i] = posBuffer[i + 1];
    }
    // Append the current position to the rightmost pixel
    posBuffer[GRAPH_WIDTH - 1] = position;
    
    // ---- Draw Graph on OLED ---- //
    drawGraph();
  }
}

// Function to draw the position vs. time graph on the OLED display
void drawGraph() {
  display.clearDisplay();
  
  // Draw axes (horizontal axis at the bottom, vertical axis on the left)
  display.drawLine(0, SCREEN_HEIGHT - 1, SCREEN_WIDTH, SCREEN_HEIGHT - 1, SSD1306_WHITE);
  display.drawLine(0, 0, 0, SCREEN_HEIGHT, SSD1306_WHITE);
  
  // Scale position parameters
  float minVal = 0.0;
  float maxVal = 10.0;
  
  // Draw the position line graph from the buffer data
  for (int x = 0; x < GRAPH_WIDTH - 1; x++) {
    float pos1 = posBuffer[x];
    float pos2 = posBuffer[x + 1];
    // Map position to Y coordinate (inverting since display origin is top-left)
    int y1 = SCREEN_HEIGHT - 1 - (int)((pos1 - minVal) / (maxVal - minVal) * (SCREEN_HEIGHT - 1));
    int y2 = SCREEN_HEIGHT - 1 - (int)((pos2 - minVal) / (maxVal - minVal) * (SCREEN_HEIGHT - 1));
    display.drawLine(x, y1, x + 1, y2, SSD1306_WHITE);
  }
  
  // Draw a horizontal line indicating the target position
  int targetY = SCREEN_HEIGHT - 1 - (int)((targetPosition - minVal) / (maxVal - minVal) * (SCREEN_HEIGHT - 1));
  display.drawLine(0, targetY, SCREEN_WIDTH, targetY, SSD1306_WHITE);
  
  display.display();
}
