// Rough Elevator Car Animation on Small Oled Display
// with button inputs and leds

// Libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Pin assignments
const int buttonPins[] = {4, 5, 6};   // Floor 1 to 3
const int ledPins[] = {A0, A1, A2};   // Active-low LEDs
bool floorRequests[] = {false, false, false};
bool lastButtonStates[] = {false, false, false};

int currentFloor = 0;  // 0 = Floor 1/Ground
int floorHeights[] = {48, 30, 12};  // Y positions for floors
int elevatorY = 48;

int direction = 0; // -1 = down, 1 = up, 0 = idle

// Timing
unsigned long lastMoveTime = 0;
const int moveInterval = 50;

// Doors
bool doorsOpen = false;
unsigned long doorTimer = 0;
const int doorDuration = 1500;

// Gear animation
int gearAngle = 0;

// ==================== Loading Screen ====================
void showSplashScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(10, 10);
  display.println("Elevate");
  display.setCursor(10, 30);
  display.println("PID Demo");
  display.display();

  int barX = 10;
  int barY = 55;
  int barW = 108;
  int barH = 6;
  display.drawRect(barX, barY, barW, barH, WHITE);

  int barSteps = 15;
  int stepWidth = (barW - 2) / barSteps;

  for (int i = 0; i <= barSteps; i++) {
    int w = i * stepWidth;
    display.fillRect(barX + 1, barY + 1, w, barH - 2, WHITE);
    display.display();
    delay(45);
  }
  delay(100); // total ~1 second
}

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }

  showSplashScreen();

  for (int i = 0; i < 3; i++) {
    pinMode(buttonPins[i], INPUT);
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], HIGH); // LEDs off
  }

  elevatorY = floorHeights[currentFloor];
}

// ==================== Loop ====================
void loop() {
  // Handle button presses
  for (int i = 0; i < 3; i++) {
    bool state = digitalRead(buttonPins[i]);
    if (state && !lastButtonStates[i]) {
      floorRequests[i] = true;
      digitalWrite(ledPins[i], LOW);

      // Immediate open if already there
      if (i == currentFloor && !doorsOpen && direction == 0) {
        floorRequests[i] = false;
        digitalWrite(ledPins[i], HIGH);
        doorsOpen = true;
        doorTimer = millis();
      }
    }
    lastButtonStates[i] = state;
  }

  // Elevator movement
  if (!doorsOpen) {
    if (direction == 0) {
      direction = getInitialDirection();
    }

    if (direction != 0 && millis() - lastMoveTime > moveInterval) {
      int nextY = elevatorY + (direction * -1); // up = decrease Y
      elevatorY = constrain(nextY, floorHeights[2], floorHeights[0]);

      lastMoveTime = millis();

      // Check if aligned with floor
      for (int i = 0; i < 3; i++) {
        if (elevatorY == floorHeights[i]) {
          currentFloor = i;

          if (floorRequests[i]) {
            floorRequests[i] = false;
            digitalWrite(ledPins[i], HIGH);
            doorsOpen = true;
            doorTimer = millis();
          }

          if (!hasRequestsInDirection(direction)) {
            direction = hasRequestsInDirection(-direction) ? -direction : 0;
          }
        }
      }
    }
  }

  // Handle doors
  if (doorsOpen && millis() - doorTimer > doorDuration) {
    doorsOpen = false;
  }

  // Rotate gear based on movement
  if (direction == 1) gearAngle -= 10; // Up = CCW
  else if (direction == -1) gearAngle += 10; // Down = CW
  gearAngle = (gearAngle + 360) % 360;

  drawElevator();
}

// ==================== Elevator Direction Logic ====================
int getInitialDirection() {
  for (int i = currentFloor + 1; i <= 2; i++) {
    if (floorRequests[i]) return 1;
  }
  for (int i = currentFloor - 1; i >= 0; i--) {
    if (floorRequests[i]) return -1;
  }
  return 0;
}

bool hasRequestsInDirection(int dir) {
  if (dir == 1) {
    for (int i = currentFloor + 1; i <= 2; i++) {
      if (floorRequests[i]) return true;
    }
  } else if (dir == -1) {
    for (int i = currentFloor - 1; i >= 0; i--) {
      if (floorRequests[i]) return true;
    }
  }
  return false;
}

// ==================== Gear Drawing ====================
void drawGear(int centerX, int centerY, int radius, int angleDeg) {
  float angle = radians(angleDeg);
  float spokeLen = radius - 2;
  float toothLen = 3;

  display.drawCircle(centerX, centerY, 2, WHITE);

  for (int i = 0; i < 6; i++) {
    float a = angle + i * PI / 3;
    int x0 = centerX + cos(a) * 1;
    int y0 = centerY + sin(a) * 1;
    int x1 = centerX + cos(a) * spokeLen;
    int y1 = centerY + sin(a) * spokeLen;
    display.drawLine(x0, y0, x1, y1, WHITE);

    int xt1 = centerX + cos(a) * (spokeLen + 1);
    int yt1 = centerY + sin(a) * (spokeLen + 1);
    int xt2 = centerX + cos(a) * (spokeLen + 1 + toothLen);
    int yt2 = centerY + sin(a) * (spokeLen + 1 + toothLen);
    display.drawLine(xt1, yt1, xt2, yt2, WHITE);
  }
}

// ==================== OLED Drawing ====================
void drawElevator() {
  display.clearDisplay();

  // Shaft
  int shaftX = 50;
  int shaftWidth = 28;
  display.drawRect(shaftX, 0, shaftWidth, SCREEN_HEIGHT, WHITE);

  // Floor labels
  display.setTextSize(1);
  display.setTextColor(WHITE);
  for (int i = 0; i < 3; i++) {
    display.setCursor(5, floorHeights[i] + 2);
    display.print("Floor ");
    display.print(i + 1);
  }

  // Elevator car
  int carHeight = 14;
  int carWidth = shaftWidth - 4;
  int carX = shaftX + 2;

  if (doorsOpen) {
    display.drawRect(carX, elevatorY, carWidth / 2 - 1, carHeight, WHITE);
    display.drawRect(carX + carWidth / 2 + 1, elevatorY, carWidth / 2 - 1, carHeight, WHITE);
  } else {
    display.fillRect(carX, elevatorY, carWidth, carHeight, WHITE);
  }

  // Floor display
  display.setCursor(shaftX + shaftWidth + 10, 2);
  display.setTextSize(1);
  display.print("Now: ");
  display.print(currentFloor + 1);

  // Motor gear bottom-right
  drawGear(SCREEN_WIDTH - 10, SCREEN_HEIGHT - 10, 7, gearAngle);

  display.display();
}
