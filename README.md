# 🛠️ Hardware Prototype – Elevate_PLC_Control

## 🚧 Arduino-Based Elevator Physics Model

This branch documents the **hardware prototype** of the **Elevate PLC Control System**, focusing on the **Arduino-based implementation** of a simplified **elevator car physics model**. It simulates motion control using **real-time physics**, featuring a **PID control loop**, basic force modeling, and a visual user interface via an **OLED display**.

---

## 🔧 Prototype Highlights

- 🎛️ **Arduino Simulation** of a single elevator shaft  
- ⚙️ **PID Control Loop** to manage position, velocity, and motor output  
- 🧮 **Force Model**: Incorporates gravity, motor force, and friction using `F = ma`  
- 📊 **Real-Time Visualization** with **OLED Display**
- 🟢 **Status LEDs** to indicate floor requests and system states  

---

## 📦 Hardware Components

- **Arduino Uno / Nano**  
- **OLED Display** (e.g., SSD1306) 

---

## 🧠 Control Logic

### PID Control

- Simulates elevator movement 
- Uses **position error** to compute motor voltage  
- Graphs:
  - Motor voltage output
  - Elevator position
  - Velocity response

### Elevator Physics

- Discrete time simulation using:
  - `F = ma`
  - Net force = motor force – gravity – friction
  - Position and velocity update at fixed timestep

---

## 🖥️ Interface

- **OLED Screen**:
  - Visualizes motor voltage, position, and velocity in real time  


---

## ✅ Prototype Goals

- [x] Model simple elevator movement logic  
- [x] Implement and tune PID controller  
- [x] Simulate and display physics-based motion  
- [x] Provide interactive floor selection  

---

## 🧑‍💻 Contributors (Hardware Team)

- Joshua Gonzalez *(Physics Modeling & Display Logic)*  
- Miriam Maher *(OpenPLC & PID Control)*  

Project branch of: **Elevate PLC Control System**  
Main branch: [Hospital Elevator Control System](https://github.com/yourrepo/Elevate_PLC_Control)  
Product Owner: **Quang Ha**
