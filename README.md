# 🧾 PLC Development – Elevate_PLC_Control

## 🤖 OpenPLC Integration & PID Communication

This branch focuses on developing the **PLC-side logic** for the Elevate control system using **OpenPLC**. While the **elevator physics simulation remains on Arduino**, this stage handles **PID tuning**, **state control**, and **communication protocols** between the PLC and the microcontroller. The goal is to replicate high-level control functionality using **Structured Text (ST)** and **Ladder Logic (LD)** under IEC 61131-3 standards.

---

## 🎯 Objectives

- ⚙️ Implement **PID control logic** in Structured Text
- 🔄 Set up **serial/MODBUS communication** between PLC and Arduino
- 📤 Send motor control commands from PLC to Arduino

---

## 💻 Tools & Technologies

- **OpenPLC Runtime & Editor**
- **Structured Text (ST)** and **Ladder Logic (LD)**
- **Serial / Modbus RTU** communication protocol
- **Arduino Uno/Nano** as the physics simulation target
- **Tag-based I/O mapping** for OpenPLC interaction

---

## 📌 PLC Implementation Features

- 🔄 **Serial Communication**: Read/write commands to Arduino for position control
- 📈 **PID Logic**: Replicates or tunes control loop from Arduino using ST blocks

---

## 🔄 System Architecture

```plaintext
┌────────────┐     Serial/Bus        ┌───────────────┐
│  OpenPLC   │  ⇄  Communication ⇄  │   Arduino     │
│   (PID)    │                       │ (Physics Sim) │
└────────────┘                       └───────────────┘
