# Voom-Bot: Autonomous Indoor Cleaning Robot

## Overview
Voom-Bot is an autonomous indoor cleaning robot built in Webots using the Pioneer 3-DX platform. It is designed to navigate indoor environments, avoid obstacles, and clean areas without human control.

The robot uses a finite state machine (FSM) to manage behaviour and switches between exploration, cleaning, avoidance, and safety states depending on sensor input and system conditions.

---

## Features
- Autonomous indoor navigation and exploration  
- Obstacle avoidance using proximity sensors  
- Cliff detection for safety  
- Grid-based coverage of the environment  
- Automatic return to charging dock  
- Battery monitoring and charging behaviour  
- Stuck detection and recovery mode  
- LED and speaker feedback for system status  

---

## Inputs (Sensors)
- 8 × Proximity sensors  
- Cliff sensors  
- GPS  
- Compass  
- Keyboard input (manual override)  

---

## Outputs (Actuators)
- Differential drive motors  
- LED indicators  
- Speaker output  

---

## Behaviour System
The robot is controlled using a Finite State Machine (FSM) with the following states:

- PAUSED  
- EXPLORE  
- CLEANING  
- AVOID  
- CLIFF  
- RECOVERY  
- CHARGING  

The system continuously switches between states based on sensor readings, battery level, and environment conditions.

---

## How It Works
1. The robot explores the environment autonomously  
2. It detects obstacles and avoids collisions  
3. It builds a simple grid-based map of visited areas  
4. It switches to cleaning mode when appropriate  
5. It returns to the dock when battery is low  
6. Safety systems handle cliffs and stuck situations  

---

## Team
- Lucas El-Harrif  
- Michelle Murutsi  
- Tashinga Blessed Nyamadzawo  
