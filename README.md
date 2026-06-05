---

# Autonomous Plant-Watering Robot

A multi-microcontroller system that autonomously locates a plant pot, navigates to it, checks soil moisture, and stops when it arrives. Built as a university project.

---

## How it works

The system consists of three separate microcontrollers that communicate over WiFi on a shared local network. The main ESP32 acts as the brain — it runs a web server, receives data from the other two modules, and controls the robot's movement. All devices connect via mDNS, so the robot is always reachable at `http://wateringrobot.local` without hardcoded IP addresses.

---

## Modules

### 1. Main ESP32 — Robot controller

The central unit. Runs the navigation logic on core 1 while a dedicated FreeRTOS task on core 0 continuously polls the camera module for target angle data every 150 ms.

The robot operates as a state machine with four states:

| State | Behaviour |
|---|---|
| `SEARCH` | Spins left or right in short bursts, pausing between turns to wait for a camera response |
| `FOLLOWING` | Drives toward the detected pot using proportional steering — the further the target is from centre, the more the inner wheel is boosted |
| `WALL_AVOID` | Backs up for 500 ms, then turns right for 550 ms when an obstacle is detected that is not the target pot |
| `ARRIVED` | Stops and fine-tunes alignment using short 150 ms spin pulses until the pot is centred in frame |

Wall detection uses an HC-SR04 ultrasonic sensor mounted at the front. If an object appears within 18 cm, the robot checks whether the camera has seen the pot recently (within the last 5 seconds). If yes — it has arrived at the destination and switches to `STATE_ARRIVED`. If no — it treats it as an obstacle and executes the avoidance manoeuvre.

A live dashboard is served at the root URL, auto-refreshing every second and showing current state, distance, camera angle, soil moisture, and whether the pot was seen recently.

---

### 2. ESP32-CAM — Visual target detection

An AI Thinker ESP32-CAM module running a colour detection algorithm. It captures frames at QQVGA resolution (160×120) in RGB565 format and scans every 4th pixel for red objects.

A pixel is classified as red if its red channel value exceeds 12 and is greater than both green and blue channels. When more than 6 red pixels are found in a frame, the module calculates the horizontal centre of mass of all red pixels and converts it to an angle relative to the camera's field of view (65° by default):

```
angle = (centerX - 80) × (FOV/2) / 80
```

This angle — negative for left, positive for right, `404` for not found — is sent to the main ESP32 via HTTP GET every 200 ms. The main robot uses this value both for steering while following and for confirming arrival when the ultrasonic sensor detects a close object.

---

### 3. ESP-01S — Soil moisture sensor

A minimal ESP8266-based module that reads a digital soil moisture sensor on GPIO2 every 10 seconds and sends the result to the main ESP32 via HTTP GET:

```
http://wateringrobot.local/soil?status=1   // dry
http://wateringrobot.local/soil?status=0   // wet
```

`HIGH` on the pin means dry, `LOW` means wet. The result is displayed on the robot's web dashboard and can be used to confirm whether watering is needed upon arrival.

---

## System diagram

```
┌─────────────┐     HTTP /angle     ┌──────────────────┐
│  ESP32-CAM  │ ─────────────────▶  │                  │
│  Red object │                     │   Main ESP32     │
│  detection  │                     │   Robot + Web    │
└─────────────┘                     │   Server         │
                                    │                  │
┌─────────────┐     HTTP /soil      │                  │
│   ESP-01S   │ ─────────────────▶  │                  │
│   Soil      │                     └──────────────────┘
│   moisture  │
└─────────────┘
```

---

## Hardware

| Component | Role |
|---|---|
| ESP32 (main) | Navigation, state machine, web server |
| ESP32-CAM (AI Thinker) | Red object detection, angle calculation |
| ESP-01S | Soil moisture sensing |
| HC-SR04 | Obstacle and arrival detection |
| L298N | Dual H-bridge motor driver |
| 4× DC gear motors | Drive |
| Capacitive soil moisture sensor | Plant soil reading |

---

## Tech stack

- Arduino framework (ESP32 + ESP8266 core)
- FreeRTOS — dual-core task pinning on ESP32
- mDNS — zero-config device discovery on local network
- HTTP — inter-device communication
- RGB565 frame processing — on-device computer vision without any external library
