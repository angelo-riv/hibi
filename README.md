# Hibi — Autonomous Obstacle Avoidance Robot

Inspired by Chapter 5 of *Understanding Intelligence* by Rolf Pfeifer, Hibi is an autonomous mobile robot designed for obstacle avoidance using proximity and collision sensors.

Hibi integrates ultrasonic distance sensors and mechanical collision switches to perceive its environment and control two DC motors through an embedded microcontroller system. Its behavior is governed by a finite state machine that enables forward motion and avoidance maneuvers, while a neural-network–inspired control structure based on Hebbian learning strengthens associations between proximity sensing and avoidance actions. This allows the robot to gradually rely more on early proximity detection rather than collision events when navigating obstacles.

![Hibi Robot](IMG_5558.PNG)

---

## Hardware Requirements

### Sensing Subsystem
- (HC-SR04) 2x Ultrasonic distance sensors — early obstacle detection
- (SS-5GL2) 2x Mechanical limit switches — collision detection

### Control Subsystem
- (DFRduino Uno v3.0) 1x Arduino Uno microcontroller — sensor processing and decision making
- DFRobot Prototype Shield for Uno
- Embedded finite state machine with Hebbian-inspired control logic

### Actuation Subsystem
- 2x DC motors — robot locomotion
- (TB6612FNG) Motor driver shield — motor speed and direction control

### Power Subsystem
- Battery supply for the Arduino and motor driver

### User Interface
- 1x Push button — start/stop the robot
- 1x Onboard LED — system status indication

---

## How It Works

Hibi runs a finite state machine with four states: `FORWARD`, `AVOID_LEFT`, `AVOID_RIGHT`, and `STOP`.

On each loop iteration, sensor readings are mapped to a 4-element input vector:

| Input | Source | Fires when |
|-------|--------|------------|
| `IN_UL` | Left ultrasonic | Distance < 8 cm |
| `IN_UR` | Right ultrasonic | Distance < 8 cm |
| `IN_CL` | Left collision switch | Switch pressed |
| `IN_CR` | Right collision switch | Switch pressed |

Action scores are computed as a weighted sum of inputs across a `4×4` weight matrix. The highest-scoring action is selected and executed. When an avoidance action fires, the corresponding weights are incremented by the learning rate `η = 0.005` (Hebbian update), capped at 1.0.

Over time, the ultrasonic inputs accumulate weight toward avoidance actions, so the robot learns to react to proximity before physically colliding.

### Initial Weights
Collision inputs are seeded with non-zero weights to bootstrap avoidance behavior:

```
CL → AVOID_RIGHT: 0.7,  AVOID_LEFT: 0.1
CR → AVOID_LEFT:  0.7,  AVOID_RIGHT: 0.1
```

All ultrasonic weights start at 0.0 and grow through experience.

---

## Supplementary Materials

- [Infographic](https://drive.google.com/file/d/1Fx8aNa_9BHokyRSV4DMcMjhIMGPzt8rJ/view?usp=sharing)
- [Demo Video](https://drive.google.com/file/d/11S7KAj6Iqm0KiBqjt94TlFJgFTxz4p3S/view?usp=sharing)
- [Project Report (PDF)](MT2TA4_Project_Supplementary/hibiMLRobot.pdf)

---

## Datasheets & Resources

- [Limit Switch SS-5GL2](https://omronfs.omron.com/en_US/ecb/products/pdf/en-ss.pdf) — Actuator: Hinge Roller Lever, Contact Form: SPDT
- [Ultrasonic Sensor HC-SR04](https://handsontec.com/dataspecs/sensor/SR-04-Ultrasonic.pdf)
- [Motor Driver TB6612FNG](https://cdn-shop.adafruit.com/datasheets/TB6612FNG_datasheet_en_20121101.pdf)
- [DFRobot Prototype Shield for Uno](https://drive.google.com/file/d/1QQJZuXcxKEsXqXhCjPRjp93PmYUfy-Mz/view?usp=sharing)
- [Arduino Uno Pinout](https://docs.arduino.cc/resources/pinouts/A000066-full-pinout.pdf)
- [Understanding Intelligence — Rolf Pfeifer, Ch. 5 (pg. 139/160)](https://drive.google.com/file/d/1d1wppGQiDiBHhIOcZgl9_t_G0V4phKvB/view?usp=sharing)
