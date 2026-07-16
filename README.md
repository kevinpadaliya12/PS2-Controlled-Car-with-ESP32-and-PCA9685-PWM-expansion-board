# PS2-Controlled-Car-with-ESP32-and-PCA9685-PWM-expansion-board


An ESP32-based wireless robot car controlled via a PS2 controller, using a PCA9685 expansion board over I2C for smooth PWM motor control with R-Theta (polar) drive mixing.

## Features

- Wireless PS2 controller support (DualShock / Wireless DualShock)
- Smooth R-Theta (polar coordinate) joystick mixing for natural driving feel
- D-pad for precise full-speed directional control
- PCA9685 expansion board for 12-bit PWM motor control over I2C
- Controller disconnect detection — motors stop automatically on signal loss
- I2C bus recovery to handle communication faults
- Dead zone filtering to eliminate joystick center drift

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | ESP32 |
| Motor Driver Expansion | PCA9685-based expansion board (I2C address `0x40`) |
| Controller | PS2 Wireless DualShock receiver |
| Motors | 2× DC motors (Left and Right) |

### Pin Connections

#### PS2 Receiver

| PS2 Wire | ESP32 Pin | Color |
|---|---|---|
| DAT (MISO) | GPIO 22 | White |
| CMD (MOSI) | GPIO 23 | Grey |
| SEL (CS) | GPIO 26 | Green |
| CLK | GPIO 33 | Blue |
| VCC | 3.3V | — |
| GND | GND | — |

#### Expansion Board (I2C)

| Signal | ESP32 Pin |
|---|---|
| SDA | GPIO 18 |
| SCL | GPIO 19 |
| VCC | 5V |
| GND | GND |

#### PCA9685 Motor Channels

| Motor | IN1 Channel | IN2 Channel |
|---|---|---|
| Left motor | CH 12 | CH 13 |
| Right motor | CH 14 | CH 15 |

---

## Software Dependencies

Install these libraries via Arduino IDE Library Manager:

| Library | Purpose |
|---|---|
| `PS2X_lib` | PS2 controller communication |
| `Wire` (built-in) | I2C communication |

---

## How It Works

### R-Theta Drive Mixing

Instead of simple tank-style control, the joystick axes are converted to polar coordinates:

- **r** (magnitude) — how far the stick is pushed → overall speed
- **θ** (angle) — which direction → turn ratio between motors

The motor mixing formula rotates the joystick frame by 45°:

```
Left  motor = r × sin(θ + 45°)
Right motor = r × sin(θ − 45°)
```

This gives smooth, proportional control — a gentle push gives slow speed, a full push gives full speed, and diagonal pushes give natural arcing turns.

### Control Map

| Input | Action |
|---|---|
| Left joystick | Smooth proportional drive (R-Theta mixed) |
| D-pad Up | Forward (full speed) |
| D-pad Down | Reverse (full speed) |
| D-pad Left | Left turn (full speed) |
| D-pad Right | Right turn (full speed) |
| SELECT | Stop motors |

### Disconnect Safety

The PS2 wireless receiver outputs `LX=255, LY=255` when signal is lost. The firmware detects this and immediately stops all motors rather than acting on the garbage values.

---

## Project Structure

```
esp32-ps2-robot/
├── esp32-ps2-robot.ino     # Main firmware
├── expansion.py            # Original MicroPython PCA9685 library (reference)
└── README.md
```

---

## Setup & Flash

1. Install [Arduino IDE](https://www.arduino.cc/en/software) with ESP32 board support
   - Add this URL in *File → Preferences → Additional Board Manager URLs*:
     `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Install **esp32** by Espressif from *Tools → Board → Board Manager*

2. Install the `PS2X_lib` library via *Sketch → Include Library → Manage Libraries*

3. Select board: **Tools → Board → ESP32 Dev Module**

4. Connect ESP32 via USB and select the correct COM port

5. Upload the sketch

6. Open Serial Monitor at **115200 baud** to see debug output

---

## Serial Monitor Output

On successful boot you will see:

```
=== Boot ===
PCA9685 found at 0x40
Expansion Board ready.
PS2 try #1 err=0
Wireless DualShock found
=== Ready ===
```

During operation:

```
LX=128 LY=90 L=2100 R=2100     ← joystick forward
FWD                              ← D-pad up
CONTROLLER DISCONNECTED — motors stopped   ← signal lost
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Motors spin on boot | PCA9685 not initialized | Check SDA=18 SCL=19 wiring |
| `PCA9685 NOT found!` | I2C wiring issue | Verify SDA/SCL connections and 5V power to expansion board |
| Motors spin when controller disconnects | Missing disconnect guard | Ensure `isControllerConnected()` check is present |
| Left/Right turns reversed | Motor wiring polarity | Swap `true`/`false` in D-pad LEFT/RIGHT motor calls |
| `PS2 try #N err=3` loop | Controller not paired | Turn on PS2 receiver before powering ESP32 |
| Joystick drift at center | Dead zone too small | Increase `DEAD_ZONE` define (default 20) |

---

## Key Defines to Tune

```cpp
#define DEAD_ZONE  20    // Joystick center dead zone (increase if drifting)
#define MIN_PWM   200    // Minimum PWM to send to motors (below this = stop)
```

---

## License

MIT License — free to use, modify, and distribute.

---

## Author

KP — B.Tech Electronics and Communication Engineering, Nirma University  
Embedded Firmware Engineer
