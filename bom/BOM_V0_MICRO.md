# Project Eagle-Eye v0 Micro: Twin-Coreless Flying Wing Build Guide & BOM

## 1. Overview & Concept
**Eagle-Eye v0 Micro** is an ultra-lightweight (32g All-Up Weight), 100% 3D-printable test airframe designed for rapid indoor/backyard prototyping before flying the full 920mm Coroplast wing.

### Core Propulsion & Control Innovations
- **Propulsion:** Twin 7×16mm (or 8.5×20mm) high-RPM coreless brushed DC motors mounted directly into streamlined nacelles in the left and right wings ($y = \pm 46\text{ mm}$).
- **Differential Thrust Steering:** Eliminates heavy servos and mechanical linkages completely!
  - Yaw & Roll steering are achieved purely by modulating motor RPM differential.
  - Reflexed S-camber airfoil provides pitch autostability without a horizontal tail.
- **Silent 20 kHz Ultrasonic PWM:** Driven via dual N-channel MOSFETs directly from the ESP32 / ESP32-C3 LEDC peripheral.
- **Budget:** Total cost to build is **under ₹500 INR**.

---

## 2. Electrical Schematic & Motor Drive Circuit

```
            +3.7V LiPo (+) -------------------------+--------------------+
                                                   |                    |
                                              [Left Motor]         [Right Motor]
                                                   |                    |
                                                   +---|<|---+          +---|<|---+
                                                   |  1N4148 |          |  1N4148 |
                                                   | (Diode) |          | (Diode) |
                                                   |         |          |         |
                                                Drain                 Drain
                                            +-----------+         +-----------+
   ESP32 GPIO 13 (Left PWM) ---[ 100R ]---> | Gate      |         | Gate      | <---[ 100R ]--- ESP32 GPIO 12 (Right PWM)
                                            |  AO3400   |         |  AO3400   |
                                +---[10k]---|  MOSFET   |         |  MOSFET   |---[10k]---+
                                |           +-----------+         +-----------+           |
                                |              Source                Source               |
                                |                 |                     |                 |
   Common Ground (GND) ---------+-----------------+---------------------+-----------------+
```

### Pinout Mapping (ESP32-WROOM-32 or ESP32-C3 SuperMini)
- **Left Coreless Motor (PWM Channel 0):** `GPIO 13` (or `GPIO 4` on ESP32-C3)
- **Right Coreless Motor (PWM Channel 1):** `GPIO 12` (or `GPIO 5` on ESP32-C3)
- **MPU6050 (I2C):** `SDA = GPIO 21`, `SCL = GPIO 22`
- **1S LiPo Battery ADC:** `GPIO 34` (or `GPIO 0` on C3) via 10k/10k divider
- **Power Supply:** Single 1S 3.7V 300mAh LiPo connects directly to 3.3V LDO input and motor positive rail.

---

## 3. Bill of Materials & Verified Indian Pricing

| Component | Technical Specification | Vendor | Price (INR) |
|---|---|---|---|
| **Motors** | 7×16mm Coreless Motors (1 CW + 1 CCW Pair, 50,000 RPM) | [QuartzComponents](https://quartzcomponents.com/products/coreless-motor-716) | ₹135.00 |
| **Propellers** | 55mm Micro Propellers (1 CW + 1 CCW Pair, 0.8mm bore) | [Robu.in](https://robu.in/product/55mm-propeller-blade-for-716-coreless-motor/) | ₹45.00 |
| **MOSFETs** | 2× AO3400 / SI2302 N-Channel MOSFETs (SOT-23, 30V 5A) | QuartzComponents | ₹20.00 |
| **Diodes & Resistors** | 2× 1N4148 fast flyback diodes + 10kΩ gate pulldowns | QuartzComponents | ₹10.00 |
| **Battery** | 1S 3.7V 300mAh 30C LiPo (Whoop cell, BT2.0 / PH2.0) | Robu.in / Local | ₹185.00 |
| **Micro MCU** | ESP32-C3 SuperMini or ESP32-WROOM DevKit | Existing Inventory | ₹90.00 |
| **TOTAL** | | | **₹485.00 INR** |

---

## 4. 3D Printing Instructions for v0 Micro
- **File:** [`cad_v0_micro/micro_flying_wing_v0.stl`](file:///dev/shm/eagle-eye/cad_v0_micro/micro_flying_wing_v0.stl)
- **Material:** Standard PLA or PolyLite LW-PLA (Lightweight foaming PLA)
- **Nozzle:** 0.4 mm
- **Layer Height:** 0.16 mm – 0.20 mm
- **Infill:** 10% gyroid or single perimeter with zero infill on wing tips
- **Print Time:** ~1 hour 45 minutes on modern printers (Bambu/Ender)
- **Printed Airframe Weight:** ~15–18 grams
