# Project Eagle-Eye: Comprehensive Airframe Build, Tuning & Flight Operations Guide

## 1. 920mm Swept Flying Wing Aerodynamic Specifications

### 1.1 Airframe Geometry & Planform
The Eagle-Eye airframe is a 920mm swept flying wing constructed from a single sheet of **5mm Sunpack / Coroplast Polypropylene Flute Board** (or 5mm Depron / Bio-Foam). Polypropylene flutes must run parallel to the leading edge for maximum crash resistance.

```
                     Apex (Nose)
                        / \
                       /   \
                      /     \
    920mm Wingspan   /       \   Sweep Angle: 28°
                    /  [POD]  \
                   /     |     \
                  /      v      \
                 /   CG: 172mm   \
                /_________________\
                [ELE]   [PROP]   [ELE]
               <-40mm->  8045   <-40mm->
```

- **Wingspan ($b$):** 920 mm (36.2 in)
- **Root Chord ($c_r$):** 240 mm
- **Tip Chord ($c_t$):** 140 mm
- **Leading Edge Sweep Angle ($\Lambda$):** 28.0°
- **Wing Area ($S$):** $0.1748\text{ m}^2$ ($271\text{ in}^2$)
- **Aspect Ratio ($AR$):** $4.84$
- **All-Up Weight (AUW with 3S 18650):** 465 g
- **Wing Loading:** $26.6\text{ g/dm}^2$ ($8.7\text{ oz/ft}^2$) — optimal for low-speed 30–60 km/h cruising.
- **Center of Gravity (CG):** **$172\text{ mm} \pm 4\text{ mm}$ measured back from the front apex along the centerline**.

### 1.2 Control Surfaces (Elevons)
- **Chord:** 40 mm constant chord.
- **Span:** 280 mm per wing half.
- **Hinge Type:** 3M Blenderm medical tape or continuous Coroplast outer skin score-and-fold.
- **Reflex:** **$2.5\text{ mm}$ upward reflex at neutral trim** to stabilize the reflexed autostable reflex camber line required by tailless flying wings.

---

## 2. Airframe Assembly & Carbon/Bamboo Spar Integration
1. **Flute Orientation:** Align Coroplast flutes at 28° parallel to the leading edge of each wing half.
2. **Main Spar:** Insert a 4mm bamboo dowel or pultruded carbon rod through the main flute channel at 25% chord ($60\text{ mm}$ behind leading edge). Secure with UHU Por or low-temperature hot glue.
3. **Fuselage Pod Cutout:** Cut a 54mm wide by 180mm long central notch from the foam apex to slide the 3D-printed avionics pod into place. Secure pod with hot glue and reinforced filament tape cross-strapping.
4. **Motor Mount Alignment:** The rear firewall incorporates an integrated **$2^\circ$ down-thrust and $2^\circ$ right-thrust offset** to counter prop torque and pitch-up under sudden full-throttle bursts.

---

## 3. Marine & Mountain Environment Protection (Salt & Moisture Defense)
Coastal roads (e.g., Goa, Mumbai Marine Drive, East Coast Road) subject aircraft to high concentrations of sodium chloride salt fog, while mountain passes (Western Ghats, Nilgiris) bring dense mist and condensation.
1. **Conformal Coating:** Apply 2 thin coats of **silicone or acrylic conformal coating (e.g., MG Chemicals 422B or Kot-Coat 128)** over both sides of the ESP32-WROOM-32, MPU-6050, NEO-6M PCB, and 30A ESC.
2. **Exclusion Zones:** Do NOT coat the barometer vent hole (if equipped), USB connectors, or MPU6050 vent port.
3. **Corrosion Prevention on Contacts:** Coat the 3S 18650 copper busbar contacts and servo lead headers with dielectric silicone grease (Dow Corning 4 or Permatex 22058).
4. **Ventilation:** Forward NACA ducts provide cooling airflow directly over the ESC heat sink while shedding droplets through the bottom floor drains.

---

## 4. Cascaded PID Tuning & Control Surface Rates

### 4.1 PID Gains (Configured in Airborne Firmware)
- **Roll Rate PID:**
  - $K_p = 3.50$ (Crisp bank response without high-frequency aileron flutter)
  - $K_i = 0.05$ (Corrects asymmetric crosswind drift along coastal cliffs)
  - $K_d = 0.45$ (Suppresses mechanical overshoot)
- **Pitch Rate PID:**
  - $K_p = 3.80$ (Damped elevator response to eliminate flying wing phugoid porpoising)
  - $K_i = 0.05$ (Maintains steady level altitude in gusty thermals)
  - $K_d = 0.50$ (Rapid recovery from turbulence)

### 4.2 Servo Physical Throws
- **Aileron Travel:** $\pm 14^\circ$ ($\pm 12\text{ mm}$ at elevon trailing edge).
- **Elevator Travel:** $\pm 11^\circ$ ($\pm 9\text{ mm}$ at elevon trailing edge).
- **Expo:** $25\%$ exponential on ground station manual override stick mapping.

---

## 5. Standard Operating Procedures (SOP) & Checklists

### 5.1 Pre-Flight Hardware Inspection Checklist
- [ ] Airframe CG balanced at 172mm (test on two fingertips underneath).
- [ ] Elevons show equal $2.5\text{ mm}$ upward reflex at neutral stick.
- [ ] Motor spins smoothly counter-clockwise (pusher configuration with prop text facing forward).
- [ ] 3S 18650 cells measured $> 12.4\text{ V}$ ($4.13\text{ V/cell}$).
- [ ] Antenna oriented vertically on ground station and airborne pod.
- [ ] Ground station shows GPS lock with $\ge 7$ satellites.
- [ ] Video feed clear and latency $< 30\text{ ms}$ on ground station screen.
- [ ] Manual override tested: tapping `SPACE` on ground station displays `[OVERRIDE]` in $< 5\text{ ms}$.

### 5.2 Hand-Launch Procedure (Solo Pilot)
1. Arm motor and select **`MANUAL FPV`** (Mode 1).
2. Set throttle stick to **$65\%$** (firm steady push from 1400KV motor with 8045 prop).
3. Hold aircraft by bottom center finger grips behind CG.
4. Launch with a firm level throw directly into the wind at $10^\circ$ upward angle.
5. Climb to $25\text{ m}$ AGL, level wings, and press key **`2`** or select **`VISION AUTONOMOUS TRACK`** to engage car tracking.

### 5.3 Indian DGCA Compliance & VLOS Spotter Protocol
- **Operating Category:** Nano/Micro RPA (< 2 kg AUW) under Indian Drone Rules.
- **Flight Ceiling:** Strict maximum altitude **$60\text{ m}$ ($200\text{ ft}$) AGL**.
- **Visual Line of Sight (VLOS):** A dedicated Human Spotter must keep continuous un-aided visual line-of-sight on the aircraft while the Ground Station Operator monitors the computer vision pursuit telemetry.
- **Airspace Clearance:** Ensure operations are conducted strictly in **Green Zone** airspace (verified via DGCA DigitalSky portal). Never fly within 5 km of an active airport or naval base.
- **Instant FPV Override:** If a manned aircraft, bird of prey, or obstacle is spotted, the spotter calls "ABORT", and the operator taps `SPACE` to instantly command manual control and steer away.
