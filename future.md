# 🔮 Project Eagle-Eye — Future Engineering Roadmap & Architecture Analysis

This document outlines the robotic guidance upgrades, embedded on-board computer vision targets, and avionics roadmap for **Project Eagle-Eye**, the autonomous car-tracking fixed-wing UAV.

---

## 🏛️ Current Architectural State & Technical Audit

### Core Strengths
- **Dual-Core FreeRTOS Flight Controller**: Deterministic 500 Hz Mahony attitude filter and elevon mixer on an ESP32, completely decoupling low-level stabilization from high-level guidance.
- **Low-Cost Sourcing Architecture**: Functional bill-of-materials engineered under ₹2,000 INR using off-the-shelf maker inventory (SX1278 LoRa, 18650 cells, EMAX servos).
- **Hard Real-Time Manual Preemption**: Sub-5ms gamepad takeover interrupting autonomous LoRa commands instantaneously upon pilot input.

### Identified Bottlenecks & Hardware Realities
1. **Ground-Station Computation Split**: Target tracking (YOLOv8n + ByteTrack) runs on a ground laptop processing an analog 5.8GHz FPV video downlink, introducing video transmission static and frame drops at extreme distances.
2. **LoRa Telemetry Bandwidth Limits**: At 25 Hz and 16 bytes per frame, telemetry bandwidth is strictly budget-constrained, precluding raw image or dense point-cloud transmissions.
3. **Elevon Wind Shear Deflection**: High-speed highway pursuit ($> 55\,\text{km/h}$) under gusty crosswinds causes elevon control surface blowback without digital metal-gear servos.

---

## 🗺️ Multi-Phase Roadmap

### Phase 1: On-Board Edge AI Visual Servoing
- [ ] **On-Board NPU Integration**: Mount a Sipeed MaixCAM (SOPHGO SG2002 NPU, 1.0 TOPS) or Raspberry Pi Zero 2W to run quantized YOLOv8n directly in the nose cone via MIPI CSI-2.
- [ ] **Direct Flight Controller Serial Link**: Pass target bounding box pixel coordinates directly to the ESP32 flight controller over 1Mbaud UART, eliminating analog downlink latency.
- [ ] **Adaptive Pure Pursuit Guidance**: Implement 3D vector field path following (ALOS - Adaptive Line of Sight) ensuring smooth, energy-efficient circular loitering around moving target vehicles.

### Phase 2: ExpressLRS (ELRS) & Centimeter RTK-GPS
- [ ] **ExpressLRS Telemetry Migration**: Upgrade from SX1278 SPI LoRa to ExpressLRS 868/915 MHz with bidirectional MAVLink telemetry, delivering up to 250 Hz control packet rates.
- [ ] **Dual RTK-GPS Setup**: Deploy u-blox ZED-F9P RTK GPS on both the aircraft and the target vehicle, achieving centimeter-accurate relative positioning for automated rooftop landings.
- [ ] **Thermal FLIR Lepton Ingestion**: Add a secondary lightweight thermal core (FLIR Lepton 3.5) with automatic optical/thermal sensor fusion for nighttime search and pursuit operations.

### Phase 3: Aerodynamic Airframe V2 & Autonomous Retracts
- [ ] **EPP Molded Airframe with Carbon Spar Reinforcement**: Transition from hot-wire cut foam to molded high-density EPP with twin 6mm carbon spars to prevent high-speed flutter.
- [ ] **Digital Airspeed Pitot Tube (MS4525DO)**: Incorporate dynamic differential pressure airspeed sensing to prevent stall during steep bank-and-turn orbits into head/tail winds.
- [ ] **PX4 / ArduPlane Co-Simulation**: Stand up a Gazebo / SITL (Software In The Loop) simulation environment modeling true swept flying wing aerodynamics under turbulent atmospheric boundary layers.

---

## 🔒 Life-Safety & Airspace Failsafes

- **Autonomous Geo-Fencing & Altitude Clamping**: Firmware enforces strict hard ceilings (120m AGL per DGCA regulations) and circular geofence boundaries, executing immediate motor cutoff if breached.
- **Fail-Safe Spiral Descent**: In the event of dual LoRa signal loss and GPS lock failure, the elevons automatically lock into a stable $15^{\circ}$ banked glide descent with motor disarmed.
