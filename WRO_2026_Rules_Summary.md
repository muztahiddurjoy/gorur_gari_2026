# WRO 2026 Future Engineers: Self-Driving Cars — Rulebook & Project Guidelines Summary

> **Document Summary**: Comprehensive breakdown of the official *World Robot Olympiad (WRO) 2026 Future Engineers — Self-Driving Cars* General Rules (`WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules (1).pdf`). Tailored for team **gorur_gari_2026** to guide vehicle design, software architecture, competition operations, and GitHub documentation compliance.

---

## 1. Overview & Competition Format

- **Category**: WRO Future Engineers — Self-Driving Cars (Age 14–22, born 2004–2012 for the 2026 season).
- **Team Composition**: 2–3 students + 1 coach (Coach must be $\ge$ 18 years old).
- **Core Format**: Time Attack autonomous racing (single car on track per attempt).
- **Total Maximum Score**: **122 Points**
  - ~75% Vehicle Field Performance (92 pts max: 30 pts Open + 62 pts Obstacle)
  - ~25% Engineering Journal & GitHub Documentation (30 pts max)
- **Surprise Rule**: Expected to be introduced for international finals (can modify/add specific challenge rules prior to the event).

---

## 2. Vehicle Regulations & Hardware Constraints (CRITICAL)

### 📐 Physical Dimensions & Weight
| Parameter | Maximum Limit | Notes / Enforcement |
| :--- | :--- | :--- |
| **Length** | **300 mm** | Checked during vehicle check and repair times |
| **Width** | **200 mm** | Checked during vehicle check and repair times |
| **Height** | **300 mm** | Checked during vehicle check and repair times |
| **Weight** | **1.5 kg** | Strictly enforced at vehicle check |

### 🚗 Drivetrain & Kinematics Requirements
- **Kinematic Structure**: Must be a **4-wheeled vehicle** with **exactly one driving axle** (FWD, RWD, or 4WD) and **one steering actuator** (e.g., servo motor for Ackermann steering).
- **PROHIBITED Configurations**:
  - ❌ **NO Differential Drive Robots** (skid-steer / two-wheel drive turning via differential motor speeds).
  - ❌ **NO Electronic Differentials** (using one motor per side to steer/turn).
  - ❌ **NO Omni-wheels, Ball Casters, or Spherical Wheels**.
- **Driving Motors**: Maximum of **2 driving motors**. Both driving motors **must be physically coupled** (directly or via a shared gearbox/axle) to the driven wheels. They cannot be driven independently per wheel.

### ⚡ Electronics, Controllers & Wireless Restrictions
- **Controllers**: Any Single-Board Computer (SBC, e.g., Raspberry Pi 4/5, Jetson Nano) and/or Single-Board Microcontroller (SBM, e.g., Arduino, ESP32, STM32). Multiple controllers allowed.
- **Sensors**: Unlimited quantity, brand, or type (LiDAR, Depth/RGB Cameras, ToF distance sensors, IMU, Color/Line sensors). Smartphones are allowed as cameras/processors.
- **Wireless Rule (STRICT)**: ❌ **ALL wireless communication (Wi-Fi, Bluetooth, RF) MUST BE DISABLED** during competition rounds. If built-in on SBC/SBM, it must be turned off in OS/firmware. Wire-only communication between components.

---

## 3. Mandatory Start & Control Button Procedure

The vehicle **must** strictly implement the 2-button startup procedure:

```
[Vehicle Switch OFF] 
         │
         ▼  (Turn Main Power Switch ON)
[Power Turned ON] ───> Entering Waiting State
         │
         ▼  (Judge signals "3, 2, 1, GO!" ──> Team presses Start Push Button)
[Program Execution Starts & Vehicle Moves]
```

1. **Power Switch**: Exactly **1 main switch** to power on the SBC/microcontroller.
2. **Start Button**: Exactly **1 physical Push Button** (or single screen touch / EV3 button) to initiate program execution.
3. **No Interaction**: No sensor calibration, physical pre-adjustments to pass data, switch configuration coding, or wireless triggering is permitted before or during the start.

---

## 4. Challenge Round Specifications

### 🏁 4.1 Open Challenge (Duration: 3 Minutes)
- **Goal**: Complete **3 full laps** in the random challenge direction (Clockwise or Counter-Clockwise) as fast as possible and stop autonomously.
- **Track Layout**:
  - Distance between walls: Dynamically set per round to **600 mm (narrow)** or **1000 mm (wide)** ($\pm 100\text{ mm}$).
  - No traffic signs (pillars) present.
  - **Outer Wall Rule**: The vehicle **must NOT touch the outer boundary wall** during Open Challenge rounds.
- **Finish Condition**: Stop completely inside the finish section after 3 laps (within 15 seconds), or cross into the corner section past the finish.

---

### 🚦 4.2 Obstacle Challenge (Duration: 3 Minutes)

- **Goal**: Complete **3 full laps** navigating around red and green traffic sign pillars, then perform **Parallel Parking** in the designated parking lot.
- **Track Layout**:
  - Distance between walls: Fixed at **1000 mm** ($\pm 10\text{ mm}$).
  - Traffic signs: Up to 7 Red and 7 Green parallelepiped pillars ($50 \times 50 \times 100\text{ mm}$).
- **Traffic Sign Obedience Rules**:
  - 🔴 **Red Pillar**: Must pass on the **RIGHT** side.
  - 🟢 **Green Pillar**: Must pass on the **LEFT** side.
  - *Post-Lap 3 Note*: After completing 3 laps, pillars on the way to the parking lot can be bypassed on either side, provided they are not moved.

```
       [Green Pillar]                [Red Pillar]
            │                             │
    ◀───────┘ (Pass LEFT)         (Pass RIGHT) └───────►
   [Vehicle]                             [Vehicle]
```

- **Pillar Displacement & Fault Thresholds**:
  - **Allowed**: Touching/moving a pillar *as long as* its base projection remains inside the **85 mm diameter circle** around its seat.
  - ❌ **Round Stop Penalty**: Moving/knocking a pillar completely outside the 85 mm circle immediately terminates the attempt.
  - ❌ **Wrong-Side Passing Threshold**: Passing a pillar on the wrong side ends the round as soon as the vehicle completely crosses the radial line running from the inner wall to the outer wall at that pillar's location.

- **Parallel Parking Regulations**:
  - **Parking Lot Location**: Located in the starting section. Width: **20 cm**; Length: **$1.5 \times \text{Vehicle Length}$**.
  - **Boundaries**: Bounded by two magenta wooden blocks ($200 \times 20 \times 100\text{ mm}$). Touching either magenta block immediately terminates the round with zero parking points.
  - **Fully Parked Criteria**: Vehicle projection completely inside the parking box **AND** parallel to the outer wall (wheel distance difference to outer wall $\le 2\text{ cm}$).
  - **Start Bonus**: Starting from inside the parking lot scores additional bonus points (awarded only if at least 1 full lap is completed).

---

## 5. Repairing Actions & Penalties

- **Permission**: Granted **once per round** only if the vehicle is completely stopped (stuck against wall, electronic glitch, etc.). Cannot be requested during the 3rd lap or while moving ($>50\text{ mm}$ in 5 seconds).
- **Execution**: Vehicle is removed, repaired (mechanically/electronically), placed in the center of the same section, powered ON, and restarted via the Start button.
- **Penalty**: **The total score for that round is divided by 2** (timer continues running during repairs). No code uploading or data input is allowed during repair.

---

## 6. Scoring Matrix & Ranking Hierarchy

### 📊 Point Breakdown
| Category | Task / Milestone | Points | Max Total |
| :--- | :--- | :---: | :---: |
| **Open Challenge** | Laps passed successfully (8 sections / lap) | 1 pt / section | 24 pts |
| | Finish stop in start section after 3 laps | 3 pts | 3 pts |
| | Driving out of start section in round direction | 1 pt / lap | 3 pts |
| | **Open Challenge Subtotal** | | **30 pts** |
| **Obstacle Challenge** | Base lap driving & section passage | Same as Open | 30 pts |
| | Traffic signs not moved during 3 laps | 10 pts | 10 pts |
| | (Or traffic signs moved, but 3 laps completed) | (8 pts) | (8 pts) |
| | Start from parking lot bonus (min 1 lap completed) | 7 pts | 7 pts |
| | Parallel Parking — Fully inside & parallel ($\le 2\text{ cm}$) | 15 pts | 15 pts |
| | (Parallel Parking — Partial / non-parallel) | (7 pts) | (7 pts) |
| | **Obstacle Challenge Subtotal** | | **62 pts** |
| **Documentation** | Engineering Journal + GitHub Repo (Appendix C) | Rubric | **30 pts** |
| **GRAND TOTAL** | | | **122 pts** |

### 🏆 Tie-Breaking Criteria (Priority Order)
1. **Total Points** (Best Open + Best Obstacle + Documentation)
2. Points of the **best Obstacle Challenge round**
3. Time of the **best Obstacle Challenge round**
4. Points of the 2nd-best Obstacle Challenge round
5. Time of the 2nd-best Obstacle Challenge round
6. Points for **Engineering Journal & Documentation**
7. Points of the best Open Challenge round
8. Points of the 2nd-best Open Challenge round
9. Time of the best Open Challenge round
10. Time of the 2nd-best Open Challenge round

---

## 7. Documentation & GitHub Submission Guidelines (30 Points)

Documentation MUST be submitted via a public GitHub repository link **no later than 3 weeks before the competition** and remain public for **at least 12 months**.

### 📅 Mandatory Commit History Schedule
- **Commit 1**: $\ge$ **2 months** before competition (Must contain $\ge \frac{1}{5}$ of final code).
- **Commit 2**: $\ge$ **1 month** before competition.
- **Commit 3**: $\ge$ **2 weeks** before competition (**Primary evaluation commit** for scoring).

### 📝 README.md Requirements
- **Language**: English only.
- **Minimum Length**: **5,000 characters**.
- **Content**: Component overview, hardware/software interaction, detailed build/compile/upload instructions, 3D CAD files, and wiring diagrams.

### 📋 Evaluation Rubric Breakdown (Appendix C — 5 Criteria @ 0, 2, 4, 6 pts each)
1. **Mobility & Mechanical Design (6 pts)**:
   - Torque/speed calculations, chassis selection tradeoffs, steering linkage design, CAD diagrams, test/iteration logs.
2. **Power & Sensor Architecture (6 pts)**:
   - Complete power budget, current draw calculations, voltage regulation, sensor placement justification (FOV, glare/shadow mitigation), wiring diagrams, calibration procedures.
3. **Software Architecture & Obstacle Strategy (6 pts)**:
   - State machine diagrams / flowcharts, algorithm justification (PID control, LiDAR disparity extender, vision pipelines), edge-case handling, performance tuning metrics.
4. **Systems Thinking & Engineering Decisions (6 pts)**:
   - Explicit project constraints, trade-off analysis ("Why we chose X over Y"), system failure modes and mitigations, versioning progression (v1 $\rightarrow$ v2 $\rightarrow$ v3).
5. **Reproducibility & GitHub Quality (6 pts)**:
   - Professional repository layout, clean commit messages, code comments, complete CAD/STL/schematics, step-by-step reproduction guide.

---

## 8. Actionable Technical Directives for `gorur_gari_2026`

To ensure 100% compliance and maximum scoring efficiency for the `gorur_gari_2026` ROS 2 stack:

### 🛠️ Hardware & Mechanics
- [ ] **Verify Drivetrain Kinematics**: Ensure front-wheel steering (servo) + single rear/front drive axle (or center diff 4WD). **Eliminate any differential/skid-steering logic**.
- [ ] **Dimensions & Weight Verification**: Ensure robot dimensions are well under $300 \times 200 \times 300\text{ mm}$ and weight $< 1.5\text{ kg}$.
- [ ] **Dual Control Interfaces**: Confirm 1 hard power switch + 1 dedicated Start Push Button connected to GPIO/microcontroller.
- [ ] **Wireless Disable**: Add shell/OS scripts to disable Wi-Fi and Bluetooth on Raspberry Pi / Jetson upon boot during competition runs.

### 💻 Software & Navigation Stack (ROS 2)
- [ ] **Disparity Extender / Wall Follower (Open Challenge)**:
  - Optimize dynamic track width handling ($600\text{ mm}$ vs $1000\text{ mm}$).
  - Implement strict outer-wall avoidance logic (zero outer-wall touching allowed).
- [ ] **Vision / LiDAR Pillar Classifier (Obstacle Challenge)**:
  - Robust HSV / Color detection for **Red** (pass RIGHT) and **Green** (pass LEFT) pillars.
  - Implement radial threshold monitoring to prevent crossing the pillar radius line on the wrong side.
  - Track pillar position vs 85 mm safety margin to avoid knocking pillars out of bounds.
- [ ] **Parallel Parking State Machine**:
  - Implement precision odometry / ToF-guided parallel parking controller.
  - Ensure wall clearance distance is measured for $< 2\text{ cm}$ parallel alignment.
  - Avoid touching magenta boundary blocks.

### 📄 GitHub & Documentation Workflow
- [ ] Maintain consistent Git commit history adhering to the 2-month, 1-month, and 2-week milestones.
- [ ] Draft a comprehensive `README.md` ($\ge 5000$ chars) detailing ROS 2 nodes, wiring schematics, power budgets, and mechanical design rationale.
- [ ] Maintain an Engineering Journal PDF detailing test iterations and tradeoffs.
