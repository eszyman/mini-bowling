# 3D Printed Mini Bowling Alley: Pinsetter Controller

This repository contains the core logic for a state-driven, decoupled pinsetter architecture. The primary execution logic resides in the main `.ino` file, while hardware assignments and tuning variables are isolated in `pin_config.h` and `general_config.h`. 

```mermaid
flowchart LR
    %% Grouping for Inputs
    subgraph Inputs ["Inputs (Sensors & Buttons)"]
        direction TB
        IR[IR Sensor<br>Pin/Conveyor]
        Hall[Hall Effect Sensor<br>Turret Homing]
        BallTrig[Ball Trigger Sensor<br>Hardware Interrupt]
        BallSpd[Ball Speed Sensor<br>Hardware Interrupt]
        ResetBtn[Hardware Button<br>Reset/Pause/Maint]
    end

    %% Grouping for the Main MCU and Software Classes
    subgraph MCU ["Main Controller (Arduino Mega)"]
        direction TB
        Orchestrator{Game Orchestrator FSM}
        
        %% Internal Software Controllers
        TCtrl[Turret Controller]
        DCtrl[Deck Controller]
        SCtrl[Sweep Controller]
        BRCtrl[Ball Return Controller]
        BSCtrl[Ball Sensor Controller]
        WCtrl[WLED Controller]
        SMCtrl[ScoreMore Controller]

        %% Orchestrator commands the subsystems
        Orchestrator --> TCtrl & DCtrl & SCtrl & BRCtrl
        BSCtrl -->|Trigger Events| Orchestrator
        SMCtrl -->|Serial Commands| Orchestrator
        ResetBtn -->|Debounced Events| Orchestrator
    end

    %% Grouping for Actuators and Physical Outputs
    subgraph Actuators ["Outputs (Actuators & Relays)"]
        direction TB
        Stepper[Turret Stepper Motor<br>Step/Dir/Enable]
        Relay[Conveyor Motor Relay]
        DeckServos[Deck Servos<br>L/R Raise, Slide, Scissor]
        SweepServos[Sweep Servos<br>L/R Sweep]
        DoorServo[Ball Return Door Servo]
        LogicPins[Logical Indicator LEDs<br>Frame 1/2, Strike, Spare, etc.]
    end

    %% Grouping for External Systems
    subgraph External ["External Systems"]
        direction TB
        ESP32[ESP32 WLED Board<br>Fire & Forget JSON]
        PC[PC Scoring Software<br>ScoreMore]
        Strips[WS2812B NeoPixel Strips<br>L/R Lane, L/R Deck]
    end

    %% Routing Inputs to Controllers
    IR --> TCtrl
    Hall --> TCtrl
    BallTrig --> BSCtrl
    BallSpd --> BSCtrl

    %% Routing Controllers to Actuators
    TCtrl --> Stepper
    TCtrl --> Relay
    BRCtrl -.->|Monitors/Shares| Relay
    DCtrl --> DeckServos
    SCtrl --> SweepServos
    BRCtrl --> DoorServo
    SMCtrl --> LogicPins

    %% External Communications
    WCtrl -->|Serial1 TX 115200 Baud| ESP32
    ESP32 --> Strips
    SMCtrl <-->|Serial USB 9600 Baud| PC
    
    %% Styling
    classDef hardware fill:#2b2b2b,stroke:#555,stroke-width:2px,color:#fff;
    classDef software fill:#0f4c75,stroke:#3282b8,stroke-width:2px,color:#fff;
    classDef external fill:#1b262c,stroke:#e43f5a,stroke-width:2px,color:#fff;
    
    class Inputs,Actuators hardware;
    class TCtrl,DCtrl,SCtrl,BRCtrl,BSCtrl,WCtrl,SMCtrl,Orchestrator software;
    class External external;
```

---

## 1. Event-Driven Architecture & Short Loop Time

The system relies on a non-blocking Finite State Machine (FSM) architecture. The main `loop()` contains zero `delay()` statements and executes rapidly by continuously polling the `.update()` methods of each subsystem controller.

This short loop time is a strict mechanical necessity for three reasons:
1. **Stepper Motor Pulses:** The `AccelStepper` library requires frequent, uninterrupted execution to generate the precise microsecond pulses needed for smooth turret rotation.
2. **Asynchronous Serial Parsing:** `ScoreMoreSerial.update()` must constantly read incoming bytes from the PC to prevent buffer overflow and ensure immediate hardware responses.
3. **Hardware Interrupts:** The dual ball sensors utilize hardware interrupts (ISRs). A fast main loop ensures `BallSensor.updateAndCheck()` processes these flags instantly, keeping the `GameOrchestrator` synchronized with the physical ball.

When a subsystem needs to wait (e.g., waiting for a servo tween or pin settling), it logs the current `millis()` timestamp, updates its state, and yields control back to the main loop until the target duration has elapsed.
```mermaid
stateDiagram-v2
    %% Operational Modes (GameOrchestrator::_opMode)
    [*] --> BOOTING : System Startup (Preset Booting)
    BOOTING --> WAITING_FOR_BALL : Complete / Home (Throw 1 Ready, Preset Normal)

    %% Gameplay Loop (Mode: NORMAL, Transitions handle _throwCount)
    WAITING_FOR_BALL --> Throw1Cycle : Ball Sensed / Serial 't' (_throwCount == 1)
    WAITING_FOR_BALL --> Throw2Cycle : Ball Sensed / Serial 't' (_throwCount == 2 OR _fillBallArmed)
    WAITING_FOR_BALL --> ResetCycle : Strike Sensed / Lane Reset Button / Serial 'r'

    Throw1Cycle --> WAITING_FOR_BALL : Complete (Ready Throw 2, Trigger Ball Door)
    Throw2Cycle --> WAITING_FOR_BALL : Complete (Set Pins, Ready Throw 1, Trigger Ball Door)
    ResetCycle --> WAITING_FOR_BALL : Complete (Set Pins, Ready Throw 1)

    %% Idle States / Transitions (IsPaused context)
    WAITING_FOR_BALL --> PAUSED : Idle Timeout (120s, Sweep Guard, Preset Pause)
    PAUSED --> WAITING_FOR_BALL : Ball Sensed / Button Press / Serial 't' (Sweep Up)

    %% Maintenance Mode Loop
    BOOTING --> MODE_MAINT_ENTERING : Long Press / Serial 'm'
    WAITING_FOR_BALL --> MODE_MAINT_ENTERING : Long Press / Serial 'm'
    Throw1Cycle --> MODE_MAINT_ENTERING : Long Press / Serial 'm' (Sequence Abort)
    Throw2Cycle --> MODE_MAINT_ENTERING : Long Press / Serial 'm' (Sequence Abort)
    ResetCycle --> MODE_MAINT_ENTERING : Long Press / Serial 'm' (Sequence Abort)

    MODE_MAINT_ENTERING --> MODE_MAINT_ACTIVE : Emergency Halt & Decouple Complete (Preset Maintenance)
    MODE_MAINT_ACTIVE --> BOOTING : Button Press Exit (Initialize Kinematics)

    %% Sequence conceptual groups (Conceptual representation of multiple underlying states)
    state Throw1Cycle {
        T1_Start --> T1_Settle : Wait 3s (T1_PIN_SETTLE_WAIT)
        T1_Settle --> T1_Grab : Deck Grab (T1_DECK_GRAB)
        T1_Grab --> T1_Sweep_Deadwood : Sweep Back/Guard (T1_SWEEP_BACK/GUARD_POST)
        T1_Sweep_Deadwood --> T1_Drop : Deck Drop (T1_DECK_DROP)
        T1_Drop --> T1_Raise_Sweep : Sweep Up (T1_SWEEP_UP)
        T1_Raise_Sweep --> T1_Wait_Turret : Wait/Wake Turret (T1_WAIT_TURRET_IDLE)
        T1_Wait_Turret --> T1_End
    }

    state Throw2Cycle {
        T2_Start --> T2_Sweep_Clear : Settle 3s / Sweep Back/Guard (T2_SWEEP_BACK / RESET_SWEEP_GUARD_POST)
        T2_Sweep_Clear --> T2_Drop_Complete_Checks : Wait for Turret Pins/Drop Complete
        T2_Drop_Complete_Checks --> T2_Set_Pins : Dwell 1.5s / Deck Set Pins (RESET_DECK_SET)
        T2_Set_Pins --> T2_Raise_Sweep : Sweep Up (RESET_SWEEP_UP)
        T2_Raise_Sweep --> T2_Wait_Turret : Wait/Wake Turret (RESET_WAIT_TURRET_IDLE)
        T2_Wait_Turret --> T2_End
    }

    state ResetCycle {
        R_Sweep : Abort and Sweep Back (MANUAL_RESET_START)
        R_Guard : Sweep Guard (MANUAL_RESET_BACK)
        R_Sweep --> R_Guard
        R_Guard --> T2_Drop_Complete_Checks : Proceed with Throw 2 Pin Drop Logic
    }
```

---

## 2. Distributed Processing: WLED Integration

LED animations are offloaded to an external ESP-WROOM32 board running WLED. The primary Arduino commands this board using a "fire-and-forget" JSON API over `Serial1`.

Driving WS2812B NeoPixel strips directly from the primary microcontroller requires disabling hardware interrupts while data is pushed down the strip. Given the length of the lane and deck strips, this process would block the CPU for several milliseconds, breaking the AccelStepper pulse timing (causing lost steps) and creating blind spots for the ball trigger ISRs. Offloading the LEDs allows the Arduino Mega to focus entirely on real-time kinematics and game state FSM execution.

```mermaid
flowchart LR
    %% Styling
    classDef pc fill:#2b2b2b,stroke:#555,stroke-width:2px,color:#fff;
    classDef mega fill:#0f4c75,stroke:#3282b8,stroke-width:2px,color:#fff;
    classDef esp fill:#1b262c,stroke:#e43f5a,stroke-width:2px,color:#fff;

    subgraph PC_Node ["PC (Scoring & UI)"]
        ScoreMore[ScoreMore Software]
    end

    subgraph Mega_Node ["Arduino Mega 2560 (Kinematics FSM)"]
        direction TB
        SM_Class[ScoreMoreController]
        Orchestrator[Game Orchestrator]
        WLED_Class[WledController]
        
        SM_Class <-->|State Updates & Parsed Cmds| Orchestrator
        Orchestrator -->|Trigger Game Events| WLED_Class
    end

    subgraph ESP_Node ["ESP32 (Lighting Co-Processor)"]
        direction TB
        WLED_Core[WLED JSON API]
        LEDs[Lane & Deck NeoPixels]
    end

    %% Serial Link 1: Mega <-> PC
    ScoreMore -->|USB Serial 0 9600 Baud<br/>Cmds: 't', 'r', 'm', 'WRITE:10:1'| SM_Class
    SM_Class -->|USB Serial 0 9600 Baud<br/>Events: 'INPUT_CHANGE:6:0'| ScoreMore
    
    %% Serial Link 2: Mega -> ESP32
    WLED_Class -->|Hardware Serial 1 115200 Baud<br/>Fire & Forget JSON: 'ps\:1'| WLED_Core
    WLED_Core -->|FastLED/NeoPixel Data line| LEDs

    %% Apply Styles
    class PC_Node,ScoreMore pc;
    class Mega_Node,Orchestrator,SM_Class,WLED_Class mega;
    class ESP_Node,WLED_Core,LEDs esp;
```
---

## 3. Subsystem Breakdown

The codebase is modularized into distinct C++ classes, each managing a specific hardware zone.

| Controller | Function | Core Hardware (`pin_config.h`) |
| :--- | :--- | :--- |
| **TurretController** | Manages the stepper motor that loads pins into the deck. Tracks homing, pin detection lockouts, and empty-turret purges. | `STEP_PIN`, `DIR_PIN`, `IR_SENSOR_PIN`, `HALL_EFFECT_PIN` |
| **DeckController** | Controls the sliding, raising, dropping, and scissor mechanisms that set and pick up the pins on the pindeck. | `SCISSOR_PIN`, `SLIDE_PIN`, `RAISE_LEFT_PIN`, `RAISE_RIGHT_PIN` |
| **SweepController** | Drives the left and right sweep arms to clear deadwood using non-blocking tweening for smooth movement. | `LEFT_SWEEP_PIN`, `RIGHT_SWEEP_PIN` |
| **BallReturnController** | Operates the safety door for the ball return, holding it open on a timer after a frame clears. | `BALL_RETURN_PIN` |
| **BallSensorController** | Monitors the physical ball speed and trigger sensors using hardware interrupts to guarantee detection. | `BALL_SENSOR_PIN`, `BALL_SPEED_PIN` |
| **WledController** | Manages the `Serial1` link to the ESP32, translating game events into specific WLED preset triggers. | `Serial1` (Hardware TX) |
| **ScoreMoreController** | Parses incoming serial data from the scoring software to drive lane states, strike animations, and resets. | `Serial` (Main USB) |

---

## 4. The Game Orchestrator

The `GameOrchestrator` acts as the master FSM. Rather than directly controlling hardware pins, it commands poses and sequence triggers to the subsystem controllers. 

Key responsibilities include:
* **Boot Sequence:** Safely homing the turret, clearing the deck, and priming the initial 10 pins.
* **Throw Cycle:** Determining if a Throw 1 (grab and sweep) or Throw 2 (deck clear) is required based on the ball trigger and `_throwCount`.
* **Background Fetching:** Instructing the turret to refill in the background while waiting for the bowler.
* **State Management:** Handling power states, timeouts, and emergency overrides.

---

## 5. Hardware Interface & Operation

The physical interface relies on a single hardware button connected to `PINSETTER_RESET_PIN`. A non-blocking debounce algorithm distinguishes between tap and hold durations to control three distinct system states.

### Standard Operation (Short Press < 1000ms)
During normal operation, a short press acts as a manual frame reset. It aborts the current sequence, drops the sweep to clear deadwood, resets `_throwCount` to 1, and sends a hardware reset pulse back to the ScoreMore PC software.

### Pause Mode (Idle Timeout)
To protect servos and reduce power consumption, the orchestrator implements an automatic pause state. 
* **Trigger:** The system idles in `WAITING_FOR_BALL` for 2 minutes (120,000 ms) without detecting a ball or receiving a serial command.
* **Execution:** The sweep arms drop to `GUARD` to physically block the pit. The ESP32 is commanded to `WLED_PRESET_PAUSE`. Background pin drops are suspended.
* **Wake:** Throwing a ball (breaking the IR beam), a short button press, or a serial `'t'` command immediately wakes the system, raising the sweep arms and resuming normal gameplay.

### Maintenance Mode (Long Press ≥ 1000ms)
Maintenance Mode is a safe state designed for clearing jams or making physical adjustments without fighting motor torque.

* **Entering (Long Press or `'m'` serial command):**
  1. **Emergency Halt:** Turret stops instantly; ball return door snaps closed.
  2. **Clearance:** Deck forces scissors open to drop held pins; sweep moves to `GUARD`.
  3. **Decoupling:** Deck lowers to maintenance height. The `SlideServo` is explicitly detached, and the turret `STEPPER_ENABLE_PIN` is pulled HIGH to cut stepper coil power. The WLED strip updates to the maintenance preset.
* **Exiting (Short Press):**
  A short press exits Maintenance Mode. Servos re-attach and stepper torque is restored. The system transitions directly to the `BOOT_INIT` state. The ensuing booting routine automatically handles the LED preset transitions and executes a complete re-homing sequence for the turret, deck, and sweep before resuming gameplay.
