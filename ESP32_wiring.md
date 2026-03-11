# LED Controller Project: Arduino Mega to ESP32

This repository contains the wiring and logic flow for a multi-strip LED controller setup utilizing an **Arduino Mega 2560** for high-level logic and an **ESP32** for WLED/FastLED driving.

---

## 🔌 Wiring Reference
The following diagram illustrates the standard multi-strip digital wiring for WLED-based systems.

![WLED Multi-strip Digital Wiring](https://kno.wled.ge/assets/images/content/WLED_MultistripdigitalWiring.png)

---

## 📊 System Architecture & Data Flow
The ESP32 acts as the primary driver, receiving "Fire & Forget" JSON commands from the Arduino Mega via a voltage-divided serial link.
If using a voltage divider circuit, use lower value resistors to ensure the logic falls below 0.8V
I used 470R and 1K values for my R1 and R2 and that worked, while 1K and 2K did not.

```mermaid
flowchart LR
    %% Styling
    classDef mega fill:#0f4c75,stroke:#3282b8,stroke-width:2px,color:#fff;
    classDef esp fill:#1b262c,stroke:#e43f5a,stroke-width:2px,color:#fff;
    classDef component fill:#333,stroke:#ccc,stroke-width:1px,color:#fff;

    subgraph Mega_Node ["Arduino Mega 2560"]
        Tx[Tx / Pin 18]
    end

    subgraph Converter ["Voltage Step Down"]
        VD[Voltage Divider Circuit<br>R1 = 1K<br>R2 = 470R]
    end

    subgraph ESP_Node ["ESP32"]
        direction TB
        Rx[Rx / GPIO3]
        GPI0[Data Pin]
        GPI1[Data Pin]
        GPI2[Data Pin]
        GPI3[Data Pin]
    end

    subgraph Converter2 ["Voltage Step Up"]
        LS[Logic Level Shifter<br>SN74AHCT125N]
    end

    subgraph Strips ["LED Strips"]
        LED1((LED Strip 1))
        LED2((LED Strip 2))
        LED3((LED Strip 3))
        LED4((LED Strip 4))
    end

    %% Serial Link: Mega -> ESP32
    Tx -->|5V Serial 1 115200 Baud| VD
    VD -->|3.3V Serial| Rx
    
    %% Data Flow
    GPI0 --->|WS281x|LS
    GPI1 --->|WS281x|LS
    GPI2 --->|WS281x|LS
    GPI3 --->|WS281x|LS

    LS -->|Data Line 1|LED1
    LS -->|Data Line 2|LED2
    LS -->|Data Line 3|LED3
    LS -->|Data Line 4|LED4

    %% Apply Styles
    class Mega_Node mega;
    class ESP_Node esp;
    class LED1,LED2,LED3,LED4 esp;
    class VD,LS component;
```
