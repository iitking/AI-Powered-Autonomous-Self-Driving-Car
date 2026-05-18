# ⚡ Power Architecture & Electrical Safety

Autonomous vehicles experience extreme dynamic current fluctuations as propulsion DC motors accelerate, decelerate, and steering servos rapidly sweep. This document specifies the power architecture to guarantee noise isolation, eliminate MCU brownouts, and ensure system safety.

---

## 1. Dual-Rail Power Distribution Design

```text
       [ LiPo Battery 2S/3S (7.4V - 11.1V, 2200mAh) ]
                              │
                    [ 10A Master Fuse ]
                              │
                    [ SPST Power Switch ]
                              │
             ┌────────────────┴────────────────┐
             │                                 │
     (Raw Motor Rail)                  (Logic & Servo Rail)
             │                                 │
             ↓                                 ↓
     ┌───────────────┐                ┌─────────────────┐
     │ TB6612FNG     │                │ DC-DC Buck #1   │ (Step-down to 5.1V @ 3.5A)
     │ VM (Motor Pwr)│                └────────┬────────┘
     └───────┬───────┘                         │
             │                         ┌───────┴───────┐
             ↓                         ↓               ↓
      Rear DC Motors            Raspberry Pi /   Arduino / ESP32
                                Jetson SBC       Logic VCC
                                               
                                       ┌─────────────────┐
                                       │ DC-DC Buck #2   │ (Step-down to 6.0V @ 3.0A)
                                       └────────┬────────┘
                                                │
                                                ↓
                                          Steering Servo
                                          (MG996R)
```

---

## 2. Common Ground Reference & Ground Loops

- **Requirement:** A single common ground connection MUST unite the battery, all buck regulators, the motor driver, the microcontroller, and the single-board computer.
- **Star Ground Topology:** Connect all ground returns to a central copper terminal block or busbar rather than daisy-chaining ground wires. This prevents high motor return currents from creating inductive ground bounce on the sensitive logic ground pins.

---

## 3. Battery Voltage Monitoring Circuit

To prevent discharging the LiPo battery below critical thresholds (3.3V per cell), a high-impedance voltage divider connects the battery terminal to an Arduino analog pin (A0):

```text
Battery (+) (7.4V - 12.6V)
       │
      [R1 = 10 kΩ]
       │
       ├───► Analog Pin A0
       │
      [R2 = 4.7 kΩ]
       │
      GND
```

**ADC Calculation Formula:**
$$V_{in} = V_{adc} \cdot \left(\frac{R_1 + R_2}{R_2}\right) = \left(\frac{\text{ADC}}{1023} \cdot 5.0\text{V}\right) \cdot \left(\frac{14.7}{4.7}\right) \approx \text{ADC} \cdot 0.01528\text{ V}$$

If $V_{in} < 6.8\text{V}$ (for 2S LiPo), the software triggers an audible buzzer alarm and disables propulsion to safeguard battery chemistry.
