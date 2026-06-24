# Robust Line Following Specification (PID + Normalization)

The existing line-following code is based on simple binary thresholds. This causes severe weaving (oscillations) and is highly sensitive to variations in room lighting and ground texture. We can implement a robust, continuous control algorithm using **Sensor Normalization** and a **PID Controller**.

---

## 1. Sensor Normalization

To make the three sensor readings comparable and immune to variations in ambient lighting, we must scale each raw value $V_i$ into a normalized range of $[0.0, 1.0]$:

* **`0.0`** represents the dark ground (minimum reflection).
* **`1.0`** represents the white tape (maximum reflection).

Using the minimum and maximum calibration limits obtained for each sensor:

$$V_{\text{norm}, i} = \frac{V_i - \text{Min}_i}{\text{Max}_i - \text{Min}_i}$$

---

## 2. Line Position Error Estimation

Instead of checking simple thresholds, we can compute the continuous position of the line relative to the center of the sensor array using a **weighted average** of the normalized readings ($L, M, R$):

$$\text{Error} = \frac{R - L}{L + M + R}$$

### Advantages

* **Immunity to Lighting:** Because the error is a ratio normalized by the sum $L + M + R$, overall changes in room brightness scale the numerator and denominator equally, leaving the computed error stable.
* **Continuous Range:** The error ranges continuously from $-1.0$ (line fully on the left) to $+1.0$ (line fully on the right).

---

## 3. PID Control Algorithm

A PID controller computes a smooth steering angle by analyzing the current error, its accumulation over time, and its rate of change:

$$\text{Steer Angle} = K_p \times \text{Error} + K_i \times \int \text{Error}\, dt + K_d \times \frac{d(\text{Error})}{dt}$$

* **Proportional ($K_p$):** Standard correction proportional to the line offset.
* **Integral ($K_i$):** Corrects steady-state offsets (usually set to $0$ or very small to avoid build-up).
* **Derivative ($K_d$):** Dampens corrections if the robot is returning to the center too quickly, eliminating weaving.

---

## 4. Dynamic Speed Control

To keep tracking stable, the robot should slow down on curves and speed up on straightaways:

$$\text{Speed} = \text{Base Speed} \times (1.0 - K_{\text{speed}} \times |\text{Error}|)$$

---

## Proposed Implementation Plan

We will create a new class/helper `PIDController` and integrate it into a new line-following script. We will need to store:

1. `min_values` and `max_values` for each sensor in `config.json`.
2. $K_p, K_i, K_d$ constants in `config.json`.
