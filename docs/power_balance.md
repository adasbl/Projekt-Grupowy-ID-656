# Robot Power Budget

## 1. Available Power

| Power Supply Module | Output Voltage | Maximum Output Current | Total Available Power |
| :--- | :--- | :--- | :--- |
| **Waveshare UPS Module 3S** | 5 V | 5 A | **25.00 W** |

---

## 2. Power Consumption Calculation – Component Breakdown

Maximum (peak) power demand of all components.

| Component | Peak Power | Notes / Calculation Basis |
| :--- | :--- | :--- |
| **NVIDIA Jetson Nano** | 5.00 W or 10.00 W | Depending on the selected operating mode (nvpmodel). |
| **Intel 8265NGW** | ~2.00 W | Active Wi-Fi/BT transmission under load. |
| **RPLiDAR A1** | 1.75 W | Peak consumption during scanner startup. |
| **Camera and Sensors** | 0.50 W | Continuous operation of vision and sensors. |
| **STM32L476RGT6** | ~0.10 W | Maximum margin for microcontroller logic. |
| **Pololu Motor (Left)** | ~2.16 W | LP series: 6 V × 0.36 A (powered at 5 V). |
| **Pololu Motor (Right)** | ~2.16 W | LP series: 6 V × 0.36 A (powered at 5 V). |
| **DRV8833 Driver** | Negligible | Heat losses (minor at low current). |

---

## 3. Load Summary

Total consumption in two Jetson operating modes, assuming maximum load.

| Parameter | Variant 1: Jetson in 5W mode | Variant 2: Jetson in 10W mode |
| :--- | :--- | :--- |
| **Peak Logic Consumption** | ~9.35 W | ~14.35 W |
| **Peak Drive Consumption** | ~4.32 W | ~4.32 W |
| **TOTAL CONSUMPTION (Peak)**| **~13.67 W** | **~18.67 W** |
| **Power Supply Limit (UPS)** | 25.00 W | 25.00 W |
| **Remaining Power Margin** | **11.33 W (45%)** | **6.33 W (25%)** |
