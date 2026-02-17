# i# ESP32 IMU Signal Stability Analyzer
Real-Time Embedded Motion Analytics Pipeline

## Overview
This project implements an embedded motion analysis pipeline on ESP32 that converts raw IMU acceleration data into a quantitative stability score. The system performs fixed-rate sampling (~100 Hz), acceleration magnitude computation, digital filtering, jerk analysis, and Signal Stability Index (SSI) calculation to evaluate motion behavior under static and dynamic conditions.

Rather than simply logging sensor values, the goal of this project is to demonstrate embedded signal processing, feature extraction, and derived motion analytics similar to workflows used in robotics, wearables, and hardware sensing systems.

## System Pipeline
Raw IMU Data → Acceleration Magnitude → Moving-Average Filter → Jerk Analysis → Signal Stability Index (SSI)

## Hardware
- ESP32 Microcontroller
- I2C IMU Sensor
- Embedded C++ (Arduino Framework)

## Data
Processed datasets are provided in `/data/`:
- `rest_processed.csv`
- `shake_processed.csv`

Each dataset includes:

| Condition | Raw STD | Filtered STD | Noise Reduction | SSI |
|-----------|---------|--------------|-----------------|-----|
| Rest      | 0.00323 | 0.00122      | 62%             | 0.995 |
| Shake     | 0.71557 | 0.30849      | 57%             | 0.758 |

### Static Motion — Raw vs Filtered
![Rest Plot](docs/images/rest_plot.png)

### Dynamic Motion — Raw vs Filtered
![Shake Plot](docs/images/shake_plot.png)

### Jerk Analysis
![Jerk Plot](docs/images/jerk_plot.png)

### Signal Stability Index
![SSI Chart](docs/images/ssi_chart.png)

## Dashboard
![Dashboard](docs/dashboard_screenshot.png)

## Key Engineering Outcomes
- Fixed-rate (~100 Hz) embedded sampling over I2C
- ~62% reduction in signal variance using moving-average filtering
- Real-time feature extraction on-device (magnitude, jerk, SSI)
- Derived stability metric capable of distinguishing static vs dynamic motion states
