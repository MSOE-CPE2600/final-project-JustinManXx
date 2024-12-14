# Control System

## Overview

This program simulates a control system using sensor data (temperature, pressure, and level) and adjusts an actuator using PID controllers. The system runs on two threads:
1. **Sensor Data Simulation**: Simulates and updates temperature, pressure, and level.
2. **Control System**: Uses PID controllers to adjust the actuator based on sensor readings.

Results are logged in an Excel file, which includes charts for visualization.

---

## Features

- Simulates sensor readings (temperature, pressure, and level).
- PID control for temperature and pressure adjustment.
- Writes data to an Excel sheet with real-time charts.
- Interactive control: Allows stopping the program using the Enter or Space key.

---

## Requirements

- [libxlsxwriter](https://libxlsxwriter.github.io/) library for Excel output

---

## Compilation

To compile the program, use the following command:

```bash
gcc -o controlSystem controlSystem.c -lpthread -lxlsxwriter -lm
```
Or use the Makefile given to compile:
```bash
make
```
