# 2D Drone Self-Localization using AoA and Rotating Antenna (GNSS-free)

This repository contains the MATLAB code, measurement-based models, and simulation results corresponding to the paper:

**"Drone Self-Localization using AoA from Stationary Base Stations in GNSS-Denied Environments"**

## 📘 Project Overview

This study proposes a simple, cost-effective method for **GNSS-free drone localization** using **Angle of Arrival (AoA)** measurements. The method is particularly suited for **low-cost UAVs** and **GNSS-denied scenarios**.

The drone estimates AoA by **rotating a non-isotropic antenna** to measure **signal strength variations** from **stationary cellular base stations**. Using these AoA estimates from multiple BSs, the drone determines its 2D position via the **least squares (LS) method**.

### ✨ Key Features

- Antenna rotation-based AoA estimation
- Signal strength-based AoA angle derivation
- Least squares localization using multiple BSs
- Measurement-based error modeling
- Realistic simulation in macro cellular scenarios

---

## 🛠 Requirements

- MATLAB R2021a or later
- Signal Processing Toolbox (recommended)
- Tested on Windows and Linux

---

## 🗂 Folder Structure

