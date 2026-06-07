# Wi-Fi 6 CSI Sensing Pipeline for PlutoSDR

An end-to-end, hardware-validated MATLAB pipeline for extracting high-fidelity Channel State Information (CSI) from 802.11ax (Wi-Fi 6) packets using ADALM-Pluto Software-Defined Radios (SDRs). 

This project repurposes the High Efficiency Long Training Field (HE-LTF) built into the Wi-Fi 6 standard to perform environmental sensing. By continuously estimating the multipath channel transfer function $H(f,t)$ over time, this pipeline enables device-free target detection, motion tracking, and velocity estimation. The extracted and corrected CSI data is formatted specifically for seamless integration into downstream Applied AI and deep learning pipelines.

## Key Features

* **Standards-Compliant Waveforms:** Utilizes exact 802.11ax HE-SU packet structures for realistic channel sounding.
* **High Temporal Resolution:** Optimizes packet payload length to maximize the packet delivery rate, yielding high-density CSI snapshots.
* **Robust Hardware Impairment Correction:** Features inline algorithms to eliminate local oscillator drift and timing jitter (CFO, CPE, SFO).
* **Live Delay-Doppler Processing:** Transforms raw frequency-domain CSI into real-time 2D Delay-Doppler maps to visualize target range and radial velocity.
* **Integrated Software Simulator:** Includes a memory-efficient synthetic channel simulator to test logic and visualize targets without physical SDRs.

## Hardware & Software Requirements

* **Hardware:** 2x ADALM-Pluto SDRs (One dedicated TX, one dedicated RX).
* **Software:** MATLAB (R2023a or newer recommended).
* **Toolboxes:** * WLAN Toolbox
    * Communications Toolbox
    * Communications Toolbox Support Package for ADALM-PLUTO Radio

## Repository Structure

* **`wifi_config.m`**  **Centralized Configuration Interface**
    Acts as the single source of truth for the entire sensing pipeline. It defines all shared parameters for the transmitter and receiver, including 802.11ax PHY settings (bandwidth, MCS, channel), ADALM-Pluto hardware serial numbers, RF gains, and signal processing toggles.
* **`WiFi_tx.m`**  **Waveform Generator & Transmitter**
    The active transmission engine. It generates a reproducible Wi-Fi 6 baseband burst and interfaces with the TX PlutoSDR, continuously streaming the waveform over the air using a Direct Memory Access (DMA) loop to maintain precise packet boundaries.
* **`WiFi_rx.m`**  **Receiver & CSI Extraction Engine**
    The core hardware processing script. It captures over-the-air RF frames, performs packet detection, and isolates the HE-LTF sequence. It applies inline hardware impairment corrections and features live data visualization (CSI heatmaps, Delay-Doppler maps, and RF waterfalls). 
* **`SENS_TEST.m`**  **Integrated Testbench (Hardware + Sim)**
    A comprehensive validation script. It functions as the standard hardware pipeline but features a toggle (`CFG.useHardware = false`) to activate a synthetic multipath channel simulator. This allows for offline testing of Doppler and delay logic against defined virtual targets (range, velocity, gain) without active radios.

## The Signal Processing Pipeline

Raw Wi-Fi CSI extracted from unsynchronized SDRs is heavily degraded by independent hardware clocks. This pipeline implements a rigid correction sequence to ensure the phase of $H(f,t)$ accurately represents physical environmental changes:

1.  **Packet Detection:** Utilizes sliding windowed autocorrelation on the Legacy Short Training Field (L-STF) to find accurate packet start indices.
2.  **Carrier Frequency Offset (CFO) Correction:** Mismatched TX/RX oscillators cause a continuous phase rotation. This is corrected using coarse (L-STF) and fine (L-LTF) estimation. Without this, the Doppler axis is entirely noise.
3.  **Channel Estimation:** Demodulates the HE-LTF segment and computes the least-squares channel estimate ($H(k) = Y(k)/X(k)$) yielding a complex vector representing attenuation and phase shift per frequency bin.
4.  **Common Phase Error (CPE) Removal:** Residual phase noise causes a scalar phase rotation identical across all subcarriers. Removed by subtracting the mean phase. Without this, Doppler peaks smear due to packet-to-packet phase jumps.
5.  **Sampling Frequency Offset (SFO) Removal:** Mismatched ADC/DAC sampling clocks create a linear phase slope across subcarriers. Removed by fitting and subtracting a straight line from the unwrapped phase. Without this, the range/delay axis artificially drifts.

## Usage Instructions

**1. Hardware Configuration**
Open `wifi_config.m` and update the `CFG.hw.txID` and `CFG.hw.rxID` variables with the explicit `sn:` serial numbers of your ADALM-Pluto devices. Adjust `CFG.phy.BW` (e.g., 'CBW20' or 'CBW40') as needed.

**2. Initialize the Transmitter**
Open a MATLAB session and execute `WiFi_tx.m`. The script will generate the waveform and begin a continuous DMA transmission loop. Leave this session running.

**3. Initialize the Receiver**
Open a **second, separate MATLAB session** (to prevent the TX loop from blocking the RX execution) and execute `WiFi_rx.m`. 
* You will be prompted in the Command Window to define the capture duration (in seconds).
* You will be prompted to enable/disable live plotting (select 'n' for maximum packet throughput).

**4. Data Extraction & AI Integration**
Once the capture completes, the receiver safely releases the SDR. Your data is available in the workspace as the complex double matrix `CSI_with_timestamps`. 
* **Column 1:** Elapsed temporal timestamp (seconds).
* **Columns 2-End:** Corrected complex CSI subcarriers.
Save this matrix as a `.mat` file for direct loading into PyTorch or TensorFlow for deep learning applications.

---
**Author:** 
Mohamed Elamine Benattia \\
Hadj Mebarek ZEGRAR (https://github.com/zegrarhadjmebarek-arch) \\
Salah Eddine ZEGRAR (https://github.com/salaheddinezegrari7-cell)
