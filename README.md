# FFT Hardware Acceleration on Xilinx Zynq using AXI DMA + ACP

Hardware-accelerated Fast Fourier Transform (FFT) implementation on **Xilinx Zynq-7000 SoC** using **AXI DMA through the ACP port**, with benchmarking between software execution on the Processing System (PS) and hardware acceleration in Programmable Logic (PL).

---

# Project Overview

This project implements an **8-point FFT** using two execution methods:

## 1. Processing System (PS)

FFT executed in software on the ARM Cortex-A9 processor using C code.

### Includes:

- Input reordering (bit-reversal style arrangement)
- Butterfly computation stages
- Twiddle factor multiplication
- Execution time measurement using `XTime`

---

## 2. Programmable Logic (PL)

FFT executed in FPGA fabric using Xilinx FFT IP Core.

### Connected Through:

- AXI DMA
- ACP (Accelerator Coherency Port)
- DDR memory interface

### Two DMA Modes Tested:

- Polling mode
- Interrupt mode

---

# Objectives

- Compare software FFT vs hardware FFT performance
- Measure DMA overhead
- Evaluate ACP coherent memory transfers
- Compare polling vs interrupt transfer methods
- Demonstrate PS/PL co-design on Zynq

---

# Hardware Platform

| Item | Description |
|------|-------------|
| Board | Digilent Arty Z7-20 |
| SoC | Xilinx Zynq-7000 |
| CPU | ARM Cortex-A9 |
| FPGA Tool | Vivado 2021.2 |
| SDK | Vitis 2021.2 |

---

# Vivado Block Design

The hardware system contains:

- Zynq Processing System
- AXI DMA
- FFT IP Core
- AXI Interconnect
- Processor Reset
- System ILA Debug Core

```text
DDR Memory
   ↓
AXI DMA MM2S
   ↓
FFT IP Core
   ↓
AXI DMA S2MM
   ↓
DDR Memory
