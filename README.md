# Adaptive Digital Control Architecture for Multi-Agent Industrial Electroplating Lines

This repository contains the implementation and supporting materials for the paper:

**Adaptive Digital Control Architecture for Multi-Agent Industrial Electroplating Lines: A Modular Microcontroller-Based Approach**

## Overview

This work presents a deterministic embedded control architecture for an industrial electroplating line with:

- **two autonomous trolleys**
- **18 station-aligned process positions**
- **shared-track transport**
- **redundant grouped baths**
- **layered safety supervision**
- **compact execution on a single ATmega2560 microcontroller**

The central goal of the project is to show that adaptive, resource-aware, and safety-layered electroplating control can be realized on a compact embedded platform without a real-time operating system and without a computationally heavy supervisory runtime.

The controller combines:

- asynchronous finite-state execution for each trolley
- runtime allocation of equivalent process baths
- dwell-time-preserving retrieval from grouped baths
- binary-coded station-referenced positioning
- distributed thermal supervision
- predictive admissibility checking
- software arbitration and hardware-wired fail-safe protection

## Main Contributions

The repository reflects the core technical contributions reported in the manuscript:

1. **Asynchronous dual-trolley control**  
   Each trolley advances according to local execution conditions rather than a globally blocking sequential routine.

2. **Virtual Process Groups (VPGs)**  
   Equivalent technological baths are abstracted into grouped resources, allowing runtime destination selection instead of rigid fixed-bath addressing.

3. **Temporal FIFO retrieval**  
   Retrieval from grouped baths is based on process-completion readiness, preserving dwell-time integrity while improving throughput.

4. **Compact embedded process representation**  
   The process logic is encoded in a form suitable for EEPROM-centered execution on a resource-constrained controller.

5. **Layered safety architecture**  
   Safety is implemented through predictive pre-start checks, runtime software conflict prevention, and hardware-wired normally closed interlocks.

## Repository Contents

Typical repository contents include:

- `Firmware_Main_Electronics_FINAL_DT...ino`  
  Main embedded controller implementation for the ATmega2560-based electroplating control architecture.

- `Validation_Dispatcher_Sim...ino`  
  Controller-oriented validation companion used to reconstruct comparative operating trends and scenario-level performance results reported in the paper.

- manuscript files (`.docx`, figures, supplementary material)  
  Supporting research documentation associated with the article.

If filenames differ slightly across revisions, use the most recent versioned files.

## System Architecture Summary

### 1. Physical Line Model

The control architecture is derived from a real industrial electroplating layout with:

- 18 station-aligned process positions
- shared trolley corridor
- vertical lifting and transport phases
- grouped zinc-plating and passivation resources
- thermal process supervision
- operator-accessible industrial environment

### 2. Positioning Principle

Instead of relying primarily on cumulative motion estimation, the controller reconstructs trolley position from **binary-coded workstation-referenced sensor states**. This provides station-level absolute positioning aligned with the actual process structure.

### 3. Runtime Resource Allocation

Conventional electroplating control often maps each process step to a fixed physical bath. In this architecture, equivalent baths are represented as **Virtual Process Groups**, and the physical destination is selected at runtime according to:

- availability
- conflict status
- grouped-bath admissibility
- safety constraints

### 4. Retrieval Logic

Grouped-bath unloading is handled through **Temporal FIFO** semantics. Retrieval is driven by process-completion readiness rather than simple entry order, which improves grouped-bath utilization while preserving dwell-time correctness.

### 5. Safety Layers

The system uses multiple protection layers:

- **predictive admissibility checking before execution**
- **runtime software arbitration**
- **shared-track conflict prevention**
- **emergency stop handling**
- **hardware-wired normally closed interlocks**
- **distributed thermal monitoring and fault handling**

## Validation Philosophy

This repository includes a **controller-oriented validation companion**, not a full industrial digital twin.

That distinction is important.

The purpose of the validation module is to remain close to the implemented control logic and to reconstruct controller-level comparative behavior reported in the paper, including:

- comparative makespan trends
- trolley idle-time reduction
- grouped-bath utilization improvement
- bottleneck-event reduction
- degraded grouped-bath operation
- predictive recipe admissibility timing

The validation companion is therefore intended as a **transparent scenario-based verification tool** that mirrors the operating logic of the embedded controller at a compact level.

## Reported Results

In the manuscript, controller-centered validation of an 18-station zinc electroplating line showed:

- **makespan reduction:** 1642 min -> 1244 min over a 100-batch horizon
- **throughput improvement:** 24.2%
- **average trolley idle time:** 18.4 min/batch -> 4.1 min/batch
- **grouped-bath utilization:** 64% -> 91%
- **tracked bottleneck incidents:** 18 -> 2

These results indicate that adaptive grouped-resource handling and asynchronous trolley coordination can improve productivity while maintaining dwell-time integrity and safety invariants.

## How to Use the Files

### Main firmware

The main firmware is the embedded controller implementation. It is intended for:

- architecture inspection
- logic review
- supplementary material support
- adaptation for laboratory or industrial prototyping

Typical functional blocks include:

- station decoding
- trolley state machines
- grouped-bath assignment
- drum-cycle handling
- dwell-time tracking
- thermal monitoring
- HMI signaling
- safety and emergency logic

### Validation companion

The validation script is intended for:

- scenario-level comparison
- reproducing controller-level trends described in the manuscript
- checking grouped-bath allocation behavior
- examining degraded operation logic
- inspecting predictive admissibility timing and batch-level metrics

## Implementation Notes

- The project is intentionally based on a **deterministic embedded approach**
- The controller logic is designed for **harsh industrial conditions**
- The implementation emphasizes **execution consistency**, not abstract optimization detached from physical control
- The supplementary scripts are meant to support the manuscript and clarify how the reported architecture behaves

## Limitations

This repository should be interpreted with the same boundaries stated in the manuscript:

- it is **not** a full plant-wide MES/SCADA optimization framework
- it is **not** a complete digital twin of the production line
- the validation logic is a **controller-oriented scenario companion**
- predictive admissibility checking is implemented as a **conservative embedded filter**, not as a full hybrid process model
- the architecture is tailored to the studied class of electroplating lines and should be adapted carefully for other layouts

## Suggested Citation

If you use this repository in academic or technical work, cite the associated paper.

**Title:** Adaptive Digital Control Architecture for Multi-Agent Industrial Electroplating Lines: A Modular Microcontroller-Based Approach

## Contact / Notes

This repository accompanies a research study focused on deterministic embedded control for industrial electroplating systems.  
For manuscript-specific interpretation, always refer to the latest accepted version of the paper.

---

## Quick Summary

This repository demonstrates how a compact microcontroller-based controller can:

- coordinate multiple trolleys on a shared industrial line
- allocate redundant process baths adaptively
- preserve technological dwell times
- maintain safety through layered protection
- improve throughput without relying on heavy supervisory infrastructure
