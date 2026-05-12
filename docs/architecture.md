# Architecture Documentation

## System Overview

The Rocket Control TFG project implements a closed-loop trajectory control system for sounding rockets using rear-fin deflection. The system integrates with RocketPy's 6-DOF flight simulation through its internal `_Controller` infrastructure.

## System Architecture

```mermaid
graph TD
    subgraph Input
        Config[config.py]
        TOML[Leon 2 TOML]
        Reference[Trajectory CSV]
    end

    subgraph Core_Engine
        Main[main.py]
        Builder[Rocket Builder]
        EnvBuilder[Environment Builder]
        Sim[Simulation Orchestrator]
    end

    subgraph Flight_Simulation_RocketPy
        Physics[6-DOF Physics Engine]
        ControllerHook[_Controller Hook]
    end

    subgraph Control_Logic
        Control[fin_controller]
        Guidance[PD Guidance]
        Attitude[PID Attitude]
        Mixer[Mixer]
        FinAdapter[FinAdapter Aerodynamics]
    end

    Input --> Main
    Main --> Builder
    Main --> EnvBuilder
    Main --> Sim
    
    Builder --> Physics
    EnvBuilder --> Physics
    Sim --> Physics
    
    Physics <--> ControllerHook
    ControllerHook <--> Control
    
    Control --> Guidance
    Guidance --> Attitude
    Attitude --> Mixer
    Mixer --> FinAdapter
    FinAdapter --> Physics
```

## Data Flow

### 1. Initialization
- **Configuration**: Loads execution parameters from `config.py` and rocket geometry/motor data from TOML files.
- **Environment**: Sets up the RocketPy `Environment` with site-specific atmospheric and gravity data.
- **Rocket**: Constructs the `Rocket` assembly, attaching a `GenericMotor` and the controlled `GenericSurface`.

### 2. Control Loop (Simultaneous with Integration)
During each step of the ODE solver:
- **State Estimation**: The controller receives the current 13-state vector (position, velocity, quaternion, angular rates) from the solver.
- **Guidance**: Computes the required acceleration based on the position and velocity error relative to the reference trajectory.
- **Attitude Control**: Determines the target orientation to achieve the required acceleration and uses a PID loop to compute the virtual control moments.
- **Actuation**: Maps virtual moments to fin deflections, applying rate and position limits.
- **Aerodynamics**: The `FinAdapter` translates deflections into lift, side force, and roll moment coefficients, which are then used by the physics engine to calculate the resulting forces.

### 3. Output and Analysis
- **Post-Processing**: Extracts the integrated flight history and computes performance metrics (MAE, RMSE, Saturation).
- **Visualization**: Generates a suite of plots for trajectory analysis, tracking performance, and control effort.
- **Artifacts**: Saves all data, metrics, and configuration snapshots to a timestamped result directory.

## Design Patterns

- **Singleton Pattern**: A module-level controller state ensures consistent data access between the simulation callbacks and the aerodynamic model.
- **Adapter Pattern**: The `FinAdapter` decouples the control logic from RocketPy's internal coefficient handling.
- **PD-PID Architecture**: A cascaded loop structure separates trajectory guidance (outer loop) from vehicle stabilization (inner loop).
