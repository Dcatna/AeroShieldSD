# AeroShield – Mission-Centric Assurance Pipeline (PX4/QGC/MAVLink Baseline)

The **AeroShield** project aims to build a reproducible prototype connecting a MAVLink-based C2 baseline (**PX4 + QGroundControl + MAVSDK**) to a simple assurance pipeline.  

Our objective is to create a traceable flow for mission assurance:

> **Mission Execution → Logged Trace → Attack Replay → Contract Check/Test Result**

This pipeline will include:
- **Mission contract templates** (JSON/YAML)
- **Logging** of MAVLink messages, operator commands, and autopilot states
- **Attack injection scripts** for replaying and modifying MAVLink traces
- A **digital twin testbed** (Gazebo or AirSim) to run baseline and adversarial scenarios

The end goal is to deliver a reproducible workflow, documentation, and sample logs that can be used for future mission assurance research.
