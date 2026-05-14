# Drone Garage

Drone Garage is a decentralized UAV communication and coordination system designed for resilient multi-drone operations in environments where traditional centralized communication may fail or become unreliable. The project combines wireless mesh networking, autonomous leader election, MAVLink telemetry forwarding, and edge-based object detection to create a distributed aerial communication platform with no single point of failure.

---

# Overview

Modern UAV systems often rely on centralized infrastructure or direct point-to-point communication. This project explores an alternative architecture where drones communicate through a self-healing mesh network capable of maintaining connectivity even when nodes fail or move out of range.

Each drone acts as both a communication endpoint and a routing node within the network. The system dynamically elects a leader drone responsible for forwarding telemetry and detection data back to the ground station.

The platform also integrates edge AI object detection using YOLOv8 running directly on Raspberry Pi hardware.

---

# Features

- Decentralized mesh-based drone communication
- Dynamic leader election with automatic failover
- MAVLink telemetry forwarding across the mesh network
- Edge AI object detection using YOLOv8
- Wireless multi-hop communication
- Base station telemetry aggregation
- Low-cost hardware architecture
- Real-time detection forwarding
- UART communication between microcontrollers and onboard computers
- ZMQ-based data transport to the ground station

---

# System Architecture

The system consists of several major components.
<img width="512" height="452" alt="Full_system" src="https://github.com/user-attachments/assets/55e92c7e-d208-4e75-ba0e-612f774aa105" />

## Drone Node

Each drone contains:
<img width="813" height="317" alt="indi_drone" src="https://github.com/user-attachments/assets/2b757898-2dd9-4f95-a7b5-b5a7a20fc0b3" />

### Raspberry Pi
- Runs object detection
- Handles MAVLink routing
- Communicates with ESP8266

### ESP8266
- Maintains mesh network participation
- Relays packets between drones
- Participates in leader election

---

## Mesh Network

The wireless mesh network is built using the `painlessMesh` library on ESP8266 devices.

Capabilities include:

- Multi-hop packet forwarding
- Dynamic topology management
- Node discovery
- Leader reelection after node failure
- Redundant communication paths

---

## Leader Drone

The elected leader drone is responsible for:

- Aggregating drone telemetry
- Receiving object detection data
- Forwarding data to the base station
- Acting as the gateway between the swarm and operator

If the leader fails, a new leader is automatically elected.

---

## Ground Station

The ground station receives data through a ZMQ socket and processes:

- MAVLink telemetry
- Object detection results
- Mesh communication statistics
- Drone status information

---

# Object Detection Pipeline

The system uses YOLOv8 for onboard object detection.

Detection data includes:

- Object label
- Confidence score
- Bounding box coordinates
- Timestamp information

Detections are generated locally on the Raspberry Pi and transmitted through the mesh network to the base station.

---

# Communication Flow

1. Drone captures image/video frame
2. YOLOv8 performs object detection locally
3. Detection packet is generated
4. Raspberry Pi sends packet to ESP8266 over UART
5. ESP8266 forwards packet through mesh network
6. Leader drone receives packet
7. Leader forwards packet to base station
8. Ground station processes and displays results

---

# UART Packet Structure

Communication between the Raspberry Pi and ESP8266 uses a custom UART framing protocol.

| Field | Description |
|---|---|
| Sync Bytes | Packet synchronization |
| Length | Payload size |
| Sequence Number | Packet ordering |
| Payload | Telemetry or detection data |
| Checksum | Error detection |
<img width="831" height="63" alt="packet" src="https://github.com/user-attachments/assets/859b3983-8ad3-422e-9d17-99677d548c0e" />

---

# Technologies Used

## Hardware
- Raspberry Pi
- ESP8266
- UAV platforms
- WiFi mesh networking hardware

## Software
- Python
- C++
- YOLOv8
- MAVLink Router
- painlessMesh
- ZeroMQ (ZMQ)

---

# Experimental Testing

The system was tested under multiple networking conditions including:

- Line-of-sight communication
- Non-line-of-sight communication
- Multi-hop relay scenarios
- Leader failure and reelection
- Variable node spacing

Metrics evaluated included:

- Packet loss
- Throughput
- End-to-end latency
- Mesh reliability
  
<img width="743" height="442" alt="drop_rate_by_experiment (1)" src="https://github.com/user-attachments/assets/63dd83d2-a7d0-4560-a359-e1b66aecf81d" />
<img width="743" height="442" alt="mean_delay_by_experiment (1)" src="https://github.com/user-attachments/assets/adaea87d-3cb9-4a86-9830-c76d929edb4f" />
<img width="743" height="442" alt="throughput_by_experiment (1)" src="https://github.com/user-attachments/assets/76a607be-2833-4ceb-ab24-048139424021" />

---

# Research Goals

This project investigates:

- Resilient UAV swarm communication
- Distributed aerial networking
- Low-cost autonomous coordination
- Edge AI integration for UAV systems
- Fault-tolerant drone communication architectures

---

# Future Work

Potential future improvements include:

- Scaling beyond three nodes
- Autonomous flight integration
- Improved routing optimization
- Enhanced security mechanisms
- Long-range radio integration
- Distributed task coordination
- Edge model optimization
- Real-time mapping and tracking
