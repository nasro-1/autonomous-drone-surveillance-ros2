# Autonomous Drone Surveillance Mission — ROS 2 / PX4 / YOLOv8

## Autonomous Drone Surveillance Mission

An autonomous UAV surveillance and target-delivery simulation integrating **PX4 SITL, ROS 2, Gazebo Harmonic, YOLOv8, MAVSDK-Python, OpenCV, and Micro-XRCE-DDS**.

The system implements an end-to-end perception-to-action mission in a simulated Sonoma Raceway environment:

```text
Takeoff
   ↓
Waypoint Patrol / Search
   ↓
YOLOv8 Target Detection
   ↓
Temporal Target Lock
   ↓
Pixel-to-Ground Localization
   ↓
Proportional Offboard Guidance
   ↓
Approach Target
   ↓
Payload Delivery
   ↓
Return to Home
   ↓
Landing / Disarm
```

---

## 👨‍🎓 Student Team

This project was developed and presented by:

- **Issam MENAS**
- **Abderrahmane Ryad AGGOUNE**
- **Amine Mohamed BEKHOUCHE**
- **Ahcene Zakaria AOUANOUK**
- **Lotfi DJEBBAR**

---

## 👨‍🏫 Academic Supervisor

**Nasr-Eddine Mellah**

**Academic Supervisor**  
**Specialization: Electronics**  
École Nationale Polytechnique d'Alger (ENP)

---

## 📌 Project Objective

The project aims to develop a fully autonomous UAV mission capable of:

1. Taking off autonomously.
2. Flying over a predefined surveillance area.
3. Searching for ground targets.
4. Detecting vehicles and people using YOLOv8.
5. Filtering unstable detections using temporal confirmation.
6. Estimating the target position from image coordinates.
7. Converting camera observations into ground offsets.
8. Guiding the UAV toward the target using PX4 Offboard control.
9. Approaching the target with controlled velocity.
10. Simulating payload delivery.
11. Returning to the home position.
12. Landing and completing the mission.

The project is presented as an Industrial Robotics project and uses PX4, ROS 2 and YOLOv8 for autonomous perception and flight simulation.

---

## 🏗️ End-to-End Architecture

```text
                    ┌───────────────────────────┐
                    │     Gazebo Harmonic       │
                    │   Sonoma Raceway World    │
                    └─────────────┬─────────────┘
                                  │
                                  ▼
                    ┌───────────────────────────┐
                    │       X500 UAV            │
                    │     IMX214 Camera         │
                    └─────────────┬─────────────┘
                                  │
                                  ▼
                    ┌───────────────────────────┐
                    │       ROS 2 / DDS         │
                    │    Camera Image Topic     │
                    └─────────────┬─────────────┘
                                  │
                                  ▼
                    ┌───────────────────────────┐
                    │        YOLOv8             │
                    │  Target Object Detection  │
                    │                           │
                    │ car / truck / bus / person│
                    └─────────────┬─────────────┘
                                  │
                                  ▼
                    ┌───────────────────────────┐
                    │   Temporal Lock Filter   │
                    │   60-frame confirmation   │
                    └─────────────┬─────────────┘
                                  │
                                  ▼
                    ┌───────────────────────────┐
                    │  Pixel-to-Ground Model    │
                    │    Pinhole Projection     │
                    └─────────────┬─────────────┘
                                  │
                                  ▼
                    ┌───────────────────────────┐
                    │  Mission State Machine    │
                    │                           │
                    │ TAKEOFF                   │
                    │ WAYPOINT_NAV              │
                    │ SEARCH                    │
                    │ TARGET_LOCKED             │
                    │ APPROACH                  │
                    │ PAYLOAD_RELEASE            │
                    │ RTH                       │
                    │ LANDED                    │
                    └─────────────┬─────────────┘
                                  │
                                  ▼
                    ┌───────────────────────────┐
                    │       MAVSDK-Python       │
                    │   PX4 Offboard Commands   │
                    └─────────────┬─────────────┘
                                  │
                                  ▼
                    ┌───────────────────────────┐
                    │        PX4 SITL           │
                    │   Flight Controller       │
                    └───────────────────────────┘
```

---

## 🧠 Technologies

| Technology | Role |
|---|---|
| PX4 SITL | Flight-controller simulation |
| ROS 2 Humble | Robotic middleware and communication |
| Gazebo Harmonic | Physics and 3D simulation |
| YOLOv8 | Real-time object detection |
| OpenCV | Image processing and visualization |
| MAVSDK-Python | High-level UAV control |
| Micro-XRCE-DDS | PX4 ↔ ROS 2 communication bridge |
| DDS | Distributed communication |
| Python | Mission and perception logic |
| X500 | Simulated quadrotor platform |
| IMX214 | Simulated onboard camera |

---

## 🎯 Mission Workflow

### 1. Takeoff

The UAV arms and takes off to the mission altitude.

### 2. Waypoint Navigation

The drone follows predefined NED patrol waypoints over the simulated racetrack.

Example patrol coordinates:

```text
(30,  0, -20)
(30, 30, -20)
( 0, 30, -20)
( 0,  0, -20)
```

### 3. Target Search

During the mission, the camera continuously observes the ground.

### 4. Object Detection

YOLOv8 detects relevant objects in the camera image.

The project identifies:

- Car
- Truck
- Bus
- Person

The presentation specifies a detection confidence threshold of:

```text
confidence ≥ 0.40
```

### 5. Target Lock

To avoid unstable flight caused by occasional false detections, the system applies a temporal debounce mechanism.

The target must remain confirmed for:

```text
60 consecutive frames
```

before the mission transitions to the target-locked phase.

### 6. Target Localization

The detected image coordinates are converted into relative ground offsets using a pinhole-projection model.

The system estimates:

```text
North offset
East offset
```

from:

- Image position
- Camera field of view
- UAV altitude

### 7. Autonomous Approach

A capped proportional guidance controller generates the velocity command.

The approach velocity is limited by a maximum velocity and reduced as the UAV approaches the target.

### 8. Payload Delivery

At the target position, the system enters the payload-release phase.

In the current simulation, payload release is represented by a timed hover rather than a physical actuator.

### 9. Return to Home

After delivery, the UAV returns toward the home position.

### 10. Landing

The UAV lands and completes the mission.

---

## 👁️ Computer Vision

The simulated UAV camera is based on an **IMX214** camera feed.

The presentation reports an operating rate of approximately:

```text
25–30 Hz
```

The detection pipeline is:

```text
Gazebo Camera
      ↓
ROS 2 Camera Topic
      ↓
OpenCV / CvBridge
      ↓
YOLOv8
      ↓
Bounding Box
      ↓
Target Class Filtering
      ↓
Temporal Detection Filter
      ↓
Target Lock
```

---

## 🎯 Detection Filter

The temporal filter is designed to reduce detection flicker and false positives.

Logic:

```text
Receive camera frame
        ↓
YOLOv8 inference
        ↓
Is target class detected?
      /       \
    YES        NO
     ↓          ↓
Counter +1   Counter -1
      \        /
       ↓
Counter > 60?
      /   \
    NO     YES
    ↓       ↓
Continue   Lock Target
```

The filter is implemented in the mission/perception architecture to improve flight stability.

---

## 📐 Coordinate Transformation

Autonomous flight requires conversion between different reference frames.

The project considers:

- Inertial/NED frame
- Drone/body/FRD frame
- Camera/image frame
- World coordinates

The presentation describes the transformation concept as:

```text
P_world = R_drone · P_body + T_drone
```

The target image position is converted into horizontal ground offsets using the camera geometry and UAV altitude.

---

## 🚁 PX4 Offboard Guidance

The UAV is controlled through MAVSDK using PX4 Offboard commands.

The approach controller follows the proportional guidance concept:

```text
V_cmd = min(V_max, dist) / dist · e
```

where the commanded velocity is reduced as the UAV gets closer to the target.

The mission therefore transitions from:

```text
High-speed approach
        ↓
Controlled deceleration
        ↓
Position hold
        ↓
Payload delivery
```

---

## 🔄 Mission State Machine

The main mission node implements the following states:

```text
IDLE
  ↓
TAKEOFF
  ↓
WAYPOINT_NAV
  ↓
SEARCH
  ↓
TARGET_LOCKED
  ↓
APPROACH
  ↓
PAYLOAD_RELEASE
  ↓
RTH
  ↓
LANDED
```

The mission node publishes mission state information through ROS 2 topics.

---

## 📡 ROS 2 Interfaces

Important interfaces in the project include:

```text
/camera
/target_detected
/target_bbox
/mission/state
/mission/log
```

The mission node subscribes to target detection and bounding-box information and publishes mission state and log messages.

---

## 🧩 Main Project Files

```text
.
├── mission_node.py
├── uav_camera_det.py
├── uav_camera_det_patched.py
├── yolo_debug.py
├── demo_flight.py
├── demo_detect_land.py
├── simple_mission.py
├── film_demo.py
├── move_car.py
├── keyboard-mavsdk-test.py
├── setup_gimbal.py
├── KeyPressModule.py
├── worlds/
│   └── default.sdf
├── models/
│   ├── pickup/
│   ├── hatchback/
│   ├── hatchback_blue/
│   └── sonoma_raceway/
├── px4_ros2_gazebo.yml
├── How_to_run.md
└── yolov8n.pt
```

---

## 🧪 Simulation Environment

The project uses the **Sonoma Raceway** environment as the virtual testing ground.

The environment was selected to provide:

- Roads and racetrack geometry
- Vehicles
- Ground textures
- Complex visual conditions
- A realistic surveillance scenario

The project documentation also reports initial engineering challenges involving drone spawning and magnetic-field interference in the custom world.

---

## 🛠️ Engineering Challenges

The project addressed several simulation problems.

| Challenge | Solution |
|---|---|
| Black camera feed | NVIDIA driver/EGL configuration adjustments |
| Magnetic interference | PX4 EKF2 magnetic-check configuration adjustment |
| Drone coasting during approach | Position control during final approach |
| Detection flickering | 60-frame temporal debounce filter |

These engineering solutions were part of making the complete perception-to-flight loop stable.

---

## 📊 Reported Results

According to the project presentation, the system achieved:

### Lock Stability

**95%**

The presentation reports high reliability in detecting and tracking moving targets on the Sonoma Raceway environment.

### YOLOv8 Inference

```text
Average inference time: ~12 ms
```

### Maximum Approach Velocity

```text
5.0 m/s
```

### Stable Hover

```text
±0.1 m reported accuracy
```

The reported mission was verified in the project presentation using PX4 and Gazebo.

---

## 🖥️ Docker

The project includes a Docker-based execution workflow.

Build the image:

```bash
docker rm -f px4_ros2_gazebo_yolov8_container 2>/dev/null
docker build -t px4_ros2_gazebo_yolov8_image .
```

Run the container:

```bash
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run --privileged -it --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --env="XAUTHORITY=$XAUTH" \
  --volume="$XAUTH:$XAUTH" \
  --network=host --ipc=host --shm-size=2gb \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --rm \
  --name px4_ros2_gazebo_yolov8_container \
  px4_ros2_gazebo_yolov8_image
```

See `How_to_run.md` for the project-specific execution procedure.

---

## ⚙️ Important Configuration

The project uses PX4 SITL and Gazebo with the X500 platform.

The supplied configuration includes:

```text
PX4_GZ_MODEL=x500_depth
```

and a custom model pose:

```text
PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7"
```

The exact configuration may depend on the local PX4, Gazebo, ROS 2, NVIDIA and Micro-XRCE-DDS installation.

---

## ⚠️ Implementation Notes

The supplied project contains several implementation variants and demonstration scripts.

One important point when reproducing the project:

- The presentation describes **YOLOv8n** as the performance-oriented model.
- `mission_node.py` refers to `yolov8n.pt`.
- The supplied `uav_camera_det.py` currently loads `yolov8m.pt`.

Therefore, the model used at runtime should be checked before reproducing reported performance.

Similarly, some scripts are demonstrations or experimental variants rather than components of a single launch system.

---

## 🔮 Future Development

Potential extensions include:

- Real UAV deployment
- Real camera integration
- Depth-based target localization
- Improved camera calibration
- Multi-target tracking
- ByteTrack/DeepSORT integration
- Improved visual servoing
- EKF-based target-position estimation
- Dynamic obstacle avoidance
- Real payload actuator control
- GPS-denied navigation
- Embedded NVIDIA Jetson deployment
- Hardware-in-the-loop testing
- Real-time mission monitoring dashboard
- AI-based target classification and tracking

---

## 🎓 Academic Context

This project was developed as an **Industrial Robotics Module 2026** project.

It demonstrates the integration of:

- Electronics
- Robotics
- UAV control
- Embedded systems
- Computer vision
- Artificial intelligence
- ROS 2
- PX4
- Autonomous navigation
- Dynamic modeling
- Control systems

The project connects perception, decision-making and flight control into a complete autonomous robotic system.

---

## 👨‍🏫 Academic Supervision

**Nasr-Eddine Mellah**

**Academic Supervisor**  
**Specialization: Electronics**  
École Nationale Polytechnique d'Alger (ENP)

---

## 👨‍🎓 Student Team

**Issam MENAS**  
**Abderrahmane Ryad AGGOUNE**  
**Amine Mohamed BEKHOUCHE**  
**Ahcene Zakaria AOUANOUK**  
**Lotfi DJEBBAR**

---

## 📚 Documentation

The project presentation is included separately as:

```text
Autonomous Drone Surveillance Mission
```

The presentation contains the system architecture, computer-vision pipeline, kinematic/dynamic models, detection filtering, pinhole localization, proportional guidance, engineering challenges, and reported results.

---

## ⭐ Acknowledgment

This repository documents an academic autonomous-drone surveillance project developed by the student team under the academic supervision of:

**Nasr-Eddine Mellah**

**Specialization: Electronics**

---

## 📜 Third-Party Software

This project integrates several open-source technologies and software components, including:

- PX4
- ROS 2
- Gazebo
- YOLO / Ultralytics
- MAVSDK
- OpenCV
- Micro-XRCE-DDS

Please preserve and respect the licenses and attribution requirements of the individual third-party components used by the project.

---

## 📬 Project

**Autonomous Drone Surveillance Mission**

Industrial Robotics Module — 2026

**Academic Supervisor:**  
**Nasr-Eddine Mellah — Electronics**

**Student Team:**  
Issam MENAS · Abderrahmane Ryad AGGOUNE · Amine Mohamed BEKHOUCHE · Ahcene Zakaria AOUANOUK · Lotfi DJEBBAR
