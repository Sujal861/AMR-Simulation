# AMR (Autonomous Mobile Manipulator) Simulation

A comprehensive PyBullet-based simulation environment for an Autonomous Mobile Manipulator designed for agricultural applications. This project features a realistic 6-DOF manipulator arm mounted on a Mecanum wheel mobile base with integrated sensors.

## 🌟 Key Features

### 🤖 Robot Design
- **Mobile Base**: Mecanum wheel drive for omnidirectional movement
- **Manipulator Arm**: 6-DOF arm with enhanced reach and precision
- **End Effector**: Dual-finger gripper for object manipulation
- **Sensors**: LiDAR and camera integration for autonomous operation
- **Realistic Physics**: Accurate mass distribution and joint limits

### 🧠 Advanced Capabilities
- **Forward/Inverse Kinematics**: Complete kinematic analysis
- **Trajectory Planning**: Smooth motion planning with multiple algorithms
- **Pose Management**: Predefined poses for agricultural tasks
- **Real-time Control**: WebSocket-based control interface
- **Safety Features**: Joint limit enforcement and emergency stops
- **Autonomous Harvesting**: One-button start for automatic paddy harvesting

### 🌾 Agricultural Environment
- **Realistic Farming Field**: Paddy fields with water simulation
- **Vegetation**: Multiple paddy plants with growth stages
- **Terrain**: Irrigation channels and natural features
- **Task Automation**: Complete agricultural workflow simulation

## 🚀 Quick Start

### Prerequisites
- Python 3.8 or higher
- PyBullet
- NumPy, SciPy
- Other dependencies (see requirements.txt)

### Installation
```bash
# Clone the repository
git clone <repository-url>
cd amr-simulation

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Run setup verification
python test_setup.py
```

## 🎮 Usage

### 🎯 Autonomous Paddy Harvester (Recommended)
```bash
python sim/scripts/start_button_harvester.py
```
**Just press START to begin automatic paddy harvesting!**

The AMR will automatically:
- Navigate to each paddy plant in the field
- Rotate to face the plant
- Execute complete harvesting sequence:
  - Position arm for harvesting
  - Open gripper
  - Grasp the plant
  - Lift and transport to storage area
  - Release the plant
  - Return to home position
- Track progress and statistics
- Complete the entire field systematically

**Controls:**
- `START` - Begin autonomous harvesting
- `RESET` - Reset robot position and counters
- `QUIT` - Exit the program

### 🎯 AMR Demo
```bash
python sim/scripts/amr_demo.py
```
Experience the complete AMR capabilities:
- Autonomous navigation with Mecanum wheels
- Precise 6-DOF arm manipulation
- Gripper control and object handling
- Sensor-based inspection and mapping
- Complete agricultural task automation

### 🏃‍♂️ Basic Simulation
```bash
python sim/scripts/launch_sim.py
```
Launch the PyBullet simulation with the AMR robot in the farming environment.

### 🎮 Teleoperation
```bash
python sim/scripts/teleop.py
```
Control the robot manually using keyboard input:
- **WASD**: Base movement
- **Arrow Keys**: Arm joint control
- **1-6**: Individual joint selection
- **Space**: Emergency stop

### 🛤️ Waypoint Following
```bash
python sim/scripts/follow_waypoints.py
```
Execute predefined trajectories and pose sequences.

### 🎭 Pose Demonstrations
```bash
python sim/scripts/demo_poses.py
```
Cycle through various agricultural poses and sequences.

### 🌾 Farming Field Demo
```bash
python sim/scripts/farming_demo.py
```
Experience the complete farming environment with realistic paddy fields.

## 📁 Project Structure

```
amr-simulation/
├── sim/
│   ├── models/
│   │   └── robot.urdf          # AMR robot model
│   └── scripts/
│       ├── launch_sim.py       # Main simulation launcher
│       ├── start_button_harvester.py  # 🎯 Autonomous harvester
│       ├── amr_demo.py         # AMR capabilities demo
│       ├── teleop.py           # Manual control
│       ├── follow_waypoints.py # Trajectory execution
│       ├── demo_poses.py       # Pose demonstrations
│       └── farming_demo.py     # Farming environment demo
├── kinematics/
│   ├── fk.py                   # Forward kinematics
│   ├── ik_numeric.py           # Inverse kinematics
│   └── trajectory.py           # Trajectory planning
├── configs/
│   ├── dh_params.yaml          # DH parameters
│   ├── poses.yaml              # Pose configurations
│   └── poses.py                # Pose management
├── pi-control/
│   └── server.py               # WebSocket control server
├── tests/
│   └── test_kinematics.py      # Unit tests
├── requirements.txt
├── setup.py
└── README.md
```

## 🔧 Configuration

### Robot Parameters
- **Base Dimensions**: 1.2m × 0.8m × 0.15m
- **Wheel Configuration**: 4 Mecanum wheels (0.12m radius)
- **Arm Reach**: ~0.75m total reach
- **Payload Capacity**: 5kg at full extension
- **Joint Limits**: ±π radians for all joints

### Sensor Configuration
- **LiDAR**: 360° scanning range
- **Camera**: RGB camera for visual inspection
- **Sensor Tower**: Elevated sensor platform

### Harvesting Parameters
- **Field Layout**: 3×3 grid of paddy plants
- **Plant Spacing**: 1.5m between plants
- **Harvesting Speed**: Optimized for precision and safety
- **Storage Area**: Designated drop zone for harvested plants

## 🎯 Agricultural Tasks

The AMR is designed for various agricultural applications:

### 🌱 Planting Operations
- **Seed Placement**: Precise seed positioning
- **Drilling**: Soil preparation for planting
- **Covering**: Soil placement over seeds

### 💧 Irrigation Management
- **Watering**: Precise water application
- **Channel Maintenance**: Irrigation system upkeep
- **Moisture Monitoring**: Sensor-based soil analysis

### 🌾 Harvesting Support
- **Crop Inspection**: Visual assessment of plant health
- **Selective Harvesting**: Individual plant handling
- **Quality Control**: Size and ripeness evaluation
- **Autonomous Harvesting**: Complete field harvesting with one button

### 🔧 Maintenance Tasks
- **Tool Exchange**: Automated tool switching
- **Equipment Inspection**: Preventive maintenance
- **Repair Operations**: Field equipment servicing

## 🧪 Testing

Run the comprehensive test suite:
```bash
# Run all tests
pytest tests/

# Run specific test
python -m pytest tests/test_kinematics.py

# Run setup verification
python test_setup.py
```

## 🔌 Control Interface

### WebSocket Server
```bash
python sim/scripts/sim_backend.py
```
Start the simulation backend server for external control.

### Client Control
```bash
python pi-control/server.py
```
Connect to the simulation and send control commands.

## 🚀 Future Enhancements

### Planned Features
- **SLAM Integration**: Simultaneous Localization and Mapping
- **Path Planning**: Advanced navigation algorithms
- **Multi-Robot Coordination**: Swarm robotics support
- **Machine Learning**: AI-powered task optimization
- **Cloud Integration**: Remote monitoring and control
- **Real Hardware**: Physical robot implementation

### Advanced Capabilities
- **Dynamic Obstacle Avoidance**: Real-time path adjustment
- **Force Control**: Compliant manipulation
- **Vision Processing**: Advanced computer vision
- **Predictive Maintenance**: AI-based equipment monitoring
- **Weather Integration**: Environmental adaptation

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments

- PyBullet physics engine
- ROS community for robotics standards
- Agricultural robotics research community
- Open-source robotics projects

---

**Ready to explore autonomous mobile manipulation? Press START to begin harvesting!** 🚀🌾
