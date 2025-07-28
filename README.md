# WiPod ROS2 Setup Guide

This repository contains instructions for setting up the `wipod_ws` ROS 2 workspace. It includes all necessary dependencies, installation steps, and links to required SDKs and map files.

---

## 📦 Map Resource

Download the PCD map from Hugging Face:  
🔗 [https://huggingface.co/sai1405/PCD_maps/tree/main](https://huggingface.co/sai1405/PCD_maps/tree/main)

---

## 🔧 Dependency Installation

### 1. System Libraries

#### a. Xsens MTi SDK  
Download and install from Movella:  
👉 [Xsens MTi Manager SDK](https://www.movella.com/support/software-documentation)

#### b. GTSAM for LIO-SAM  
Install using the following commands:
```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

#### c. Teledyne FLIR (Spinnaker) SDK  
Download and install from Teledyne FLIR:  
👉 [Spinnaker SDK](https://www.teledynevisionsolutions.com/products/spinnaker-sdk/?model=Spinnaker%20SDK&vertical=machine%20vision&segment=iis)

---

## 🧩 ROS 2 Package Dependencies

Make sure the following ROS 2 (Humble) packages are installed:
- `ros-humble-desktop`
- `ros-humble-nav2*`

Install using:
```bash
sudo apt install ros-humble-desktop
sudo apt install ros-humble-nav2*
```

---

## 🏗️ Workspace Build Steps

Follow these steps to build the workspace:

```bash
# 1. Create workspace directories
mkdir wipod_ws && cd wipod_ws
mkdir src && cd src

# 2. Clone the workspace repository
git clone https://github.com/klausmikealsontheking/wipod_ws.git

# 3. Build the Xsens driver library
pushd src/sensing/sensor_components/Xsens_MTi_ROS_Driver_and_Ntrip_Client/src/xsens_mti_ros2_driver/lib/xspublic
make
popd

# 4. Build the entire workspace
cd ../../../../../../..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## ✅ Done!

You’re now ready to launch the WiPod stack using ROS 2 Humble.