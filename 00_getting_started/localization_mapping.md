# RoboRacer Offroad Localization with Livox LiDAR

This repository contains a complete setup guide for localization and mapping on the **RoboRacer offroad platform** using the **Livox MID360 LiDAR sensor**. The system is built using **ROS2** and integrates localization, mapping, and navigation pipelines tailored for challenging offroad environments.

---

## Prerequisites

- **ROS2** installed
- **Livox MID360 LiDAR** connected and working
- Two separate ROS2 workspaces:
  - `ros2_ws`: Main workspace (e.g. localization, navigation)
  - `livox_ros2_ws`: Dedicated to Livox drivers

---

## Workspace Setup

### ROS2 Workspace Setup (`ros2_ws`)

```bash
source /opt/ros/<distro>/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Livox Driver Workspace Setup (`livox_ros2_ws`)

```bash
source /opt/ros/<distro>/setup.bash
source ~/livox_ros2_ws/install/setup.bash
```

Replace `<distro>` with your ROS2 distribution name.

---

## Localization

This mode uses pre-built maps and sensor input to localize the vehicle in real-time.

### Step 1: Launch Platform Bringup

In a terminal sourced with `ros2_ws`:

```bash
ros2 launch fitenth_stack bringup_launch.py
```

### Step 2: Launch Livox Driver

In a terminal sourced with `livox_ros2_ws`:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch_PointCloud2Msg.py
```

**If you want RViz Visualization**

```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```



### Step 3: Launch Localization Pipeline

In a terminal sourced with `ros2_ws`:

```bash
ros2 launch pcl_localization_ros2 pcl_localization.launch.py
```

### Step 4: Example Navigation (Pure Pursuit)

In a terminal sourced with `ros2_ws`:

```bash
ros2 launch pure_pursuit pure_pursuit_launch.py
```

---

## Mapping Mode

This mode builds a map using Fast-LIO and the Livox MID360 sensor. It requires both workspaces to be sourced in the same terminal.
### Step 0: Launch Platform Bringup and Launch livox driver with Timestamp

**Including timestamp with Lidar Data to avoid distortion (this is helpful for mapping but not required for localization)**

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch_CustomMsg.py
```



### Step 1: Launch Fast-LIO Mapping

```bash
source /opt/ros/<distro>/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/livox_ros2_ws/install/setup.bash

ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

### Step 2: Save the Map

After the mapping session, in a new terminal (also sourced with both workspaces):

```bash
source /opt/ros/<distro>/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/livox_ros2_ws/install/setup.bash

ros2 service call /map_save std_srvs/srv/Trigger {}
```

---



---


