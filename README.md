# Leap Motion Gesture Recognition

## Installation

### Install Required Libraries
Install ROS.

Follow the instructions to install the necessary libraries of Leap:
[LeapC Python Bindings](https://github.com/ultraleap/leapc-python-bindings/tree/main)

## Running on Windows Linux Subsystem (WLS)

1. Open **PowerShell as Administrator**
2. Run the following command to list USB devices:
   ```powershell
   usbipd list
   ```
3. Identify the USB device ID you need to use.
4. Bind the USB device using:
   ```powershell
   usbipd bind --busid 4-4
   ```
   *(Replace `4-4` with the correct bus ID from the list.)*

## Building and Running in Ubuntu

### Build the Executable
1. Navigate to the `colcon_ws` workspace:
   ```bash
   cd colcon_ws
   ```
2. Build the project with:
   ```bash
   colcon build --symlink-install
   ```
3. Source the setup file:
   ```bash
   . install/setup.bash
   ```

### Run the Application
To launch the applications, use the following commands:

```bash
ros2 run leap_gesture_recognition leap_gesture_recognition
ros2 run leap_motion leap_motion
```

### View Topic Output
To visualize the output of the topic, run:

```bash
ros2 topic echo /leapmotion/joints
```


## Tracked Hand Joint Data
The system tracks 3D positions for key hand joints, structured as follows:

- **Palm & Wrist:** Palm position, rotation (quaternion), and wrist position.
- **Fingers:** Each finger (thumb, index, middle, ring, pinky) has:
  - Metacarpal
  - Proximal
  - Intermediate
  - Distal
  
Each joint is represented by its `x`, `y`, and `z` coordinates.

### Example Naming Convention
- `left_palm_x`, `left_palm_y`, `left_palm_z`
- `left_thumb_proximal_x`, `left_thumb_proximal_y`, `left_thumb_proximal_z`
- `right_index_distal_x`, `right_index_distal_y`, `right_index_distal_z`

This structure applies to both hands (`left_` and `right_`).
