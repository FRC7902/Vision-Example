# Vision Subsystem

This template provides a partially completed WPILib Java project for a differential (tank-style) drive robot.  
Programmers are expected to **fill in the missing code** to

1. Set up the Drive Subsystem   

    a. Set up motor controllers

    b. Bind the motors to arcade drive controls

2. Set up an Arcade Drive command
3. Bind a command to a joystick controller

Finally, you should be able to simulate the robot movement on both a 2D field and a 3D field. The remainder of the document is a guide to how to get your simulation running.

### ▶️ 1. Run the Robot in Simulation

Once you’ve filled in the required code, you can **run the robot in simulation** using the WPILib extension:

1. Open the Command Palette in VS Code: `Ctrl+Shift+P` (Windows) / `Cmd+Shift+P` (Mac).
2. Select `WPILib: Simulate Robot Code`.
3. Choose the option: **Sim GUI**.

---

### 🔍 2. Open AdvantageScope (AS)

AdvantageScope is used to visualize and interact with the simulated robot.

1. Launch **AdvantageScope**.
2. Go to **File > Connect to Simulator** — this connects to the simulation running from WPILib.

---

### 🗺️ 3. Visualize in 2D Field ("Odometry" UI)

1. In AS, switch to the **"Odometry"** tab at the top.
2. In the left sidebar, locate the data key named **`MyPose`**.
3. Drag `MyPose` into the **“Poses”** section at the bottom — the robot should now appear on the 2D field.

---

### 🎮 4. Set Up Robot Control

To control the robot simulation:

1. In AS, open the **"Robot Simulation"** window (usually on the right or bottom).
2. Under **Joysticks**, find the `Joysticks[0]` slot.
3. Drag **Keyboard 0** into `Joysticks[0]` (unless you have a physical controller connected).
   - `Joysticks[0]`, `[1]`, etc., represent the slots used in your robot code (e.g., `new Joystick(0)`).
   - Binding `Keyboard 0` simulates a gamepad using your keyboard.
4. Under **Robot State**, click **“Teleoperated”** to enable the robot.

### 🕹️ Default Keyboard Controls

- `W` = Drive Forward  
- `S` = Drive Backward  
- `A` = Rotate Counter-Clockwise (left)  
- `D` = Rotate Clockwise (right)

---

### 🧱 5. Visualize in 3D Field

To simulate the robot in a 3D environment:

1. In AdvantageScope, switch to the **"3D Field"** UI (select from the top tab bar).
2. Click on the field model selector, and choose **"Kitbot 2025"**.
3. Your simulated robot should now appear and move around on a 3D model of the FRC field!
