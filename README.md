# Vision Subsystem

This template provides a partially completed WPILib Java project for a vision subsystem.  
Programmers are expected to **fill in the missing code** to

1. Set up the Vision Simulation   

    a. Set up the simulation camera's properties in the `Constants` file.

    b. Add the camera to the Vision Simulation.

2. View the Vision Simulator.
3. Be able to read the `TX` and `TY` values from `SmartDashboard` using `Elastic`.

The remainder of the document is a guide to how to get your simulation running.

---

### ▶️ 1. Run the Robot in Simulation

Once you’ve filled in the required code, you can **run the robot in simulation** using the WPILib extension:

1. Open the Command Palette in VS Code: `Ctrl+Shift+P` (Windows) / `Cmd+Shift+P` (Mac).
2. Select `WPILib: Simulate Robot Code`.
3. Choose the option: **Sim GUI**.

---

### 🎮 2. Set Up Robot Control

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
- `A` = Drive Left  
- `D` = Drive Right

---

### 🔍 3. Open Elastic and Add Data

Elastic is used to obtain and read data provided by the robot.

1. Launch **Elastic**.
2. Right-click on the grey-shaded boxes and click `Add Widget`.
3. Click the `SmartDashboard` drop-down box.
4. Scroll all the way down until you see the `TX` and `TY` variables corresponding to your camera.
5. Drag the variables onto the grey-shaded boxes to be able to see the data.
---

### 🗺️ 4. Open the Vision Simulator

1. In your web-brower, type in the URL: `localhost:1182` (this is for the middle camera, increment by two for additional cameras. E.g: `localhost:1184`)
2. The Vision Simulator should be visible! Move the robot around and see the values change!
