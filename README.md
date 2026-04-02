# Flight IMU 3D Visualizer
 
Real-time 3D aircraft orientation tracking using an Arduino Uno R4 WiFi and a GY-87 IMU module. Tilt the sensor board and watch a 3D model mirror its orientation in your browser.
 
![Arduino](https://img.shields.io/badge/Arduino-Uno_R4_WiFi-00979D?logo=arduino)
![License](https://img.shields.io/badge/license-MIT-blue)
 
## How It Works
 
The GY-87's MPU6050 (accelerometer + gyroscope) and QMC5883L (magnetometer) are fused with a complementary filter at 500 Hz — the same core approach used in flight controllers like Betaflight. The Arduino streams roll/pitch/yaw over serial, and a browser-based Three.js viewer reads it via the Web Serial API.
 
- **Roll & Pitch**: Gyro integration corrected by accelerometer tilt (complementary filter, α = 0.98)
- **Yaw**: Gyro integration corrected by tilt-compensated magnetometer heading (drift-free)
 
## Hardware
 
| Component | Notes |
|---|---|
| Arduino Uno R4 WiFi | Any R4 variant works |
| GY-87 10DOF IMU | MPU6050 + QMC5883L + BMP180 |
| Breadboard + 4 jumper wires | Included in SunFounder Elite Explorer Kit |
 
### Wiring
 
```
GY-87        Arduino
─────        ───────
VCC_IN  ───  5V
GND     ───  GND
SCL     ───  A5 (SCL)
SDA     ───  A4 (SDA)
```
 
No resistors or extra components needed — the GY-87 has onboard pull-ups and a voltage regulator.
 
## Software Setup
 
### Arduino
 
1. Install these libraries via Arduino IDE Library Manager:
   - **Adafruit MPU6050** (install all dependencies when prompted)
   - **QMC5883LCompass** by MPrograms
 
2. Open `flight_3d_visualizer.ino`, upload to your board.
 
3. Verify output in Serial Monitor at **115200 baud**:
   ```
   R:-0.21,P:-1.91,Y:-84.09,AX:0.34,AY:-0.01,AZ:10.57
   ```
 
### 3D Viewer
 
1. **Close the Serial Monitor** (only one program can use the COM port).
2. Open `flight_deck_3d.html` in **Chrome or Edge** (Web Serial requires a Chromium browser).
3. Click **Connect**, select your Arduino's COM port.
4. Tilt the GY-87 — the aircraft follows.
 
## Serial Protocol
 
The Arduino outputs one line per cycle at ~500 Hz:
 
```
R:<roll>,P:<pitch>,Y:<yaw>,AX:<ax>,AY:<ay>,AZ:<az>
```
 
| Field | Unit | Description |
|---|---|---|
| R | degrees | Roll (right wing down = positive) |
| P | degrees | Pitch (nose up = positive) |
| Y | degrees | Yaw / heading (clockwise = positive) |
| AX, AY, AZ | m/s² | Raw accelerometer axes |
 
Lines starting with `#` are status messages and are ignored by the viewer.
 
## Magnetometer Calibration
 
For accurate yaw, calibrate the QMC5883L before first use:
 
1. Upload the SunFounder calibration sketch (`09-gy87_compass_calibration.ino` from the Elite Explorer Kit examples).
2. Rotate the board in a figure-8 pattern when prompted.
3. Copy the output calibration offsets into `flight_3d_visualizer.ino` where indicated in the comments.
 
## Project Structure
 
```
├── flight_3d_visualizer.ino   # Arduino sketch (sensor fusion + serial output)
├── flight_deck_3d.html        # Browser 3D viewer (Three.js + Web Serial)
└── README.md
```
 
## License
 
MIT
