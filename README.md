# ROS 2 Wi‑Fi Teleoperated Robot (ESP32 + L298)

This document explains **step‑by‑step** how to build a **wireless differential drive robot** using:

* **ROS 2 Humble (Ubuntu 22.04)**
* **teleop_twist_keyboard**
* **ESP32 (Wi‑Fi communication)**
* **L298 motor driver**
* **2 DC yellow hobby motors**

This guide is written so **any beginner can reproduce the project from scratch**.

---

## 1. System Architecture

```
Keyboard
   ↓
teleop_twist_keyboard (ROS2)
   ↓   /cmd_vel
ROS Wi‑Fi Bridge (Python)
   ↓   UDP (Wi‑Fi)
ESP32
   ↓
L298 Motor Driver
   ↓
DC Motors
```

---

## 2. Hardware Required

* ESP32 Dev Module
* L298 Motor Driver
* 2 × DC Yellow Motors
* Robot chassis
* 12V battery (motors)
* Jumper wires
* Common GND between ESP32 & L298

---

## 3. Important Hardware Wiring (CRITICAL)

### L298 Connections

| L298 Pin | ESP32 / Power             |
| -------- | ------------------------- |
| IN1      | GPIO 26                   |
| IN2      | GPIO 27                   |
| IN3      | GPIO 12                   |
| IN4      | GPIO 13                   |
| ENA      | **SHORT TO 5V / ENABLED** |
| ENB      | **SHORT TO 5V / ENABLED** |
| 12V      | Battery +                 |
| GND      | Battery − & ESP32 GND     |

⚠️ **IMPORTANT DISCOVERY**

> Motors did NOT move until **ENA and ENB were tied HIGH**.
>
> L298 will NOT output motor voltage unless ENA/ENB are enabled.

---

## 4. ROS 2 Installation Check

Open a terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

If commands work → ROS 2 is installed correctly.

---

## 5. Create ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 6. Install Teleop Keyboard

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

Run teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use keys:

* `i` → forward
* `,` → backward
* `j / l` → turn

---

## 7. Create Wi‑Fi Bridge Package

```bash
cd ~/ros2_ws/src
ros2 pkg create wifi_bridge --build-type ament_python --dependencies rclpy geometry_msgs
```

Folder structure:

```
wifi_bridge/
 ├── wifi_bridge/
 │   └── cmdvel_to_wifi.py
 ├── setup.py
 └── package.xml
```

---

## 8. ROS Wi‑Fi Bridge Code

### cmdvel_to_wifi.py

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

ESP32_IP = "192.168.1.100"   # Change to your ESP32 IP
ESP32_PORT = 4210

class CmdVelToWiFi(Node):
    def __init__(self):
        super().__init__('cmdvel_to_wifi')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        left = linear - angular
        right = linear + angular

        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        data = f"{left:.2f},{right:.2f}"
        self.sock.sendto(data.encode(), (ESP32_IP, ESP32_PORT))
        self.get_logger().info(f"L:{left:.2f} R:{right:.2f}")


def main():
    rclpy.init()
    node = CmdVelToWiFi()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 9. setup.py

```python
from setuptools import setup

package_name = 'wifi_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'cmdvel_wifi = wifi_bridge.cmdvel_to_wifi:main'
        ],
    },
)
```

---

## 10. Build the Package

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 11. ESP32 Arduino Code

```cpp
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "YOUR_WIFI";
const char* password = "YOUR_PASSWORD";

WiFiUDP udp;
unsigned int localPort = 4210;

int IN1 = 26;
int IN2 = 27;
int IN3 = 12;
int IN4 = 13;

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  udp.begin(localPort);
}

void driveMotor(int in1, int in2, float speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char data[32];
    udp.read(data, 32);

    float left, right;
    sscanf(data, "%f,%f", &left, &right);

    driveMotor(IN1, IN2, left);
    driveMotor(IN3, IN4, right);
  }
}
```

⚠️ **ENA and ENB must be tied HIGH for motors to work**

---

## 12. Running the System (3 Terminals)

### Terminal 1 – Teleop

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Terminal 2 – Wi‑Fi Bridge

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run wifi_bridge cmdvel_wifi
```

### Terminal 3 – Debug (Optional)

```bash
ros2 topic echo /cmd_vel
```

---

## 13. Final Result

✅ Keyboard controls robot wirelessly
✅ ROS 2 → ESP32 over Wi‑Fi
✅ Differential drive robot working

---

## 14. Key Learning

* L298 **requires ENA / ENB HIGH**
* ESP32 is **3.3V logic** (important!)
* Always verify motor driver enable pins
* ROS debugging → hardware debugging is essential

---

## 15. Future Improvements

* PWM speed control on ENA / ENB
* Odometry feedback
* ROS Navigation stack
* LiDAR mapping

---

**Project Successfully Completed 🚀**
