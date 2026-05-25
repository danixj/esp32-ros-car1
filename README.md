# README.md

## ROS 2 Wi-Fi Teleoperated Robot (ESP32 + L298)


---

````md
# ROS 2 Wi-Fi Teleoperated Robot (ESP32 + L298)

A beginner-friendly ROS 2 project for controlling a differential drive robot wirelessly using:

- ROS 2 Humble
- ESP32
- L298 Motor Driver
- teleop_twist_keyboard
- UDP Wi-Fi Communication

---

# 1. Project Overview

This project allows a robot to be controlled from a laptop keyboard using ROS 2.

The keyboard commands are sent through ROS2 topics, converted into Wi-Fi UDP packets, and transmitted to the ESP32 which drives the motors using the L298 motor driver.

---

# 2. System Architecture

Keyboard
   ↓
teleop_twist_keyboard
   ↓
/cmd_vel Topic
   ↓
ROS2 Wi-Fi Bridge (Python)
   ↓
UDP over Wi-Fi
   ↓
ESP32
   ↓
L298 Motor Driver
   ↓
DC Motors

---

# 3. Hardware Required

- ESP32 Dev Module
- L298 Motor Driver
- 2 × DC Motors
- Robot Chassis
- 12V Battery
- Jumper Wires
- Ubuntu 22.04 Laptop
- ROS 2 Humble Installed

---

# 4. Important L298 Wiring

| L298 Pin | ESP32 |
|----------|--------|
| IN1 | GPIO 26 |
| IN2 | GPIO 27 |
| IN3 | GPIO 12 |
| IN4 | GPIO 13 |

### IMPORTANT

ENA and ENB must be HIGH.

Otherwise motors will NOT move.

You can:
- Connect ENA → 5V
- Connect ENB → 5V

---

# 5. Workspace Name

This project uses:

~/ros2_ws1

---

# 6. Create ROS2 Workspace

Open terminal:

```bash
mkdir -p ~/ros2_ws1/src

cd ~/ros2_ws1

colcon build
````

---

# 7. Source ROS2

Every new terminal requires:

```bash
source /opt/ros/humble/setup.bash
```

---

# 8. Create ROS2 Package

Go to src folder:

```bash
cd ~/ros2_ws1/src
```

Create package:

```bash
ros2 pkg create wifi_bridge \
--build-type ament_python \
--dependencies rclpy geometry_msgs
```

---

# 9. Final Project Structure

```text
ros2_ws1/
 ├── src/
 │    └── wifi_bridge/
 │         ├── wifi_bridge/
 │         │    ├── __init__.py
 │         │    └── cmdvel_to_wifi.py
 │         │
 │         ├── package.xml
 │         ├── setup.py
 │         └── setup.cfg
 │
 ├── build/
 ├── install/
 └── log/
```

---

# 10. Python Wi-Fi Bridge Code

File:

```text
~/ros2_ws1/src/wifi_bridge/wifi_bridge/cmdvel_to_wifi.py
```

Code:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

ESP32_IP = "192.168.1.100"
ESP32_PORT = 4210

class CmdVelToWiFi(Node):

    def __init__(self):

        super().__init__('cmdvel_to_wifi')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.get_logger().info("WiFi Bridge Started")

    def cmd_callback(self, msg):

        linear = msg.linear.x
        angular = msg.angular.z

        left = linear - angular
        right = linear + angular

        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        data = f"{left:.2f},{right:.2f}"

        self.sock.sendto(
            data.encode(),
            (ESP32_IP, ESP32_PORT)
        )

        self.get_logger().info(
            f"LEFT: {left:.2f} RIGHT: {right:.2f}"
        )

def main(args=None):

    rclpy.init(args=args)

    node = CmdVelToWiFi()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

# 11. setup.py

File:

```text
~/ros2_ws1/src/wifi_bridge/setup.py
```

Code:

```python
from setuptools import setup

package_name = 'wifi_bridge'

setup(
    name=package_name,
    version='0.0.0',

    packages=[package_name],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='user',

    maintainer_email='user@gmail.com',

    description='ROS2 WiFi Bridge',

    license='Apache License 2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [

            'cmdvel_wifi = wifi_bridge.cmdvel_to_wifi:main'

        ],
    },
)
```

---

# 12. setup.cfg

File:

```text
~/ros2_ws1/src/wifi_bridge/setup.cfg
```

Code:

```ini
[develop]
script_dir=$base/lib/wifi_bridge

[install]
install_scripts=$base/lib/wifi_bridge
```

---

# 13. Build Workspace

Go workspace:

```bash
cd ~/ros2_ws1
```

Build:

```bash
colcon build
```

---

# 14. Source Workspace

IMPORTANT:

```bash
source install/setup.bash
```

---

# 15. Install Teleop Keyboard

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

---

# 16. ESP32 Arduino Code

```cpp
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";

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

  Serial.println(WiFi.localIP());
}

void driveMotor(int in1, int in2, float speed) {

  if (speed > 0) {

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

  }

  else if (speed < 0) {

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

  }

  else {

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void loop() {

  int packetSize = udp.parsePacket();

  if (packetSize) {

    char incoming[32];

    udp.read(incoming, 32);

    float left;
    float right;

    sscanf(incoming, "%f,%f", &left, &right);

    driveMotor(IN1, IN2, left);

    driveMotor(IN3, IN4, right);
  }
}
```

---

# 17. Running the Robot

## TERMINAL 1 — Teleop

```bash
source /opt/ros/humble/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## TERMINAL 2 — Wi-Fi Bridge

```bash
source /opt/ros/humble/setup.bash

source ~/ros2_ws1/install/setup.bash

ros2 run wifi_bridge cmdvel_wifi
```

---

## TERMINAL 3 — Debug

```bash
source /opt/ros/humble/setup.bash

ros2 topic echo /cmd_vel
```

---

# 18. Teleop Keys

| Key | Action   |
| --- | -------- |
| i   | Forward  |
| ,   | Backward |
| j   | Left     |
| l   | Right    |
| k   | Stop     |

---

# 19. Important ROS2 Concepts

## Why multiple terminals?

Each ROS2 node usually runs in its own terminal.

Example:

* Teleop node
* Wi-Fi bridge node
* Debug node

---

## Why source setup.bash repeatedly?

Every new terminal starts fresh.

So ROS2 environment variables must be loaded again.

---

# 20. Common Beginner Errors

| Error                     | Reason                  |
| ------------------------- | ----------------------- |
| ros2: command not found   | Forgot ROS source       |
| Package not found         | Forgot workspace source |
| Motors not moving         | ENA/ENB not HIGH        |
| ESP32 not responding      | Wrong IP                |
| Code changes not updating | Forgot colcon build     |

---

# 21. After Modifying Python Code

Always run:

```bash
cd ~/ros2_ws1

colcon build

source install/setup.bash
```

---

# 22. Future Improvements

* PWM Speed Control
* Encoder Feedback
* Odometry
* LiDAR Mapping
* ROS Navigation Stack
* SLAM
* Autonomous Navigation

---

# 23. Final Flow

Keyboard
↓
ROS2 Teleop
↓
/cmd_vel
↓
Wi-Fi Bridge
↓
UDP Wi-Fi
↓
ESP32
↓
L298
↓
Motors

---

```
```
