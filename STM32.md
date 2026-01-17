Ah! So you're using **STM32** instead of OpenCR board to control the motors. That changes things!

Let me adjust the guide for **STM32-based motor control**.

---

# **UPDATED SYSTEM ARCHITECTURE: STM32 Motor Controller**

```
┌──────────────────────────┐
│       LAPTOP             │
│  - OpenTCS               │
│  - Bridge Script         │
│    Publishes /cmd_vel    │
└────────────┬─────────────┘
             │ Ethernet (ROS2)
             ▼
┌──────────────────────────┐
│    RASPBERRY PI 4        │
│  - ROS2 Node             │
│  - Receives /cmd_vel     │
│  - Sends to STM32        │
│  - Publishes /odom       │
└────────────┬─────────────┘
             │ Serial/UART
             │ (USB or GPIO)
             ▼
┌──────────────────────────┐
│       STM32              │
│  - Receives commands     │
│  - Motor PWM control     │
│  - Encoder reading       │
│  - Sends position back   │
└────────┬────────┬────────┘
         │        │
         ▼        ▼
    ┌───────┐ ┌───────┐
    │Motor 1│ │Motor 2│
    │(Left) │ │(Right)│
    └───────┘ └───────┘
```

---

# **COMMUNICATION PROTOCOL: Raspberry Pi ↔ STM32**

You need to establish a **serial communication protocol** between Raspberry Pi and STM32.

---

## **OPTION 1: Simple ASCII Protocol (Easiest)**

### **Message Format:**

**Raspberry Pi → STM32 (Motor Commands):**

```
V,<left_velocity>,<right_velocity>\n
```

**Example:**

```
V,0.15,0.15\n    // Both motors forward at 0.15 m/s
V,0.10,-0.10\n   // Left forward, right backward (rotate)
V,0.00,0.00\n    // Stop
```

**STM32 → Raspberry Pi (Odometry Feedback):**

```
O,<x>,<y>,<theta>\n
```

**Example:**

```
O,1.234,0.567,1.570\n    // Position: x=1.234m, y=0.567m, theta=1.570rad
```

---

## **OPTION 2: Binary Protocol (More Efficient)**

### **Message Format:**

**Structure:**

```c
// Motor command (RPi → STM32)
typedef struct {
    uint8_t header;        // 0xAA
    float left_velocity;   // m/s
    float right_velocity;  // m/s
    uint8_t checksum;
} __attribute__((packed)) MotorCommand;

// Odometry feedback (STM32 → RPi)
typedef struct {
    uint8_t header;        // 0xBB
    float x;               // meters
    float y;               // meters
    float theta;           // radians
    uint8_t checksum;
} __attribute__((packed)) OdometryData;
```

---

# **STEP-BY-STEP IMPLEMENTATION**

---

## **PART 1: STM32 Firmware**

### **1.1 STM32 Main Loop**

```c
// main.c on STM32

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Robot parameters
#define WHEEL_RADIUS 0.033        // meters (33mm)
#define WHEEL_BASE 0.16           // meters (160mm)
#define ENCODER_RESOLUTION 360    // pulses per revolution

// Variables
float robot_x = 0.0;
float robot_y = 0.0;
float robot_theta = 0.0;

float left_velocity = 0.0;
float right_velocity = 0.0;

char rx_buffer[64];
uint8_t rx_index = 0;

// Function prototypes
void parseCommand(char* cmd);
void sendOdometry(void);
void updateOdometry(void);
void controlMotors(float left_vel, float right_vel);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // Initialize peripherals
    MX_GPIO_Init();
    MX_USART2_UART_Init();  // UART to Raspberry Pi
    MX_TIM1_Init();         // PWM for motors
    MX_TIM2_Init();         // Encoder left wheel
    MX_TIM3_Init();         // Encoder right wheel

    // Start timers
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // Left motor
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // Right motor
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // Left encoder
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  // Right encoder

    // Enable UART receive interrupt
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[rx_index], 1);

    uint32_t last_odom_time = HAL_GetTick();

    while (1)
    {
        // Send odometry at 10 Hz
        if (HAL_GetTick() - last_odom_time >= 100) {
            updateOdometry();
            sendOdometry();
            last_odom_time = HAL_GetTick();
        }

        // Control motors based on received velocities
        controlMotors(left_velocity, right_velocity);
    }
}

// UART receive callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        if (rx_buffer[rx_index] == '\n') {
            rx_buffer[rx_index] = '\0';
            parseCommand(rx_buffer);
            rx_index = 0;
        } else {
            rx_index++;
            if (rx_index >= 63) rx_index = 0;  // Buffer overflow protection
        }

        HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[rx_index], 1);
    }
}

// Parse command from Raspberry Pi
void parseCommand(char* cmd)
{
    if (cmd[0] == 'V') {
        // Format: V,left_vel,right_vel
        char* token = strtok(cmd, ",");
        token = strtok(NULL, ",");
        if (token) left_velocity = atof(token);

        token = strtok(NULL, ",");
        if (token) right_velocity = atof(token);
    }
}

// Send odometry to Raspberry Pi
void sendOdometry(void)
{
    char odom_msg[64];
    sprintf(odom_msg, "O,%.3f,%.3f,%.3f\n", robot_x, robot_y, robot_theta);
    HAL_UART_Transmit(&huart2, (uint8_t*)odom_msg, strlen(odom_msg), 100);
}

// Update odometry from encoders
void updateOdometry(void)
{
    static int32_t last_left_count = 0;
    static int32_t last_right_count = 0;

    // Read encoder counts
    int32_t left_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t right_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);

    // Calculate delta
    int32_t delta_left = left_count - last_left_count;
    int32_t delta_right = right_count - last_right_count;

    last_left_count = left_count;
    last_right_count = right_count;

    // Convert to distance
    float left_distance = (delta_left / (float)ENCODER_RESOLUTION) * (2 * 3.14159 * WHEEL_RADIUS);
    float right_distance = (delta_right / (float)ENCODER_RESOLUTION) * (2 * 3.14159 * WHEEL_RADIUS);

    // Calculate robot movement
    float distance = (left_distance + right_distance) / 2.0;
    float delta_theta = (right_distance - left_distance) / WHEEL_BASE;

    // Update position
    robot_theta += delta_theta;
    robot_x += distance * cos(robot_theta);
    robot_y += distance * sin(robot_theta);
}

// Control motors with velocity
void controlMotors(float left_vel, float right_vel)
{
    // Convert velocity (m/s) to PWM duty cycle (0-1000)
    // Adjust these constants based on your motor characteristics
    #define MAX_VELOCITY 0.5  // m/s
    #define PWM_MAX 1000

    int16_t left_pwm = (int16_t)((left_vel / MAX_VELOCITY) * PWM_MAX);
    int16_t right_pwm = (int16_t)((right_vel / MAX_VELOCITY) * PWM_MAX);

    // Clamp PWM values
    if (left_pwm > PWM_MAX) left_pwm = PWM_MAX;
    if (left_pwm < -PWM_MAX) left_pwm = -PWM_MAX;
    if (right_pwm > PWM_MAX) right_pwm = PWM_MAX;
    if (right_pwm < -PWM_MAX) right_pwm = -PWM_MAX;

    // Set motor direction and PWM
    if (left_pwm >= 0) {
        HAL_GPIO_WritePin(MOTOR_LEFT_DIR_GPIO_Port, MOTOR_LEFT_DIR_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, left_pwm);
    } else {
        HAL_GPIO_WritePin(MOTOR_LEFT_DIR_GPIO_Port, MOTOR_LEFT_DIR_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -left_pwm);
    }

    if (right_pwm >= 0) {
        HAL_GPIO_WritePin(MOTOR_RIGHT_DIR_GPIO_Port, MOTOR_RIGHT_DIR_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_pwm);
    } else {
        HAL_GPIO_WritePin(MOTOR_RIGHT_DIR_GPIO_Port, MOTOR_RIGHT_DIR_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -right_pwm);
    }
}
```

---

## **PART 2: Raspberry Pi ROS2 Node**

### **2.1 Create Custom ROS2 Node**

```bash
# On Raspberry Pi
cd ~
mkdir -p robot_ws/src
cd robot_ws/src
ros2 pkg create --build-type ament_python stm32_robot_driver
cd stm32_robot_driver
```

### **2.2 Create Driver Node**

```bash
nano stm32_robot_driver/stm32_driver_node.py
```

**Paste this code:**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial
import threading
import math

class STM32RobotDriver(Node):
    def __init__(self):
        super().__init__('stm32_robot_driver')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        # Open serial connection to STM32
        try:
            self.serial = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f'✅ Connected to STM32 on {serial_port}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to open serial port: {e}')
            return

        # ROS2 publishers and subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Robot parameters (adjust based on your robot!)
        self.wheel_base = 0.16  # meters between wheels

        # Start serial reading thread
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()

        self.get_logger().info('STM32 Robot Driver started')

    def cmd_vel_callback(self, msg):
        """Convert Twist to differential drive commands"""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Calculate wheel velocities
        left_vel = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_vel = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # Send to STM32
        command = f"V,{left_vel:.3f},{right_vel:.3f}\n"
        try:
            self.serial.write(command.encode())
            self.get_logger().debug(f'Sent: {command.strip()}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')

    def read_serial(self):
        """Read odometry from STM32"""
        while self.running and rclpy.ok():
            try:
                line = self.serial.readline().decode('utf-8').strip()

                if line.startswith('O,'):
                    # Parse odometry: O,x,y,theta
                    parts = line.split(',')
                    if len(parts) == 4:
                        x = float(parts[1])
                        y = float(parts[2])
                        theta = float(parts[3])

                        # Publish odometry
                        self.publish_odometry(x, y, theta)

            except Exception as e:
                self.get_logger().debug(f'Serial read error: {e}')

    def publish_odometry(self, x, y, theta):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # Position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        # Orientation (convert theta to quaternion)
        odom.pose.pose.orientation = self.theta_to_quaternion(theta)

        self.odom_pub.publish(odom)

    def theta_to_quaternion(self, theta):
        """Convert yaw angle to quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(theta / 2.0)
        q.w = math.cos(theta / 2.0)
        return q

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = STM32RobotDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### **2.3 Setup Package**

```bash
cd ~/robot_ws/src/stm32_robot_driver

nano setup.py
```

**Update the entry_points:**

```python
entry_points={
    'console_scripts': [
        'stm32_driver_node = stm32_robot_driver.stm32_driver_node:main',
    ],
},
```

---

### **2.4 Build and Install**

```bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash

# Add to bashrc
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
```

---

## **PART 3: Hardware Connections**

### **Connection Options:**

**Option 1: USB (Easiest)**

```
STM32 USB ──────► Raspberry Pi USB port
Device: /dev/ttyUSB0 or /dev/ttyACM0
```

**Option 2: GPIO UART**

```
STM32 TX (PA9)  ──────► RPi GPIO 15 (RXD)
STM32 RX (PA10) ──────► RPi GPIO 14 (TXD)
STM32 GND       ──────► RPi GND
Device: /dev/ttyAMA0 or /dev/serial0
```

---

## **PART 4: Complete System Test**

### **Terminal 1 - Raspberry Pi: Launch STM32 Driver**

```bash
ssh YOUR_RPI_USERNAME@YOUR_RPI_IP
source ~/.bashrc

# Check which serial port STM32 is on
ls /dev/ttyUSB* /dev/ttyACM*

# Launch driver
ros2 run stm32_robot_driver stm32_driver_node --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=115200
```

---

### **Terminal 2 - Laptop: Test Communication**

```bash
# Check topics
ros2 topic list
# Should see: /cmd_vel, /odom

# Test moving robot
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# Watch odometry
ros2 topic echo /odom
```

---

### **Terminal 3 - Laptop: Run OpenTCS Bridge**

```bash
cd ~
python3 opentcs_real_robot_bridge.py
```

**The bridge script from before works without changes!** It just publishes `/cmd_vel` and subscribes to `/odom`.

---

## **UPDATED DATA FLOW**

```
OpenTCS (Laptop)
    ↓ HTTP
Bridge Script (Laptop)
    ↓ ROS2 /cmd_vel
STM32 Driver Node (RPi)
    ↓ Serial: "V,0.15,0.15\n"
STM32 Firmware
    ↓ PWM signals
Motors spin
    ↓ Encoders
STM32 reads position
    ↓ Serial: "O,1.23,0.56,1.57\n"
STM32 Driver Node (RPi)
    ↓ ROS2 /odom
Bridge Script (Laptop)
    ↓ Position update
Loop continues...
```

---

## **QUICK START CHECKLIST**

1. ✅ Flash STM32 with firmware above
2. ✅ Connect STM32 to Raspberry Pi (USB or UART)
3. ✅ Build ROS2 driver on Raspberry Pi
4. ✅ Test serial communication manually
5. ✅ Launch STM32 driver node
6. ✅ Test with manual `/cmd_vel` commands
7. ✅ Run OpenTCS bridge on laptop
8. ✅ Command robot from OpenTCS GUI

---

**Questions:**

1. How are you connecting STM32 to Raspberry Pi? (USB or GPIO UART?)
2. What STM32 model? (STM32F4, STM32F1, etc.)
3. Do you already have firmware, or do you need help with that?
4. What motor driver are you using? (L298N, TB6612, etc.)

Let me know and I'll help you get this working! 🤖
