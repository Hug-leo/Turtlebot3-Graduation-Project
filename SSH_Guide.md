You’re right 👍
What you pasted is **good**, but it is **not complete and not structured enough** to be a _repeatable reference_.
Below is a **FULL, CLEAN, PRODUCTION-GRADE tutorial** that **combines**:

- ✅ Clean SSH (headless, key-only)
- ✅ Static IP (Laptop ↔ Pi via Ethernet)
- ✅ Internet sharing (Laptop → Pi)
- ✅ DNS fixes
- ✅ ROS 2 Humble installation on **Raspberry Pi 4 (Ubuntu 22.04 arm64)**
- ✅ ROS 2 multi-machine (Laptop ↔ Pi)
- ✅ ALL common errors you hit + fixes

This is the **final version** you can reuse **from zero anytime**.

---

# 🧠 MASTER REFERENCE

## Raspberry Pi 4 + Ubuntu 22.04 Server + ROS 2 Humble

## Headless, Ethernet, Laptop as Gateway

---

# 🔴 ASSUMPTIONS (VERY IMPORTANT)

- Raspberry Pi 4
- Ubuntu Server **22.04 LTS (arm64)**
- NO monitor / keyboard
- Ethernet Pi ↔ Laptop
- Laptop already has **ROS 2 Humble**
- Laptop has internet (Wi-Fi)

---

# 🧹 PART A — CLEAN SLATE (SD CARD ON LAPTOP)

## A1. Mount SD card

You must see:

```
/media/hug/system-boot
/media/hug/writable
```

---

## A2. REMOVE all SSH overrides (CRITICAL)

```bash
sudo rm -f /media/hug/writable/etc/ssh/sshd_config.d/*.conf
ls /media/hug/writable/etc/ssh/sshd_config.d
```

✅ Must be **empty**

---

## A3. Reset SSH config

```bash
sudo cp /media/hug/writable/etc/ssh/sshd_config \
        /media/hug/writable/etc/ssh/sshd_config.backup
sudo nano /media/hug/writable/etc/ssh/sshd_config
```

Append **ONLY this at the end**:

```conf
PubkeyAuthentication yes
AuthorizedKeysFile %h/.ssh/authorized_keys
PasswordAuthentication no
KbdInteractiveAuthentication no
UsePAM yes
PermitRootLogin no
```

Save.

---

# 🌐 PART B — STATIC IP ON PI (ETHERNET)

## B1. Disable cloud-init networking

```bash
sudo mkdir -p /media/hug/writable/etc/cloud/cloud.cfg.d
sudo nano /media/hug/writable/etc/cloud/cloud.cfg.d/99-disable-network-config.cfg
```

```yaml
network: { config: disabled }
```

---

## B2. Netplan (Pi)

```bash
sudo rm -f /media/hug/writable/etc/netplan/*.yaml
sudo nano /media/hug/writable/etc/netplan/01-eth-static.yaml
```

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.10.2/24
      gateway4: 192.168.10.1
      nameservers:
        addresses: [8.8.8.8, 1.1.1.1]
```

Permissions (IMPORTANT):

```bash
sudo chmod 600 /media/hug/writable/etc/netplan/01-eth-static.yaml
sudo chown root:root /media/hug/writable/etc/netplan/01-eth-static.yaml
```

---

# 👤 PART C — USER & SSH KEYS (CORRECT)

## C1. Find username

```bash
grep '/home' /media/hug/writable/etc/passwd
```

Assume:

```
dinhsieu
```

---

## C2. Fix home permissions

```bash
sudo chown 1000:1003 /media/hug/writable/home/dinhsieu
sudo chmod 755 /media/hug/writable/home/dinhsieu
```

---

## C3. SSH directory

```bash
sudo rm -rf /media/hug/writable/home/dinhsieu/.ssh
sudo mkdir -p /media/hug/writable/home/dinhsieu/.ssh
sudo chmod 700 /media/hug/writable/home/dinhsieu/.ssh
sudo chown 1000:1003 /media/hug/writable/home/dinhsieu/.ssh
```

---

## C4. Generate key (Laptop)

```bash
ssh-keygen -t ed25519
```

Press ENTER for everything.

---

## C5. Install key into Pi

```bash
sudo cp ~/.ssh/id_ed25519.pub \
/media/hug/writable/home/dinhsieu/.ssh/authorized_keys

sudo chmod 600 /media/hug/writable/home/dinhsieu/.ssh/authorized_keys
sudo chown 1000:1003 \
/media/hug/writable/home/dinhsieu/.ssh/authorized_keys
```

---

# 🔐 PART D — SSH SERVICE CHECK

```bash
ls /media/hug/writable/usr/sbin/sshd
ls /media/hug/writable/etc/ssh/ssh_host_*
```

If SSH not enabled:

```bash
sudo ln -s /lib/systemd/system/ssh.service \
/media/hug/writable/etc/systemd/system/multi-user.target.wants/ssh.service
```

---

## D1. Unmount SD

```bash
sync
sudo umount /media/hug/system-boot
sudo umount /media/hug/writable
```

---

# 🚀 PART E — FIRST BOOT

1. Insert SD into Pi
2. Ethernet Pi ↔ Laptop
3. Power ON
4. Wait 2 minutes

```bash
ssh dinhsieu@192.168.10.2
```

✅ No password
✅ Instant login

---

# 🌍 PART F — LAPTOP NETWORK (GATEWAY)

## F1. Static IP on Laptop Ethernet

```bash
ip link show
sudo nano /etc/netplan/01-network-manager-all.yaml
```

```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    enp43s0: # YOUR ethernet
      dhcp4: no
      addresses:
        - 192.168.10.1/24
      nameservers:
        addresses: [8.8.8.8, 1.1.1.1]
```

```bash
sudo netplan apply
```

---

## F2. Enable internet sharing

```bash
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o wlp0s20f3 -j MASQUERADE
sudo iptables -A FORWARD -i wlp0s20f3 -o enp43s0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i enp43s0 -o wlp0s20f3 -j ACCEPT
```

---

# 🌐 PART G — VERIFY INTERNET ON PI

```bash
ping -c 3 192.168.10.1
ping -c 3 8.8.8.8
ping -c 3 google.com
```

All must work ✅

---

# 🤖 PART H — INSTALL ROS 2 HUMBLE (PI)

## H1. Locale

```bash
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
```

---

## H2. Repositories

```bash
sudo apt install -y software-properties-common curl gnupg
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" | \
sudo tee /etc/apt/sources.list.d/ros2.list
```

---

## H3. Install ROS

```bash
sudo apt update
sudo apt install -y ros-humble-ros-base \
                   ros-humble-demo-nodes-cpp \
                   ros-humble-rmw-cyclonedds-cpp
```

---

## H4. Environment

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
source ~/.bashrc
```

---

## H5. Test ROS on Pi

```bash
ros2 run demo_nodes_cpp talker
```

---

# 🔗 PART I — ROS 2 MULTI-MACHINE TEST

### Pi:

```bash
ros2 run demo_nodes_cpp talker
```

### Laptop:

```bash
export ROS_DOMAIN_ID=30
ros2 run demo_nodes_cpp listener
```

✅ Messages appear → DONE

---

# 🧯 COMMON ERRORS & FIXES

| Error                      | Fix                                   |
| -------------------------- | ------------------------------------- |
| `ros2: command not found`  | source setup.bash                     |
| `demo_nodes_cpp not found` | install ros-humble-demo-nodes-cpp     |
| CycloneDDS error           | install ros-humble-rmw-cyclonedds-cpp |
| No topics                  | ROS_DOMAIN_ID mismatch                |
| Internet broken            | NAT rules missing                     |

---

# ✅ YOU NOW HAVE

- Clean SSH
- Static IP
- Internet sharing
- ROS 2 Humble on Pi
- Laptop ↔ Pi ROS communication
- A **reusable, correct reference**

---

If you want, next I can give you:

- ✅ TurtleBot3 real robot bringup
- ✅ micro-ROS with STM32
- ✅ ROS 2 project template
- ✅ Backup script (one-command recovery)

Just tell me 👍
